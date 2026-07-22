#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <signal.h>
#include <cstring>
#include <vector>
#include <chrono>
#include <thread>

using namespace std;

struct time_stamp {
  int64_t high;
  int64_t low;
};
time_stamp *pointt;

bool exit_flag = false;
int width = 640, height = 480; // 默认分辨率，可从参数调整
image_transport::Publisher pub;
float image_scale = 1.0;
int trigger_enable = 1;
string device_name = "/dev/video0"; // 默认设备
string pixel_format = "YUYV";
int frame_rate = 0;
uint64_t stm32_timestamp_count = 0;
uint64_t ros_time_fallback_count = 0;
uint64_t timestamp_total_count = 0;

void SignalHandler(int signal) {
  if (signal == SIGINT) {
    fprintf(stderr, "\nReceived Ctrl+C, exiting...\n");
    exit_flag = true;
  }
}

void SetupSignalHandler() {
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = SignalHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

bool set_v4l2_control(int fd, int id, int value) {
  struct v4l2_control ctrl;
  ctrl.id = id;
  ctrl.value = value;
  if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) < 0) {
    perror("VIDIOC_S_CTRL");
    return false;
  }
  return true;
}

bool get_v4l2_control(int fd, int id, int &value) {
  struct v4l2_control ctrl;
  ctrl.id = id;
  if (ioctl(fd, VIDIOC_G_CTRL, &ctrl) < 0) {
    perror("VIDIOC_G_CTRL");
    return false;
  }
  value = ctrl.value;
  return true;
}

uint32_t pixelFormatFromString(const string &format) {
  if (format == "MJPG" || format == "MJPEG") return V4L2_PIX_FMT_MJPEG;
  if (format == "YUYV") return V4L2_PIX_FMT_YUYV;

  ROS_WARN("Unsupported PixelFormat '%s'. Falling back to YUYV.", format.c_str());
  return V4L2_PIX_FMT_YUYV;
}

bool set_v4l2_frame_rate(int fd, int fps) {
  if (fps <= 0) return true;

  struct v4l2_streamparm parm;
  memset(&parm, 0, sizeof(parm));
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator = fps;

  if (ioctl(fd, VIDIOC_S_PARM, &parm) < 0) {
    perror("VIDIOC_S_PARM");
    return false;
  }

  ROS_INFO("Set camera frame rate to %.3f fps",
           static_cast<double>(parm.parm.capture.timeperframe.denominator) /
               parm.parm.capture.timeperframe.numerator);
  return true;
}

void setParams(int fd, const string &params_file) {
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }
  image_scale = Params["image_scale"];
  if (image_scale < 0.1) image_scale = 1.0;

  width = Params["Width"];
  height = Params["Height"];
  if (!Params["PixelFormat"].empty()) pixel_format = (string)Params["PixelFormat"];
  if (!Params["FrameRate"].empty()) frame_rate = (int)Params["FrameRate"];

  int backlight = Params["Backlight"];
  int exposure_auto_mode = Params["ExposureAutoMode"];
  int exposure = Params["Exposure"];
  int exposure_time_lower = Params["ExposureTimeLower"];
  int exposure_time_upper = Params["ExposureTimeUpper"];
  int sharpness = Params["Sharpness"];
  int gain = Params["Gain"];
  int gain_auto = Params["GainAuto"];

  // 设置自动曝光模式
  int exposure_mode_value;
  std::string exposure_mode_str;
  switch (exposure_auto_mode) {
    case 0: // Manual
      exposure_mode_value = 1; // Manual Mode
      exposure_mode_str = "Manual";
      break;
    case 1: // Auto (使用Aperture Priority作为自动模式)
      exposure_mode_value = 3; // Aperture Priority Mode
      exposure_mode_str = "Aperture Priority (Auto)";
      break;
    case 2: // Aperture Priority
      exposure_mode_value = 3; // Aperture Priority Mode
      exposure_mode_str = "Aperture Priority";
      break;
    default:
      exposure_mode_value = 1; // Manual Mode
      exposure_mode_str = "Manual (default)";
      break;
  }

  if (!set_v4l2_control(fd, V4L2_CID_EXPOSURE_AUTO, exposure_mode_value)) {
    ROS_WARN("Failed to set auto exposure mode to %s", exposure_mode_str.c_str());
  } else {
    ROS_INFO("Set auto exposure mode to %s", exposure_mode_str.c_str());
  }

  // 如果是自动曝光，设置曝光时间范围
  if (exposure_auto_mode == 1) { // Auto mode (Aperture Priority)
    ROS_INFO("Auto exposure enabled with time bounds: %d - %d us", exposure_time_lower, exposure_time_upper);
  }

  // 如果是手动曝光，设置曝光时间
  if (exposure_auto_mode == 0) { // Manual mode
    if (!set_v4l2_control(fd, V4L2_CID_EXPOSURE_ABSOLUTE, exposure)) {
      ROS_ERROR("Failed to set exposure time to %d", exposure);
    } else {
      ROS_INFO("Set exposure time to %d", exposure);
    }
  }

  // 设置backlight为2，实现硬件触发
  if (!set_v4l2_control(fd, V4L2_CID_BACKLIGHT_COMPENSATION, 2)) {
    ROS_ERROR("Failed to set backlight for hardware trigger");
  } else {
    ROS_INFO("Set backlight to 2 for hardware trigger");
  }

  // 设置锐度（在手动曝光模式下用作曝光时间）
  if (!set_v4l2_control(fd, V4L2_CID_SHARPNESS, sharpness)) {
    ROS_ERROR("Failed to set sharpness (exposure time) to %d", sharpness);
  } else {
    ROS_INFO("Set sharpness (exposure time) to %d", sharpness);
  }

  // 设置增益（UVC摄像头通常只支持手动增益）
  if (!set_v4l2_control(fd, V4L2_CID_GAIN, gain)) {
    ROS_ERROR("Failed to set gain to %d", gain);
  } else {
    ROS_INFO("Set gain to %d", gain);
  }

  if (gain_auto == 1) {
    ROS_WARN("Auto gain requested but not supported by this UVC camera. Using manual gain: %d", gain);
  }
}

void *WorkThread(void *arg) {
  int fd = *(int *)arg;
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = width;
  fmt.fmt.pix.height = height;
  fmt.fmt.pix.pixelformat = pixelFormatFromString(pixel_format);
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

  if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
    perror("VIDIOC_S_FMT");
    return NULL;
  }
  set_v4l2_frame_rate(fd, frame_rate);

  char fourcc[5] = {
      static_cast<char>(fmt.fmt.pix.pixelformat & 0xff),
      static_cast<char>((fmt.fmt.pix.pixelformat >> 8) & 0xff),
      static_cast<char>((fmt.fmt.pix.pixelformat >> 16) & 0xff),
      static_cast<char>((fmt.fmt.pix.pixelformat >> 24) & 0xff),
      '\0'};
  ROS_INFO("Camera capture format: %ux%u %s",
           fmt.fmt.pix.width, fmt.fmt.pix.height, fourcc);

  // 请求缓冲区
  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
    perror("VIDIOC_REQBUFS");
    return NULL;
  }

  vector<void *> buffers(req.count);
  vector<size_t> buffer_lengths(req.count);

  for (int i = 0; i < req.count; ++i) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
      perror("VIDIOC_QUERYBUF");
      return NULL;
    }

    buffers[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    if (buffers[i] == MAP_FAILED) {
      perror("mmap");
      return NULL;
    }
    buffer_lengths[i] = buf.length;

    if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
      perror("VIDIOC_QBUF");
      return NULL;
    }
  }

  // 启动捕获
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
    perror("VIDIOC_STREAMON");
    return NULL;
  }

  while (!exit_flag && ros::ok()) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
      perror("VIDIOC_DQBUF");
      continue;
    }

    ros::Time rcv_time;
    if (trigger_enable && pointt != MAP_FAILED && pointt->low != 0) {
      int64_t b = pointt->low;
      double time_pc = b / 1000000000.0;
      rcv_time = ros::Time(time_pc);
    } else {
      rcv_time = ros::Time::now();
    }
    ++timestamp_total_count;
    if (timestamp_total_count <= 20 || timestamp_total_count % 100 == 0) {
      const char *source = (trigger_enable && pointt != MAP_FAILED && pointt->low != 0) ? "STM32_TIMESHARE" : "ROS_TIME_NOW_FALLBACK";
      long long timeshare_low = (pointt != MAP_FAILED) ? static_cast<long long>(pointt->low) : 0LL;
      ROS_INFO("camera timestamp source=%s total=%llu stm32=%llu fallback=%llu trigger_enable=%d timeshare_low=%lld stamp=%.9f",
               source,
               static_cast<unsigned long long>(timestamp_total_count),
               static_cast<unsigned long long>(stm32_timestamp_count),
               static_cast<unsigned long long>(ros_time_fallback_count),
               trigger_enable,
               timeshare_low,
               rcv_time.toSec());
    }

    cv::Mat rgb;
    if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG) {
      cv::Mat mjpeg(1, buf.bytesused, CV_8UC1, buffers[buf.index]);
      cv::Mat bgr = cv::imdecode(mjpeg, cv::IMREAD_COLOR);
      if (bgr.empty()) {
        ROS_WARN_THROTTLE(2.0, "Failed to decode MJPG frame");
        ioctl(fd, VIDIOC_QBUF, &buf);
        continue;
      }
      cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    } else {
      cv::Mat yuyv(height, width, CV_8UC2, buffers[buf.index]);
      cv::cvtColor(yuyv, rgb, cv::COLOR_YUV2RGB_YUYV);
    }

    if (image_scale > 0.0 && image_scale != 1.0) {
      cv::resize(rgb, rgb, cv::Size(rgb.cols * image_scale, rgb.rows * image_scale), cv::INTER_LINEAR);
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", rgb).toImageMsg();
    msg->header.stamp = rcv_time;
    pub.publish(msg);

    if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
      perror("VIDIOC_QBUF");
    }
  }

  // 停止捕获
  if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
    perror("VIDIOC_STREAMOFF");
  }

  for (int i = 0; i < req.count; ++i) {
    munmap(buffers[i], buffer_lengths[i]);
  }

  return NULL;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "uvc_driver");
  string params_file = string(argv[1]);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  ros::Rate loop_rate(10);

  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }

  trigger_enable = Params["TriggerEnable"];
  string pub_topic = Params["TopicName"];
  device_name = (string)Params["DeviceName"]; // 从参数文件读取设备名

  pub = it.advertise(pub_topic, 1);

  const char *user_name = getlogin();
  string path_for_time_stamp = "/home/" + string(user_name) + "/timeshare";
  const char *shared_file_name = path_for_time_stamp.c_str();

  int fd_time = open(shared_file_name, O_RDWR);
  pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE, MAP_SHARED, fd_time, 0);

  SetupSignalHandler();

  int fd = open(device_name.c_str(), O_RDWR);
  if (fd < 0) {
    perror("open device");
    ROS_ERROR("Failed to open device %s. Please check permissions or run with sudo.", device_name.c_str());
    return -1;
  }

  ROS_INFO("Successfully opened device: %s", device_name.c_str());

  setParams(fd, params_file);

  ROS_INFO("Finish all params set! Start grabbing...");

  pthread_t nThreadID;
  int ret = pthread_create(&nThreadID, NULL, WorkThread, &fd);
  if (ret != 0) {
    printf("thread create failed.ret = %d\n", ret);
    return -1;
  }

  while (!exit_flag && ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  if (nThreadID) {
    pthread_join(nThreadID, NULL);
    ROS_INFO_STREAM("Worker thread joined.");
  }

  close(fd);
  munmap(pointt, sizeof(time_stamp));

  return 0;
}
