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
#include <atomic>
#include <algorithm>
#include <ctime>

using namespace std;

struct time_stamp {
  int64_t high;
  int64_t low;
};
volatile time_stamp *pointt = (volatile time_stamp *)MAP_FAILED;
volatile time_stamp *clock_offset_point = (volatile time_stamp *)MAP_FAILED;

bool exit_flag = false;
int width = 640, height = 480; // 默认分辨率，可从参数调整
image_transport::Publisher pub;
float image_scale = 1.0;
int trigger_enable = 1;
string device_name = "/dev/video0"; // 默认设备
string pixel_format = "YUYV";
int frame_rate = 0;
int timestamp_wait_ms = 30;
string timestamp_source = "v4l2_lidar_clock";
string clock_offset_path = "/home/nuc/mid360_clock_offset";
int64_t timestamp_offset_us = 0;
int64_t last_timeshare_seq = 0;
int64_t last_camera_stamp_ns = 0;
uint64_t hardware_stamp_count = 0;
uint64_t fallback_stamp_count = 0;
uint64_t duplicate_frame_drop_count = 0;

bool ReadCommittedValue(volatile time_stamp *source, int64_t &seq,
                        int64_t &value) {
  if (source == MAP_FAILED) return false;
  for (int attempt = 0; attempt < 4; ++attempt) {
    const int64_t seq_before = source->high;
    std::atomic_thread_fence(std::memory_order_acquire);
    const int64_t committed_value = source->low;
    std::atomic_thread_fence(std::memory_order_acquire);
    const int64_t seq_after = source->high;
    if (seq_before == seq_after && seq_after > 0 &&
        (seq_after & 1) == 0) {
      seq = seq_after;
      value = committed_value;
      return true;
    }
    std::this_thread::yield();
  }
  return false;
}

bool ReadCommittedTimestamp(int64_t &seq, int64_t &stamp_ns) {
  return ReadCommittedValue(pointt, seq, stamp_ns) && stamp_ns > 0;
}

int64_t TimespecToNs(const struct timespec &ts) {
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

bool V4L2CaptureHostRealtimeNs(const struct v4l2_buffer &buf,
                              int64_t &host_realtime_ns) {
  const int64_t buffer_ns = static_cast<int64_t>(buf.timestamp.tv_sec) *
                                1000000000LL +
                            static_cast<int64_t>(buf.timestamp.tv_usec) * 1000LL;
  if (buffer_ns <= 0) return false;

  if ((buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
      V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
    struct timespec realtime_now, monotonic_now;
    if (clock_gettime(CLOCK_REALTIME, &realtime_now) != 0 ||
        clock_gettime(CLOCK_MONOTONIC, &monotonic_now) != 0) {
      return false;
    }
    host_realtime_ns = buffer_ns + TimespecToNs(realtime_now) -
                       TimespecToNs(monotonic_now);
  } else {
    host_realtime_ns = buffer_ns;
  }
  return true;
}

bool WaitForFreshHardwareTimestamp(int64_t &seq, int64_t &stamp_ns) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timestamp_wait_ms);
  do {
    if (ReadCommittedTimestamp(seq, stamp_ns) && seq != last_timeshare_seq) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (std::chrono::steady_clock::now() < deadline);
  return false;
}

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
  if (!Params["TimestampWaitMs"].empty())
    timestamp_wait_ms = std::max(0, (int)Params["TimestampWaitMs"]);
  if (!Params["TimestampSource"].empty())
    timestamp_source = (string)Params["TimestampSource"];
  if (!Params["ClockOffsetPath"].empty())
    clock_offset_path = (string)Params["ClockOffsetPath"];
  if (!Params["TimestampOffsetUs"].empty())
    timestamp_offset_us = (int64_t)(int)Params["TimestampOffsetUs"];

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

  // Generic V4L2 auto exposure does not provide controls for these bounds.
  // Keep the YAML values informational instead of claiming they were applied.
  if (exposure_auto_mode == 1) { // Auto mode (Aperture Priority)
    ROS_WARN("Auto exposure enabled; ExposureTimeLower/Upper (%d/%d) are not applied by this UVC driver",
             exposure_time_lower, exposure_time_upper);
  }

  // 如果是手动曝光，设置曝光时间
  if (exposure_auto_mode == 0) { // Manual mode
      if (!set_v4l2_control(fd, V4L2_CID_EXPOSURE_ABSOLUTE, exposure)) {
        ROS_ERROR("Failed to set exposure time to %d", exposure);
      } else {
        ROS_INFO("Set manual exposure to %d V4L2 units (%.3f ms)",
                 exposure, exposure * 0.1);
      }
    }

  // 设置backlight为2，实现硬件触发
  if (!set_v4l2_control(fd, V4L2_CID_BACKLIGHT_COMPENSATION, 2)) {
    ROS_ERROR("Failed to set backlight for hardware trigger");
  } else {
    ROS_INFO("Set backlight to 2 for hardware trigger");
  }

    // Sharpness is an image-processing control, not exposure time.
    if (!set_v4l2_control(fd, V4L2_CID_SHARPNESS, sharpness)) {
      ROS_ERROR("Failed to set image sharpness to %d", sharpness);
    } else {
      ROS_INFO("Set image sharpness to %d", sharpness);
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
    int64_t timestamp_ns = 0;
    if (timestamp_source == "v4l2_lidar_clock") {
      int64_t capture_host_ns = 0;
      int64_t offset_seq = 0;
      int64_t host_minus_lidar_ns = 0;
      if (!V4L2CaptureHostRealtimeNs(buf, capture_host_ns) ||
          !ReadCommittedValue(clock_offset_point, offset_seq,
                              host_minus_lidar_ns)) {
        ++duplicate_frame_drop_count;
        ROS_WARN_THROTTLE(
            5.0,
            "Dropping camera frame: V4L2 capture time or MID360 clock offset "
            "is unavailable (dropped=%lu, flags=0x%x)",
            duplicate_frame_drop_count, buf.flags);
        ioctl(fd, VIDIOC_QBUF, &buf);
        continue;
      }
      timestamp_ns = capture_host_ns - host_minus_lidar_ns +
                     timestamp_offset_us * 1000LL;
      ++hardware_stamp_count;
    } else if (timestamp_source == "lidar_legacy" && trigger_enable &&
               pointt != MAP_FAILED) {
      int64_t seq = 0;
      if (!WaitForFreshHardwareTimestamp(seq, timestamp_ns)) {
        ++duplicate_frame_drop_count;
        ROS_WARN_THROTTLE(
            5.0,
            "Dropping camera frame without a fresh LiDAR hardware timestamp "
            "(dropped=%lu, last_seq=%ld, wait_ms=%d)",
            duplicate_frame_drop_count, last_timeshare_seq, timestamp_wait_ms);
        ioctl(fd, VIDIOC_QBUF, &buf);
        continue;
      }
      last_timeshare_seq = seq;
      ++hardware_stamp_count;
    } else {
      if (timestamp_source == "v4l2_ros") {
        if (!V4L2CaptureHostRealtimeNs(buf, timestamp_ns)) {
          timestamp_ns = ros::Time::now().toNSec();
        }
      } else {
        timestamp_ns = ros::Time::now().toNSec();
      }
      ++fallback_stamp_count;
      ROS_WARN_THROTTLE(
          5.0,
          "Camera timestamp source '%s' is not hardware-domain synchronized "
          "(fallback=%lu)", timestamp_source.c_str(), fallback_stamp_count);
    }

    if (timestamp_ns <= last_camera_stamp_ns) {
      ++duplicate_frame_drop_count;
      ROS_WARN_THROTTLE(
          5.0, "Dropping non-monotonic camera timestamp: current=%ld last=%ld",
          timestamp_ns, last_camera_stamp_ns);
      ioctl(fd, VIDIOC_QBUF, &buf);
      continue;
    }
    last_camera_stamp_ns = timestamp_ns;
    rcv_time.fromNSec(timestamp_ns);

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
    ROS_INFO_STREAM_THROTTLE(
        10.0, "Camera timestamp counters: hardware=" << hardware_stamp_count
              << " fallback=" << fallback_stamp_count
              << " dropped=" << duplicate_frame_drop_count
              << " source=" << timestamp_source
              << " v4l2_flags=0x" << std::hex << buf.flags << std::dec);

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
  if (!Params["TimestampSource"].empty())
    timestamp_source = (string)Params["TimestampSource"];
  if (!Params["ClockOffsetPath"].empty())
    clock_offset_path = (string)Params["ClockOffsetPath"];
  string pub_topic = Params["TopicName"];
  device_name = (string)Params["DeviceName"]; // 从参数文件读取设备名

  pub = it.advertise(pub_topic, 1);

  const char *user_name = getlogin();
  string path_for_time_stamp = "/home/" + string(user_name) + "/timeshare";
  const char *shared_file_name = path_for_time_stamp.c_str();

  int fd_time = open(shared_file_name, O_RDWR);
  if (fd_time < 0) {
    ROS_WARN("Unable to open %s; camera timestamps will fall back to ROS time", shared_file_name);
    pointt = (volatile time_stamp *)MAP_FAILED;
  } else {
    pointt = (volatile time_stamp *)mmap(
        NULL, sizeof(time_stamp), PROT_READ, MAP_SHARED, fd_time, 0);
    if (pointt == MAP_FAILED) {
      ROS_WARN("Unable to mmap %s; camera timestamps will fall back to ROS time", shared_file_name);
    }
    close(fd_time);
  }

  int fd_offset = open(clock_offset_path.c_str(), O_RDONLY);
  if (fd_offset < 0) {
    ROS_WARN("Unable to open %s; v4l2_lidar_clock timestamps are unavailable",
             clock_offset_path.c_str());
    clock_offset_point = (volatile time_stamp *)MAP_FAILED;
  } else {
    clock_offset_point = (volatile time_stamp *)mmap(
        NULL, sizeof(time_stamp), PROT_READ, MAP_SHARED, fd_offset, 0);
    if (clock_offset_point == MAP_FAILED) {
      ROS_WARN("Unable to mmap %s; v4l2_lidar_clock timestamps are unavailable",
               clock_offset_path.c_str());
    }
    close(fd_offset);
  }

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
  if (pointt != MAP_FAILED) {
    munmap((void *)pointt, sizeof(time_stamp));
  }
  if (clock_offset_point != MAP_FAILED) {
    munmap((void *)clock_offset_point, sizeof(time_stamp));
  }

  return 0;
}
