# Version5：PTS → MID360 离线 a 标定与每次启动 b 重锚

本实现不改 STM32、TriggerEnable、Backlight、LiDAR/IMU、deskew、FAST-LIVO
后端或外参。`img_time_offset` 与 `exposure_time_init` 仍为 0；
`ExposureCenterOffsetNs`、`CameraCalibrationOffsetNs` 也固定为 0。

## 时间模型

相机进程从 UVC metadata 节点读取 32 位 48 MHz PTS，并按模 `2^32` 连续
展开。视频帧与 metadata 帧按相同的 V4L2 buffer `sequence` 一对一配对；
视频出队时最多等待 20 ms 让异步 metadata 线程登记同 sequence 样本。这里的
sequence 只作配对 ID，绝不用于生成 `sequence * 100 ms` 时间。

离线标定只确定并固定比例：

```text
a = 1.0000834459970804
camera_relative_time = unwrap(PTS) / 48e6
```

每次 `STREAMON` 后先等待 0.5 秒，再连续采集 30 秒 PTS/MID360 配对样本：

```text
b_i = mid360_time_i - a * pts_time_i
b_session = median(b_i)
```

30 秒是根据实测选定的最短可靠窗口：3 秒窗口的 drift 估计会被单帧约
1.2 ms 抖动主导，而 30 秒窗口在抽取的 6 个历史时段回放中全部通过原定阈值。

校验通过后，以中心化形式映射，避免大绝对时间数值误差：

```text
camera_time_ns = mid_anchor_ns
               + round(a * (pts_ticks - pts_anchor_ticks) / 48e6 * 1e9)
```

`a` 和 `b_session` 在整个连续流内保持不变，没有运行期拟合、滑窗或平滑器。
PTS 无法唯一配对时丢弃该视频帧，不使用旧 b、`ros::Time::now()` 或
`sequence * 100 ms` 作为后备时间源。

## 1. 离线标定固定比例 a

5–10 分钟数据仍可用于复核比例：

```bash
cd /home/nuc/versions/version5
bash tools/run_pts_mid360_stage.sh calibration 600
```

拟合采用中心化模型：

```text
x_i = pts_time_i - pts_time_0
y_i = mid360_time_i - mid360_time_0
y_i = a*x_i + c
b_ns = mid360_ns_0 + round(c*1e9 - a*pts_time_0*1e9)
```

正式拟合使用 `--warmup-s 2` 排除开流首帧瞬态。拟合脚本可输出 a、b、skew
ppm、residual mean/std/absolute P95/absolute max 和 drift，但写运行配置时只
保留比例 a，并强制：

```text
CameraClockScale: 1.0000834459970804
CameraClockOffsetNs: "0"
CameraClockCalibrationValid: 1
TimestampSource: "pts_mid360_session_anchor"
```

旧的 `pts_mid360_fixed` 和 SIGHUP 激活流程已禁用，绝不复用上一条流的绝对 b。

## 2. 每次 STREAMON 自动重锚

默认配置：

```text
SessionAnchorWarmupMs: 500
SessionAnchorCollectMs: 30000
SessionAnchorMinPairs: 20
SessionAnchorMaxAbsMeanMs: 1.0
SessionAnchorMaxP95Ms: 3.0
SessionAnchorMaxAbsDriftMsPerS: 0.01
SessionAnchorPtsResetMissingFrames: 5
```

只有样本数、mean、P95、drift 和 accepted duplicate/rollback/sequence jump
计数全部通过，驱动才发布 `/uvc_camera/session_anchor_locked=true` 并开始发布
图像。`start_slam1` 默认最多等待 75 秒，并在锁定后才启动 FAST-LIVO。

校验失败会终止相机启动，不回退其他时间源。Camera reopen、重新 `STREAMON`
或连续 5 帧缺失 PTS 会使旧锚失效，并完整重做 0.5+30 秒流程。

## 3. 固定会话映射验证

相机静止运行 120 秒：

```bash
bash tools/run_pts_mid360_stage.sh static 120
```

按正常手持方式运动 180 秒：

```bash
bash tools/run_pts_mid360_stage.sh motion 180
```

两个阶段必须接在同一次会话锚定后的连续相机流内。报告包含 residual、drift、
PTS/mapped timestamp duplicate/rollback、sequence jump、mapped Camera period，
以及从 `FASTLIVO_CUT_STATS` 提取的 LiDAR backlog。

```bash
python3 tools/summarize_pts_mid360_validations.py \
  logs/pts_mid360/<static目录>/static.json \
  logs/pts_mid360/<motion目录>/motion.json \
  --output-json logs/pts_mid360/final_comparison.json
```

## 4. 默认映射判据

固定绝对 b 隐含 Camera PTS 与 MID360 的时钟原点在重启后仍保持相同关系。
本机 2026-09-08 实测重开流后，复用旧 b 会令 Camera 比 MID360 落后约
26.4 秒，因此绝不允许把绝对 b 跨会话设为默认值。

Version5 默认固定 a，但每次开流自动生成并冻结 `b_session`；这消除了重启
epoch 变化，同时没有引入运行期在线更新。
