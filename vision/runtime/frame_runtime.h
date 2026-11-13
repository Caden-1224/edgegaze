// EdgeGaze RDK X5 帧源与有界帧队列。
// 仅负责帧的获取、时间戳和队列语义，不包含推理或对外通信。
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace edgegaze::runtime {

// 一帧图像及其时间信息。data 为 BGR8 原始像素，多个消费者可共享。
struct Frame {
  // 采集或回放时间轴上的单调时间戳；禁止用推理返回时刻填写。
  uint64_t timestamp_us{0};
  // 回放文件内的确定性媒体时间：frame_index * 1e6 / fps。
  uint64_t media_time_us{0};
  uint32_t sequence{0};
  int width{0};
  int height{0};
  std::shared_ptr<const std::vector<uint8_t>> data;
};

enum class FrameReadStatus { Ok, EndOfStream, Error };

// 帧源配置：camera=true 打开 V4L2 摄像头，否则打开视频文件。
struct FrameSourceConfig {
  bool camera{false};
  std::string source;  // /dev/video0 或视频文件路径
  double fps{30.0};    // 仅回放用于计算媒体时间
  int width{0};        // 0 表示使用设备/文件原始尺寸
  int height{0};
};

// 摄像头与视频回放共用的帧源接口。
class FrameSource {
 public:
  virtual ~FrameSource() = default;
  // 读取下一帧；流结束返回 EndOfStream，打开失败或解码错误返回 Error。
  virtual FrameReadStatus read(Frame& frame) = 0;
  // 释放设备或文件句柄，可在 read 阻塞时调用。
  virtual void stop() = 0;
};

// 按配置创建帧源；失败时返回 nullptr 并通过 error 返回原因。
std::unique_ptr<FrameSource> open_frame_source(const FrameSourceConfig& config,
                                               std::string* error = nullptr);

struct FrameQueueStats {
  uint64_t pushed{0};
  uint64_t popped{0};
  uint64_t dropped_oldest{0};  // 因队满丢弃的最旧帧数
  std::size_t peak_size{0};
  std::size_t capacity{0};
};

// 固定容量、线程安全的帧环形队列。push 不等待空间；队列满时覆盖最旧帧。
class BoundedFrameQueue {
 public:
  explicit BoundedFrameQueue(std::size_t capacity);
  ~BoundedFrameQueue();
  BoundedFrameQueue(const BoundedFrameQueue&) = delete;
  BoundedFrameQueue& operator=(const BoundedFrameQueue&) = delete;

  // 入队一帧，不阻塞；返回 true 表示为了腾位置丢弃了最旧帧。
  bool push(Frame frame);
  // 阻塞等待最多 timeout，取出最旧帧；关闭且队列为空时返回 false。
  bool pop(Frame& frame, std::chrono::milliseconds timeout);
  // 唤醒所有等待者，之后不再接受 push。
  void close();
  FrameQueueStats stats() const;

 private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<Frame> slots_;  // 固定槽位，避免每次入队分配队列节点
  std::size_t head_{0};       // 下一个写入位置
  std::size_t tail_{0};       // 最旧可读位置
  std::size_t size_{0};
  std::size_t capacity_;
  bool closed_{false};
  FrameQueueStats stats_;
};

// 帧管道配置：一个帧源、一个有界队列和可选的统计文件。
struct FramePipelineConfig {
  FrameSourceConfig source;
  std::size_t queue_capacity{8};
  std::string stats_path;  // 为空表示不落盘
};

// 负责把 FrameSource 的帧搬到 BoundedFrameQueue 的后台采集管道。
// 生命周期：start() -> pop()/queue_stats() -> stop()；析构会自动 stop()。
class FramePipeline {
 public:
  explicit FramePipeline(FramePipelineConfig config);
  ~FramePipeline();
  FramePipeline(const FramePipeline&) = delete;
  FramePipeline& operator=(const FramePipeline&) = delete;

  // 打开帧源并启动采集线程；失败返回 false。
  bool start(std::string* error = nullptr);
  // 停止采集、关闭队列，并按配置追加一行统计 JSONL；可重复调用。
  void stop();
  // 从内部队列取帧；超时或队列已关闭返回 false。
  bool pop(Frame& frame, std::chrono::milliseconds timeout);
  FrameQueueStats queue_stats() const;

 private:
  void capture_loop();
  void write_stats();

  FramePipelineConfig config_;
  std::unique_ptr<FrameSource> source_;
  BoundedFrameQueue queue_;
  std::thread thread_;
  bool started_{false};
  bool stopped_{false};
  std::atomic<bool> running_{false};  // 采集线程的退出标志
};

// 单调微秒时钟，供所有帧时间戳使用。
uint64_t monotonic_now_us();

}  // namespace edgegaze::runtime
