#include "frame_runtime.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <utility>

#if defined(EDGEGAZE_WITH_OPENCV)
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#endif

namespace edgegaze::runtime {
namespace {

// 把源路径转义成可安全写入 JSON 字符串的形式。
std::string json_escape(const std::string& value) {
  std::string out;
  for (char c : value) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      default: out += c; break;
    }
  }
  return out;
}

#if defined(EDGEGAZE_WITH_OPENCV)
// 把 OpenCV Mat 统一转成连续 BGR8 字节，并填入 Frame。
bool mat_to_frame(const cv::Mat& input, Frame* frame) {
  cv::Mat bgr;
  if (input.channels() == 1) {
    cv::cvtColor(input, bgr, cv::COLOR_GRAY2BGR);
  } else if (input.channels() == 4) {
    cv::cvtColor(input, bgr, cv::COLOR_BGRA2BGR);
  } else {
    bgr = input;
  }
  if (bgr.depth() != CV_8U) {
    cv::Mat converted;
    bgr.convertTo(converted, CV_8U);
    bgr = converted;
  }

  const std::size_t bytes =
      static_cast<std::size_t>(bgr.total() * bgr.elemSize());
  auto buffer = std::make_shared<std::vector<uint8_t>>(bytes);
  if (bgr.isContinuous()) {
    std::memcpy(buffer->data(), bgr.data, bytes);
  } else {
    const std::size_t row_bytes =
        static_cast<std::size_t>(bgr.cols * bgr.elemSize());
    for (int row = 0; row < bgr.rows; ++row) {
      std::memcpy(buffer->data() + static_cast<std::size_t>(row) * row_bytes,
                  bgr.ptr(row), row_bytes);
    }
  }

  frame->width = bgr.cols;
  frame->height = bgr.rows;
  frame->data = std::move(buffer);
  return true;
}

// 同一个 OpenCV 实现覆盖视频文件与 V4L2 摄像头；差异只在 open() 的打开方式。
class OpenCvSource final : public FrameSource {
 public:
  explicit OpenCvSource(FrameSourceConfig config) : config_(std::move(config)) {}

  ~OpenCvSource() override { stop(); }

  // 打开设备/文件并应用分辨率与帧率配置。
  bool open(std::string* error) {
    if (config_.camera) {
      capture_.open(config_.source, cv::CAP_V4L2);
    } else {
      capture_.open(config_.source);
    }
    if (!capture_.isOpened()) {
      if (error) *error = "cannot open frame source: " + config_.source;
      return false;
    }
    if (config_.width > 0) capture_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
    if (config_.height > 0) capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
    if (config_.fps > 0 && config_.camera) capture_.set(cv::CAP_PROP_FPS, config_.fps);
    if (!(config_.fps > 0)) config_.fps = 30.0;
    start_us_ = monotonic_now_us();
    return true;
  }

  // 回放使用媒体时间轴，摄像头使用采集时刻；两者都不依赖推理完成时间。
  FrameReadStatus read(Frame& frame) override {
    if (!capture_.isOpened()) return FrameReadStatus::Error;
    cv::Mat mat;
    if (!capture_.read(mat) || mat.empty()) return FrameReadStatus::EndOfStream;

    const uint64_t capture_us = monotonic_now_us();
    if (!mat_to_frame(mat, &frame)) return FrameReadStatus::Error;

    if (config_.camera) {
      frame.timestamp_us = capture_us;
      frame.media_time_us = capture_us;
    } else {
      frame.media_time_us = static_cast<uint64_t>(
          std::llround(static_cast<double>(frame_index_) * 1000000.0 / config_.fps));
      frame.timestamp_us = start_us_ + frame.media_time_us;
    }
    frame.sequence = static_cast<uint32_t>(frame_index_++);
    return FrameReadStatus::Ok;
  }

  // 释放 VideoCapture；下一次 read 会因为没有打开而返回 Error。
  void stop() override {
    if (capture_.isOpened()) capture_.release();
  }

 private:
  FrameSourceConfig config_;
  cv::VideoCapture capture_;
  uint64_t start_us_{0};
  uint64_t frame_index_{0};
};
#endif

}  // namespace

uint64_t monotonic_now_us() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

// 无 OpenCV 构建返回空源，让调用方得到明确错误而不是静默丢帧。
std::unique_ptr<FrameSource> open_frame_source(const FrameSourceConfig& config,
                                               std::string* error) {
#if defined(EDGEGAZE_WITH_OPENCV)
  auto source = std::make_unique<OpenCvSource>(config);
  if (!source->open(error)) return nullptr;
  return source;
#else
  (void)config;
  if (error) *error = "OpenCV is required for camera and video replay";
  return nullptr;
#endif
}

// capacity 必须为正；槽位提前分配，运行时 push 不再分配队列节点。
BoundedFrameQueue::BoundedFrameQueue(std::size_t capacity)
    : slots_(capacity), capacity_(capacity) {
  if (capacity_ == 0) {
    throw std::invalid_argument("BoundedFrameQueue capacity must be positive");
  }
  stats_.capacity = capacity_;
}

BoundedFrameQueue::~BoundedFrameQueue() = default;

bool BoundedFrameQueue::push(Frame frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  // 队满时 head == tail；覆盖 head（最旧槽位）后同时前移 head/tail。
  if (closed_) return false;

  bool dropped = false;
  if (size_ == capacity_) {
    slots_[head_] = std::move(frame);
    head_ = (head_ + 1) % capacity_;
    tail_ = head_;
    ++stats_.dropped_oldest;
    dropped = true;
  } else {
    slots_[head_] = std::move(frame);
    head_ = (head_ + 1) % capacity_;
    ++size_;
  }
  ++stats_.pushed;
  stats_.peak_size = std::max(stats_.peak_size, size_);
  cv_.notify_one();
  return dropped;
}

bool BoundedFrameQueue::pop(Frame& frame, std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(mutex_);
  // close() 后即使队列为空也要立刻唤醒等待者。
  if (!cv_.wait_for(lock, timeout,
                    [this] { return closed_ || size_ > 0; })) {
    return false;
  }
  if (size_ == 0) return false;
  frame = std::move(slots_[tail_]);
  tail_ = (tail_ + 1) % capacity_;
  --size_;
  ++stats_.popped;
  return true;
}

void BoundedFrameQueue::close() {
  std::lock_guard<std::mutex> lock(mutex_);
  closed_ = true;
  cv_.notify_all();
}

FrameQueueStats BoundedFrameQueue::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

FramePipeline::FramePipeline(FramePipelineConfig config)
    : config_(std::move(config)), queue_(config_.queue_capacity) {}

FramePipeline::~FramePipeline() { stop(); }

bool FramePipeline::start(std::string* error) {
  if (started_) return true;
  source_ = open_frame_source(config_.source, error);
  if (!source_) return false;
  stopped_ = false;
  started_ = true;
  running_.store(true);
  thread_ = std::thread([this] { capture_loop(); });
  return true;
}

void FramePipeline::stop() {
  if (stopped_) return;
  stopped_ = true;
  if (started_) {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
    if (source_) source_->stop();
    started_ = false;
  }
  queue_.close();
  if (source_) write_stats();
}

bool FramePipeline::pop(Frame& frame, std::chrono::milliseconds timeout) {
  return queue_.pop(frame, timeout);
}

FrameQueueStats FramePipeline::queue_stats() const { return queue_.stats(); }

// 采集线程只做 read + push；队列满时的丢弃策略由 BoundedFrameQueue 决定。
void FramePipeline::capture_loop() {
  while (running_.load()) {
    Frame frame;
    if (source_->read(frame) != FrameReadStatus::Ok) break;
    queue_.push(std::move(frame));
  }
  queue_.close();
}

// 每次 stop 追加一行 JSONL，schema 固定，便于后续汇总脚本读取。
void FramePipeline::write_stats() {
  if (config_.stats_path.empty()) return;
  const auto stats = queue_.stats();
  const std::filesystem::path path(config_.stats_path);
  if (path.has_parent_path()) {
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
  }
  std::ofstream out(path, std::ios::app);
  if (!out) return;
  out << "{\"schema\":\"edgegaze.frame_pipeline_stats.v1\""
      << ",\"source\":\"" << json_escape(config_.source.source) << "\""
      << ",\"capacity\":" << stats.capacity
      << ",\"pushed\":" << stats.pushed
      << ",\"popped\":" << stats.popped
      << ",\"dropped\":" << stats.dropped_oldest
      << ",\"peak_depth\":" << stats.peak_size
      << ",\"timestamp_us\":" << monotonic_now_us() << "}\n";
}

}  // namespace edgegaze::runtime