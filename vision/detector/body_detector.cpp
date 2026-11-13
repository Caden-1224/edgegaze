#include "body_detector.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <utility>

#if defined(EDGEGAZE_WITH_OPENCV) && defined(EDGEGAZE_WITH_BPU)
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "dnn/hb_dnn.h"
#include "dnn/hb_sys.h"
#endif

namespace edgegaze::vision {

std::vector<RawBodyBox> parse_body_box_tensor(const float* data,
                                              std::size_t float_count) {
  std::vector<RawBodyBox> boxes;
  if (data == nullptr || float_count < 6) return boxes;

  const int byte_size = static_cast<int>(data[0]);
  constexpr std::size_t kRecordFloats = 6;
  if (byte_size <= 0) return boxes;

  const std::size_t record_bytes = kRecordFloats * sizeof(float);
  const std::size_t count = static_cast<std::size_t>(byte_size) / record_bytes;
  if (count == 0 || count > (float_count - kRecordFloats) / kRecordFloats) {
    return boxes;
  }

  boxes.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const float* record = data + kRecordFloats + i * kRecordFloats;
    RawBodyBox box;
    box.left = record[0];
    box.top = record[1];
    box.right = record[2];
    box.bottom = record[3];
    box.confidence = record[4];
    box.class_label = record[5];
    boxes.push_back(box);
  }
  return boxes;
}

#if defined(EDGEGAZE_WITH_OPENCV) && defined(EDGEGAZE_WITH_BPU)

namespace {

uint64_t steady_now_us() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

// BGR8 -> NV12（Y 平面 + UV 交错），模型输入要求 NV12_SEPARATE。
void bgr_to_nv12(const cv::Mat& bgr, cv::Mat* nv12) {
  cv::Mat i420;
  cv::cvtColor(bgr, i420, cv::COLOR_BGR2YUV_I420);
  const int width = bgr.cols;
  const int height = bgr.rows;
  nv12->create(height * 3 / 2, width, CV_8UC1);

  std::memcpy(nv12->data, i420.data, static_cast<std::size_t>(width) * height);
  const uint8_t* u = i420.data + static_cast<std::size_t>(width) * height;
  const uint8_t* v = u + static_cast<std::size_t>(width) * height / 4;
  uint8_t* uv = nv12->data + static_cast<std::size_t>(width) * height;
  const int uv_count = width * height / 4;
  for (int i = 0; i < uv_count; ++i) {
    uv[2 * i] = u[i];
    uv[2 * i + 1] = v[i];
  }
}

class HbotBodyDetector final : public BodyDetector {
 public:
  ~HbotBodyDetector() override { release(); }

  bool init(const BodyDetectorConfig& config, std::string* error) override {
    release();
    threshold_ = config.confidence_threshold;

    const char* files[] = {config.model_path.c_str()};
    int ret = hbDNNInitializeFromFiles(&packed_, files, 1);
    if (ret != 0) {
      return fail(error, "hbDNNInitializeFromFiles failed: " + std::to_string(ret));
    }

    char const** names = nullptr;
    int model_count = 0;
    ret = hbDNNGetModelNameList(&names, &model_count, packed_);
    if (ret != 0 || model_count <= 0) {
      release();
      return fail(error, "hbDNNGetModelNameList failed: " + std::to_string(ret));
    }
    model_name_ = names[0];

    ret = hbDNNGetModelHandle(&model_, packed_, model_name_.c_str());
    if (ret != 0) {
      release();
      return fail(error, "hbDNNGetModelHandle failed: " + std::to_string(ret));
    }

    int input_count = 0;
    int output_count = 0;
    if (hbDNNGetInputCount(&input_count, model_) != 0 || input_count != 1 ||
        hbDNNGetOutputCount(&output_count, model_) != 0 || output_count < 9) {
      release();
      return fail(error, "unexpected model tensor count");
    }

    hbDNNTensorProperties input_props;
    if (hbDNNGetInputTensorProperties(&input_props, model_, 0) != 0 ||
        input_props.tensorType != HB_DNN_IMG_TYPE_NV12_SEPARATE ||
        input_props.validShape.numDimensions != 4) {
      release();
      return fail(error, "model input is not NV12_SEPARATE");
    }
    input_width_ = input_props.validShape.dimensionSize[3];
    input_height_ = input_props.validShape.dimensionSize[2];
    if (input_width_ <= 0 || input_height_ <= 0 ||
        (input_width_ % 16) != 0 || (input_height_ % 2) != 0) {
      release();
      return fail(error, "unsupported model input size");
    }

    input_.properties = input_props;
    y_size_ = static_cast<std::size_t>(input_width_) * input_height_;
    uv_size_ = y_size_ / 2;
    if (hbSysAllocCachedMem(&input_.sysMem[0], static_cast<uint32_t>(y_size_)) != 0 ||
        hbSysAllocCachedMem(&input_.sysMem[1], static_cast<uint32_t>(uv_size_)) != 0) {
      release();
      return fail(error, "hbSysAllocCachedMem for input failed");
    }

    output_count_ = output_count;
    outputs_.resize(output_count_);
    for (int i = 0; i < output_count_; ++i) {
      hbDNNTensorProperties props;
      if (hbDNNGetOutputTensorProperties(&props, model_, i) != 0 ||
          hbSysAllocCachedMem(&outputs_[i].sysMem[0],
                              static_cast<uint32_t>(props.alignedByteSize)) != 0) {
        release();
        return fail(error, "allocate output tensor failed");
      }
      outputs_[i].properties = props;
    }

    // 检查人体框输出（index 1）是 float32，并确认记录区能容纳解析结果。
    if (outputs_[1].properties.tensorType != HB_DNN_TENSOR_TYPE_F32) {
      release();
      return fail(error, "body box output is not float32");
    }
    return true;
  }

  bool detect(const runtime::Frame& frame, DetectionResult* result,
              std::string* error) override {
    if (!model_ || !frame.data || frame.width <= 0 || frame.height <= 0) {
      return fail(error, "detector not initialized or invalid frame");
    }
    if (result == nullptr) return fail(error, "result is null");

    result->frame_timestamp_us = frame.timestamp_us;
    const uint64_t preprocess_start = steady_now_us();
    cv::Mat bgr(frame.height, frame.width, CV_8UC3,
                const_cast<uint8_t*>(frame.data->data()));
    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(input_width_, input_height_), 0, 0,
               cv::INTER_LINEAR);
    bgr_to_nv12(resized, &nv12_);

    std::memcpy(input_.sysMem[0].virAddr, nv12_.data, y_size_);
    std::memcpy(input_.sysMem[1].virAddr,
                nv12_.data + static_cast<std::size_t>(input_width_) * input_height_,
                uv_size_);
    hbSysFlushMem(&input_.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    hbSysFlushMem(&input_.sysMem[1], HB_SYS_MEM_CACHE_CLEAN);
    const uint64_t preprocess_end = steady_now_us();

    hbDNNInferCtrlParam ctrl;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&ctrl);
    hbDNNTaskHandle_t task = nullptr;
    hbDNNTensor* output_ptr = outputs_.data();
    const uint64_t infer_start = steady_now_us();
    int ret = hbDNNInfer(&task, &output_ptr, &input_, model_, &ctrl);
    if (ret == 0) ret = hbDNNWaitTaskDone(task, 1000);
    const uint64_t infer_end = steady_now_us();
    if (task != nullptr) hbDNNReleaseTask(task);
    if (ret != 0) {
      return fail(error, "hbDNNInfer failed: " + std::to_string(ret));
    }

    const uint64_t post_start = steady_now_us();
    hbSysFlushMem(&outputs_[1].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
    const float* data =
        static_cast<const float*>(outputs_[1].sysMem[0].virAddr);
    const std::size_t float_count =
        static_cast<std::size_t>(outputs_[1].properties.alignedByteSize) /
        sizeof(float);
    const auto raw_boxes = parse_body_box_tensor(data, float_count);

    result->detections.clear();
    const float width = static_cast<float>(input_width_);
    const float height = static_cast<float>(input_height_);
    for (const auto& raw : raw_boxes) {
      if (raw.confidence < threshold_) continue;
      const float left = std::max(0.0f, std::min(raw.left, width - 1.0f));
      const float top = std::max(0.0f, std::min(raw.top, height - 1.0f));
      const float right = std::max(left, std::min(raw.right, width - 1.0f));
      const float bottom = std::max(top, std::min(raw.bottom, height - 1.0f));
      BodyDetection det;
      det.x = left / width;
      det.y = top / height;
      det.w = (right - left) / width;
      det.h = (bottom - top) / height;
      det.confidence = raw.confidence;
      result->detections.push_back(det);
    }
    const uint64_t post_end = steady_now_us();

    result->timing.preprocess_us = preprocess_end - preprocess_start;
    result->timing.infer_us = infer_end - infer_start;
    result->timing.postprocess_us = post_end - post_start;
    return true;
  }

  std::string model_name() const override { return model_name_; }
  int input_width() const override { return input_width_; }
  int input_height() const override { return input_height_; }

 private:
  static bool fail(std::string* error, const std::string& message) {
    if (error) *error = message;
    return false;
  }

  void release() {
    for (auto& output : outputs_) {
      if (output.sysMem[0].virAddr != nullptr) {
        hbSysFreeMem(&output.sysMem[0]);
        output.sysMem[0].virAddr = nullptr;
      }
    }
    outputs_.clear();
    if (input_.sysMem[0].virAddr != nullptr) {
      hbSysFreeMem(&input_.sysMem[0]);
      input_.sysMem[0].virAddr = nullptr;
    }
    if (input_.sysMem[1].virAddr != nullptr) {
      hbSysFreeMem(&input_.sysMem[1]);
      input_.sysMem[1].virAddr = nullptr;
    }
    if (packed_ != nullptr) {
      hbDNNRelease(packed_);
      packed_ = nullptr;
      model_ = nullptr;
    }
  }

  hbPackedDNNHandle_t packed_{nullptr};
  hbDNNHandle_t model_{nullptr};
  std::string model_name_;
  int input_width_{0};
  int input_height_{0};
  int output_count_{0};
  std::size_t y_size_{0};
  std::size_t uv_size_{0};
  float threshold_{0.3f};
  hbDNNTensor input_{};
  std::vector<hbDNNTensor> outputs_;
  cv::Mat nv12_;
};

}  // namespace

std::unique_ptr<BodyDetector> create_body_detector() {
  return std::make_unique<HbotBodyDetector>();
}

#else

namespace {

class UnavailableBodyDetector final : public BodyDetector {
 public:
  bool init(const BodyDetectorConfig&, std::string* error) override {
    if (error) {
      *error = "BPU detector is not built; configure with "
               "-DEDGEGAZE_WITH_BPU=ON and -DEDGEGAZE_WITH_OPENCV=ON on RDK X5";
    }
    return false;
  }
  bool detect(const runtime::Frame&, DetectionResult*, std::string* error) override {
    if (error) *error = "BPU detector is not built";
    return false;
  }
  std::string model_name() const override { return {}; }
  int input_width() const override { return 0; }
  int input_height() const override { return 0; }
};

}  // namespace

std::unique_ptr<BodyDetector> create_body_detector() {
  return std::make_unique<UnavailableBodyDetector>();
}

#endif

}  // namespace edgegaze::vision
