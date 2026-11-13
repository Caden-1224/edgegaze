// EdgeGaze RDK X5 人体检测基线。
// 只负责单帧 BGR 输入到人体框的推理，不包含跟踪、ReID 与对外通信。
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "frame_runtime.h"

namespace edgegaze::vision {

// 模型 float32 检测框张量中的一条记录（模型输入坐标系，未归一化）。
struct RawBodyBox {
  float left{0};
  float top{0};
  float right{0};
  float bottom{0};
  float confidence{0};
  float class_label{0};
};

// 解析 multitask_body_head_face_hand_kps 模型的检测框张量：
// 前 6 个 float 是张量头（首元素为记录区字节数），之后每条记录 6 个 float。
// 输入不合法时返回空数组，不抛异常。
std::vector<RawBodyBox> parse_body_box_tensor(const float* data,
                                              std::size_t float_count);

// 归一化到原帧 [0,1] 的人体框。
struct BodyDetection {
  float x{0};  // 左上角 x / 原帧宽
  float y{0};
  float w{0};
  float h{0};
  float confidence{0};
};

// 单帧三段耗时，单位微秒。
struct DetectionTiming {
  uint64_t preprocess_us{0};
  uint64_t infer_us{0};
  uint64_t postprocess_us{0};
};

struct DetectionResult {
  // 原帧时间戳，由调用方 Frame 带入；不得用推理完成时刻替代。
  uint64_t frame_timestamp_us{0};
  std::vector<BodyDetection> detections;
  DetectionTiming timing;
};

struct BodyDetectorConfig {
  std::string model_path;
  float confidence_threshold{0.3f};
};

// 单帧人体检测器。未编译 BPU/OpenCV 后端时 init 返回明确错误。
class BodyDetector {
 public:
  virtual ~BodyDetector() = default;

  // 加载模型并分配输入/输出张量；可重复调用前需先析构。
  virtual bool init(const BodyDetectorConfig& config, std::string* error) = 0;

  // 对一帧 BGR8 图像推理；结果坐标归一化到该帧。
  virtual bool detect(const runtime::Frame& frame, DetectionResult* result,
                      std::string* error) = 0;

  virtual std::string model_name() const = 0;
  virtual int input_width() const = 0;
  virtual int input_height() const = 0;
};

// 创建当前构建配置下可用的检测器。
std::unique_ptr<BodyDetector> create_body_detector();

}  // namespace edgegaze::vision
