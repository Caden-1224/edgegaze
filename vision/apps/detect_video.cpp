// EdgeGaze RDK X5 检测基线命令行：视频回放 -> 人体检测 -> JSONL + 标注视频 + 统计。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include "body_detector.h"
#include "frame_runtime.h"

namespace {

struct Options {
  std::string video;
  std::string model;
  std::string out_dir{"results"};
  std::string jsonl_path;
  std::string stats_path;
  std::string annotated_path;
  double fps{10.0};
  int max_frames{0};
  int queue_capacity{8};
  float confidence{0.3f};
};

void print_usage() {
  std::cout
      << "usage: edgegaze_detect_video --video VIDEO --model MODEL.hbm "
         "[--out-dir DIR] [--fps FPS] [--max-frames N] [--queue-capacity N] "
         "[--confidence C]\n";
}

bool parse_args(int argc, char** argv, Options* options, std::string* error) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        *error = std::string("missing value for ") + name;
        return {};
      }
      return argv[++i];
    };
    try {
      if (arg == "--video") {
        options->video = next("--video");
      } else if (arg == "--model") {
        options->model = next("--model");
      } else if (arg == "--out-dir") {
        options->out_dir = next("--out-dir");
      } else if (arg == "--jsonl") {
        options->jsonl_path = next("--jsonl");
      } else if (arg == "--stats") {
        options->stats_path = next("--stats");
      } else if (arg == "--annotated") {
        options->annotated_path = next("--annotated");
      } else if (arg == "--fps") {
        options->fps = std::stod(next("--fps"));
      } else if (arg == "--max-frames") {
        options->max_frames = std::stoi(next("--max-frames"));
      } else if (arg == "--queue-capacity") {
        options->queue_capacity = std::stoi(next("--queue-capacity"));
      } else if (arg == "--confidence") {
        options->confidence = std::stof(next("--confidence"));
      } else if (arg == "-h" || arg == "--help") {
        print_usage();
        std::exit(0);
      } else {
        *error = "unknown argument: " + arg;
        return false;
      }
    } catch (const std::exception& e) {
      *error = std::string("invalid value for ") + arg + ": " + e.what();
      return false;
    }
  }
  if (options->video.empty() || options->model.empty()) {
    *error = "--video and --model are required";
    return false;
  }
  if (options->fps <= 0 || options->queue_capacity <= 0 ||
      options->confidence < 0 || options->confidence > 1) {
    *error = "invalid fps/queue-capacity/confidence";
    return false;
  }
  return true;
}

std::string json_escape(const std::string& value) {
  std::string out;
  for (const char c : value) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      default: out += c; break;
    }
  }
  return out;
}

double percentile(const std::vector<double>& values, double p) {
  if (values.empty()) return 0.0;
  std::vector<double> sorted = values;
  std::sort(sorted.begin(), sorted.end());
  const double rank = std::ceil(p / 100.0 * static_cast<double>(sorted.size()));
  const std::size_t index = static_cast<std::size_t>(
      std::max(1.0, std::min(rank, static_cast<double>(sorted.size()))) - 1);
  return sorted[index];
}

std::string default_stem(const std::string& video) {
  const std::filesystem::path path(video);
  const std::string stem = path.stem().string();
  return stem.empty() ? "video" : stem;
}

bool write_stats(const std::string& path, const Options& options,
                 const std::string& model_name, int input_width,
                 int input_height, std::size_t frames,
                 std::size_t detections_total,
                 std::size_t frames_with_detections,
                 const std::vector<double>& preprocess_ms,
                 const std::vector<double>& infer_ms,
                 const std::vector<double>& postprocess_ms,
                 const std::vector<double>& total_ms,
                 const edgegaze::runtime::FrameQueueStats& queue) {
  const std::filesystem::path out_path(path);
  if (out_path.has_parent_path()) {
    std::error_code ec;
    std::filesystem::create_directories(out_path.parent_path(), ec);
  }
  std::ofstream out(path);
  if (!out) return false;
  out << std::fixed << std::setprecision(3)
      << "{\"schema\":\"edgegaze.body_detection_stats.v1\""
      << ",\"model\":{\"name\":\"" << json_escape(model_name) << "\""
      << ",\"path\":\"" << json_escape(options.model) << "\""
      << ",\"input_width\":" << input_width
      << ",\"input_height\":" << input_height << "}"
      << ",\"video\":\"" << json_escape(options.video) << "\""
      << ",\"fps\":" << options.fps
      << ",\"frames\":" << frames
      << ",\"detections_total\":" << detections_total
      << ",\"frames_with_detections\":" << frames_with_detections
      << ",\"timing_ms\":{\"preprocess\":{\"p50\":"
      << percentile(preprocess_ms, 50.0) << ",\"p95\":"
      << percentile(preprocess_ms, 95.0)
      << "},\"infer\":{\"p50\":" << percentile(infer_ms, 50.0)
      << ",\"p95\":" << percentile(infer_ms, 95.0)
      << "},\"postprocess\":{\"p50\":" << percentile(postprocess_ms, 50.0)
      << ",\"p95\":" << percentile(postprocess_ms, 95.0)
      << "},\"total\":{\"p50\":" << percentile(total_ms, 50.0)
      << ",\"p95\":" << percentile(total_ms, 95.0) << "}}"
      << ",\"queue\":{\"capacity\":" << queue.capacity
      << ",\"pushed\":" << queue.pushed << ",\"popped\":" << queue.popped
      << ",\"dropped\":" << queue.dropped_oldest
      << ",\"peak_depth\":" << queue.peak_size << "}}"
      << "\n";
  return static_cast<bool>(out);
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  std::string error;
  if (!parse_args(argc, argv, &options, &error)) {
    std::cerr << error << "\n";
    print_usage();
    return 2;
  }

  const std::string stem = default_stem(options.video);
  const std::filesystem::path out_dir(options.out_dir);
  std::error_code ec;
  std::filesystem::create_directories(out_dir, ec);
  if (options.jsonl_path.empty()) {
    options.jsonl_path = (out_dir / (stem + ".detections.jsonl")).string();
  }
  if (options.stats_path.empty()) {
    options.stats_path = (out_dir / (stem + ".stats.json")).string();
  }
  if (options.annotated_path.empty()) {
    options.annotated_path = (out_dir / (stem + ".annotated.mp4")).string();
  }

  auto detector = edgegaze::vision::create_body_detector();
  edgegaze::vision::BodyDetectorConfig detector_config;
  detector_config.model_path = options.model;
  detector_config.confidence_threshold = options.confidence;
  if (!detector->init(detector_config, &error)) {
    std::cerr << "detector init failed: " << error << "\n";
    return 3;
  }

  edgegaze::runtime::FramePipelineConfig pipeline_config;
  pipeline_config.source.camera = false;
  pipeline_config.source.source = options.video;
  pipeline_config.source.fps = options.fps;
  pipeline_config.queue_capacity = static_cast<std::size_t>(options.queue_capacity);
  pipeline_config.stats_path = (out_dir / (stem + ".frame_stats.jsonl")).string();
  edgegaze::runtime::FramePipeline pipeline(pipeline_config);
  if (!pipeline.start(&error)) {
    std::cerr << "frame source open failed: " << error << "\n";
    return 4;
  }

  std::ofstream jsonl(options.jsonl_path);
  if (!jsonl) {
    std::cerr << "cannot open jsonl output: " << options.jsonl_path << "\n";
    return 5;
  }

  cv::VideoWriter writer;
  std::size_t frames = 0;
  std::size_t detections_total = 0;
  std::size_t frames_with_detections = 0;
  std::vector<double> preprocess_ms;
  std::vector<double> infer_ms;
  std::vector<double> postprocess_ms;
  std::vector<double> total_ms;

  edgegaze::runtime::Frame frame;
  while (pipeline.pop(frame, std::chrono::milliseconds(5000))) {
    edgegaze::vision::DetectionResult result;
    if (!detector->detect(frame, &result, &error)) {
      std::cerr << "detect failed at frame " << frame.sequence << ": " << error << "\n";
      break;
    }

    const double pre = static_cast<double>(result.timing.preprocess_us) / 1000.0;
    const double inf = static_cast<double>(result.timing.infer_us) / 1000.0;
    const double post = static_cast<double>(result.timing.postprocess_us) / 1000.0;
    preprocess_ms.push_back(pre);
    infer_ms.push_back(inf);
    postprocess_ms.push_back(post);
    total_ms.push_back(pre + inf + post);

    std::ostringstream line;
    line << std::fixed << std::setprecision(6)
         << "{\"schema\":\"edgegaze.body_detection_frame.v1\""
         << ",\"frame_sequence\":" << frame.sequence
         << ",\"timestamp_us\":" << result.frame_timestamp_us
         << ",\"media_time_us\":" << frame.media_time_us
         << ",\"width\":" << frame.width
         << ",\"height\":" << frame.height
         << ",\"preprocess_us\":" << result.timing.preprocess_us
         << ",\"infer_us\":" << result.timing.infer_us
         << ",\"postprocess_us\":" << result.timing.postprocess_us
         << ",\"detections\":[";
    for (std::size_t i = 0; i < result.detections.size(); ++i) {
      const auto& det = result.detections[i];
      if (i != 0) line << ",";
      line << "{\"x\":" << det.x << ",\"y\":" << det.y
           << ",\"w\":" << det.w << ",\"h\":" << det.h
           << ",\"confidence\":" << det.confidence << "}";
    }
    line << "]}";
    jsonl << line.str() << "\n";

    ++frames;
    detections_total += result.detections.size();
    if (!result.detections.empty()) ++frames_with_detections;

    cv::Mat bgr(frame.height, frame.width, CV_8UC3,
                const_cast<uint8_t*>(frame.data->data()));
    cv::Mat annotated = bgr.clone();
    for (const auto& det : result.detections) {
      const int x = static_cast<int>(det.x * frame.width);
      const int y = static_cast<int>(det.y * frame.height);
      const int w = static_cast<int>(det.w * frame.width);
      const int h = static_cast<int>(det.h * frame.height);
      cv::rectangle(annotated, cv::Rect(x, y, w, h), cv::Scalar(0, 255, 0), 2);
      std::ostringstream label;
      label << "person " << std::fixed << std::setprecision(2) << det.confidence;
      cv::putText(annotated, label.str(), cv::Point(x, std::max(0, y - 6)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }
    if (!writer.isOpened()) {
      writer.open(options.annotated_path,
                  cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                  options.fps, cv::Size(frame.width, frame.height));
      if (!writer.isOpened()) {
        std::cerr << "warning: cannot open annotated video: "
                  << options.annotated_path << "\n";
      }
    }
    if (writer.isOpened()) writer.write(annotated);

    if (options.max_frames > 0 &&
        frames >= static_cast<std::size_t>(options.max_frames)) {
      break;
    }
  }

  pipeline.stop();
  writer.release();
  jsonl.flush();

  const auto queue = pipeline.queue_stats();
  if (!write_stats(options.stats_path, options, detector->model_name(),
                   detector->input_width(), detector->input_height(), frames,
                   detections_total, frames_with_detections, preprocess_ms,
                   infer_ms, postprocess_ms, total_ms, queue)) {
    std::cerr << "cannot write stats: " << options.stats_path << "\n";
    return 6;
  }

  std::cout << "frames=" << frames << " detections=" << detections_total
            << " frames_with_detections=" << frames_with_detections << "\n"
            << std::fixed << std::setprecision(3)
            << "preprocess_ms p50=" << percentile(preprocess_ms, 50.0)
            << " p95=" << percentile(preprocess_ms, 95.0) << "\n"
            << "infer_ms p50=" << percentile(infer_ms, 50.0)
            << " p95=" << percentile(infer_ms, 95.0) << "\n"
            << "postprocess_ms p50=" << percentile(postprocess_ms, 50.0)
            << " p95=" << percentile(postprocess_ms, 95.0) << "\n"
            << "total_ms p50=" << percentile(total_ms, 50.0)
            << " p95=" << percentile(total_ms, 95.0) << "\n";
  return 0;
}
