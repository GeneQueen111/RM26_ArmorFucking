#include "detect.hpp"
#include "pnp.hpp"
#include <iostream>

namespace detection
{

Detect::Detect(const std::string& model_path,
               int binary_threshold,
               const TraditionalDetector::LightParams& l_params,
               const TraditionalDetector::ArmorParams& a_params)
    : model_path_copy_(model_path),
      yolo_detector_(model_path_copy_, false),
      traditional_detector_(binary_threshold, l_params, a_params)
{
    // 初始化PnP求解器（需要根据实际相机内参修改）
    std::array<double, 9> camera_matrix = {
        1000, 0, 320,
        0, 1000, 240,
        0, 0, 1
    };
    std::vector<double> dist_coeffs = {0, 0, 0, 0, 0};
    pnp_solver_ = std::make_unique<PnPSolver>(camera_matrix, dist_coeffs);
}

std::vector<ArmorData> Detect::detect(const cv::Mat& image)
{
    if (image.empty()) {
        yolo_results_.clear();
        traditional_results_.clear();
        pnp_results_.clear();
        fused_results_.clear();
        return {};
    }

    // 先进行 YOLO 检测
    yolo_results_ = yolo_detector_.detect(image);

    // 获取传统检测器的Armor对象（用于PnP）
    traditional_results_ = traditional_detector_.detect(image);

    // 对每个装甲板进行PnP测距
    pnp_results_ = pnp_solver_->detect(traditional_results_);

    // 传统检测器根据 YOLO 结果进行融合/回退
    fused_results_ = traditional_detector_.detect(image, yolo_results_);
    return fused_results_;
}

}  // namespace detection
