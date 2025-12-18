#include "detect.hpp"
#include "pnp.hpp"
#include "kalman_detector.hpp"
#include <iostream>
#include <vector>

namespace detection
{

Detect::Detect(const std::string& model_path,
               EnemyColor detect_color,
               int binary_threshold,
               const TraditionalDetector::LightParams& l_params,
               const TraditionalDetector::ArmorParams& a_params)
    : model_path_copy_(model_path),
      detect_color_(detect_color)
{
    // 初始化 YOLO 检测器
    yolo_detector_ = std::make_unique<YoloDetector>(model_path_copy_, detect_color_, false);

    // 初始化传统检测器
    traditional_detector_ = std::make_unique<TraditionalDetector>(binary_threshold, l_params, a_params, detect_color_);

    // 初始化PnP求解器（需要根据实际相机内参修改）
    std::array<double, 9> camera_matrix = {
        1000, 0, 320,
        0, 1000, 240,
        0, 0, 1
    };
    std::vector<double> dist_coeffs = {0, 0, 0, 0, 0};
    pnp_solver_ = std::make_unique<PnPSolver>(camera_matrix, dist_coeffs);

    // 初始化 Kalman 滤波检测器
    kalman_detector_ = std::make_unique<KalmanDetector>(1e-2, 1e-1, 10);
}

void Detect::set_detect_color(EnemyColor color)
{
    detect_color_ = color;
    yolo_detector_->set_detect_color(color);
    traditional_detector_->set_detect_color(color);
}

void Detect::detect(const cv::Mat& image)
{
    if (image.empty()) {
        yolo_detector_->clear();
        traditional_detector_->clear();
        pnp_solver_->clear();
        kalman_detector_->clear();
        delta_msg_ = DeltaMsg{};
        target_info_msg_ = TargetInfoMsg{};
        return;
    }

    // 先进行 YOLO 检测（结果存储在 yolo_detector_ 内部）
    const auto& yolo_results = yolo_detector_->detect(image);

    // 传统检测（依赖 YOLO 结果，结果存储在 traditional_detector_ 内部）
    const auto& traditional_results = traditional_detector_->detect(image, yolo_results);

    // PnP 求解（结果存储在 pnp_solver_ 内部）
    pnp_solver_->detect(traditional_results);

    // Kalman 滤波（结果存储在 kalman_detector_ 内部）
    // 注意：滤波结果仅用于跟踪预测，不修改 delta_msg_ 和 target_info_msg_
    kalman_detector_->detect(traditional_results);

    // 填充消息数据（使用传统融合后的结果，不使用Kalman滤波结果）
    if (!traditional_results.empty()) {
        const auto& target = traditional_results[0];
        const cv::Point optical_center(image.cols / 2, image.rows / 2);
        const float delta_x = (target.center.x - optical_center.x) + GUN_CAM_DISTANCE_X;
        const float delta_y = (optical_center.y - target.center.y) + GUN_CAM_DISTANCE_Y;

        delta_msg_.x = delta_x;
        delta_msg_.y = delta_y;
        delta_msg_.z = 0.0f;

        target_info_msg_.target_x = static_cast<float>(target.center.x);
        target_info_msg_.target_y = static_cast<float>(target.center.y);
        target_info_msg_.optical_center_x = optical_center.x;
        target_info_msg_.optical_center_y = optical_center.y;
        target_info_msg_.delta_x = delta_x;
        target_info_msg_.delta_y = delta_y;
        target_info_msg_.flag = 1.0f;
    } else {
        delta_msg_ = DeltaMsg{};
        target_info_msg_ = TargetInfoMsg{};
    }
}

}  // namespace detection
