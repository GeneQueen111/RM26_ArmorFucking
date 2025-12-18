#ifndef DETECT_HPP_
#define DETECT_HPP_

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <memory>
#include <tuple>

#include "yolo_detection.hpp"
#include "traditional_detector.hpp"
#include "pnp.hpp"
#include "kalman_detector.hpp"

namespace detection
{

/**
 * @brief Delta 消息结构体（对应 geometry_msgs::msg::Point）
 */
struct DeltaMsg
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

/**
 * @brief 目标信息消息结构体（对应 std_msgs::msg::Float32MultiArray）
 */
struct TargetInfoMsg
{
    float target_x = 0.0f;
    float target_y = 0.0f;
    float optical_center_x = 0.0f;
    float optical_center_y = 0.0f;
    float delta_x = 0.0f;
    float delta_y = 0.0f;
    float flag = 0.0f;
};

/**
 * @brief 统一装甲板检测入口，封装 YOLO + 传统视觉流程
 */
class Detect
{
public:
    /**
     * @brief 构造检测器
     * @param model_path OpenVINO 模型路径
     * @param detect_color 检测颜色
     * @param binary_threshold 传统检测二值化阈值（默认 100）
     * @param l_params 传统检测灯条参数
     * @param a_params 传统检测装甲板参数
     */
    Detect(const std::string& model_path,
           EnemyColor detect_color,
           int binary_threshold = 100,
           const TraditionalDetector::LightParams& l_params = {},
           const TraditionalDetector::ArmorParams& a_params = {});

    /**
     * @brief 运行检测
     * @param image 输入图像（BGR）
     */
    void detect(const cv::Mat& image);

    /**
     * @brief 设置识别颜色（同时同步到子检测器）
     */
    void set_detect_color(EnemyColor color);

    /**
     * @brief 获取当前识别颜色
     */
    EnemyColor get_detect_color() const { return detect_color_; }

    /**
     * @brief 获取各阶段缓存的检测结果（直接引用各 detector 内部结果）
     */
    const std::vector<YoloArmorData>& yolo_results() const { return yolo_detector_->getdata(); }
    const std::vector<TraditionalArmorData>& traditional_results() const { return traditional_detector_->getArmors(); }
    const std::vector<PnPResult>& pnp_results() const { return pnp_solver_->getResults(); }
    const std::vector<KalmanResult>& kalman_results() const { return kalman_detector_->getResults(); }

    /**
     * @brief 获取 delta 消息数据
     */
    const DeltaMsg& get_delta_msg() const { return delta_msg_; }

    /**
     * @brief 获取目标信息消息数据
     */
    const TargetInfoMsg& get_target_info_msg() const { return target_info_msg_; }

private:
    std::string model_path_copy_;
    EnemyColor detect_color_;  // 检测颜色

    // 各个检测器
    std::unique_ptr<YoloDetector> yolo_detector_;
    std::unique_ptr<TraditionalDetector> traditional_detector_;
    std::unique_ptr<PnPSolver> pnp_solver_;
    std::unique_ptr<KalmanDetector> kalman_detector_;

    // 消息数据缓存
    DeltaMsg delta_msg_;
    TargetInfoMsg target_info_msg_;
};

}  // namespace detection

#endif  // DETECT_HPP_
