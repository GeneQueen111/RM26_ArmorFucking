#ifndef KALMAN_DETECTOR_HPP_
#define KALMAN_DETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>
#include "data.hpp"
#include "pnp.hpp"

namespace detection
{

/**
 * @brief Kalman滤波结果数据结构
 */
struct KalmanResult
{
    cv::Point2f filtered_center;   // 滤波后的中心点
    cv::Point2f predicted_center;  // 预测的中心点
    cv::Point2f velocity;          // 速度估计 (像素/帧)
    bool is_tracking = false;      // 是否正在跟踪
    int lost_count = 0;            // 丢失计数
};

/**
 * @brief Kalman滤波检测器，用于目标跟踪和位置预测
 */
class KalmanDetector
{
public:
    /**
     * @brief 构造函数
     * @param process_noise 过程噪声系数（默认1e-2）
     * @param measurement_noise 测量噪声系数（默认1e-1）
     * @param max_lost_frames 最大丢失帧数（默认10）
     */
    KalmanDetector(double process_noise = 1e-2,
                   double measurement_noise = 1e-1,
                   int max_lost_frames = 10);

    /**
     * @brief 检测入口，对传统检测结果进行Kalman滤波
     * @param traditional_results 传统检测结果
     * @return Kalman滤波结果引用
     */
    const std::vector<KalmanResult>& detect(const std::vector<TraditionalArmorData>& traditional_results);

    /**
     * @brief 检测入口重载，对PnP结果进行Kalman滤波
     * @param pnp_results PnP检测结果
     * @return Kalman滤波结果引用
     */
    const std::vector<KalmanResult>& detect(const std::vector<PnPResult>& pnp_results);

    /**
     * @brief 获取滤波结果
     */
    const std::vector<KalmanResult>& getResults() const { return results_; }

    /**
     * @brief 获取主目标的滤波结果（第一个目标）
     */
    const KalmanResult& getPrimaryResult() const { return primary_result_; }

    /**
     * @brief 清空检测结果并重置滤波器
     */
    void clear();

    /**
     * @brief 重置滤波器状态
     */
    void reset();

    /**
     * @brief 检查是否正在跟踪目标
     */
    bool isTracking() const { return is_tracking_; }

private:
    /**
     * @brief 初始化Kalman滤波器
     */
    void initKalmanFilter();

    /**
     * @brief 使用测量值更新滤波器
     * @param measurement 测量值（中心点坐标）
     * @return 滤波后的坐标
     */
    cv::Point2f update(const cv::Point2f& measurement);

    /**
     * @brief 仅预测（无测量值时）
     * @return 预测的坐标
     */
    cv::Point2f predict();

    // Kalman滤波器
    cv::KalmanFilter kf_;

    // 滤波器参数
    double process_noise_;
    double measurement_noise_;
    int max_lost_frames_;

    // 跟踪状态
    bool is_tracking_ = false;
    bool is_initialized_ = false;
    int lost_count_ = 0;

    // 结果缓存
    std::vector<KalmanResult> results_;
    KalmanResult primary_result_;

    // 上一帧的状态
    cv::Point2f last_position_;
};

}  // namespace detection

#endif  // KALMAN_DETECTOR_HPP_
