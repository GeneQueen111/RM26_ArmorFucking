#include "kalman_detector.hpp"
#include <iostream>

namespace detection
{

KalmanDetector::KalmanDetector(double process_noise,
                               double measurement_noise,
                               int max_lost_frames)
    : process_noise_(process_noise),
      measurement_noise_(measurement_noise),
      max_lost_frames_(max_lost_frames)
{
    initKalmanFilter();
}

void KalmanDetector::initKalmanFilter()
{
    // 状态向量: [x, y, vx, vy] (位置和速度)
    // 测量向量: [x, y] (只测量位置)
    kf_ = cv::KalmanFilter(4, 2, 0);

    // 状态转移矩阵 A
    // [1, 0, dt, 0 ]
    // [0, 1, 0,  dt]
    // [0, 0, 1,  0 ]
    // [0, 0, 0,  1 ]
    // dt = 1 (每帧)
    kf_.transitionMatrix = (cv::Mat_<float>(4, 4) <<
        1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);

    // 测量矩阵 H
    // [1, 0, 0, 0]
    // [0, 1, 0, 0]
    kf_.measurementMatrix = (cv::Mat_<float>(2, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0);

    // 过程噪声协方差 Q
    cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(process_noise_));

    // 测量噪声协方差 R
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(measurement_noise_));

    // 后验误差协方差 P
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1));

    is_initialized_ = false;
    is_tracking_ = false;
    lost_count_ = 0;
}

const std::vector<KalmanResult>& KalmanDetector::detect(const std::vector<TraditionalArmorData>& traditional_results)
{
    results_.clear();

    if (traditional_results.empty()) {
        // 无检测结果，执行纯预测
        if (is_tracking_) {
            lost_count_++;
            if (lost_count_ > max_lost_frames_) {
                // 丢失过久，重置跟踪
                reset();
                primary_result_ = KalmanResult{};
            } else {
                // 仅预测
                cv::Point2f predicted = predict();
                primary_result_.predicted_center = predicted;
                primary_result_.filtered_center = predicted;
                primary_result_.is_tracking = true;
                primary_result_.lost_count = lost_count_;
            }
        } else {
            primary_result_ = KalmanResult{};
        }
        return results_;
    }

    // 有检测结果，使用第一个目标进行滤波
    const auto& target = traditional_results[0];
    cv::Point2f measurement = target.center;

    cv::Point2f filtered;
    if (!is_initialized_) {
        // 首次初始化
        kf_.statePost.at<float>(0) = measurement.x;
        kf_.statePost.at<float>(1) = measurement.y;
        kf_.statePost.at<float>(2) = 0;  // 初始速度为0
        kf_.statePost.at<float>(3) = 0;
        is_initialized_ = true;
        is_tracking_ = true;
        filtered = measurement;
        last_position_ = measurement;
    } else {
        // 更新滤波器
        filtered = update(measurement);
        is_tracking_ = true;
    }

    lost_count_ = 0;

    // 填充主目标结果
    primary_result_.filtered_center = filtered;
    primary_result_.predicted_center = cv::Point2f(
        kf_.statePre.at<float>(0),
        kf_.statePre.at<float>(1));
    primary_result_.velocity = cv::Point2f(
        kf_.statePost.at<float>(2),
        kf_.statePost.at<float>(3));
    primary_result_.is_tracking = true;
    primary_result_.lost_count = 0;

    // 为每个检测到的装甲板生成结果（目前只跟踪主目标）
    for (size_t i = 0; i < traditional_results.size(); ++i) {
        KalmanResult result;
        if (i == 0) {
            result = primary_result_;
        } else {
            // 其他目标暂时只返回原始值，不进行滤波
            result.filtered_center = traditional_results[i].center;
            result.predicted_center = traditional_results[i].center;
            result.velocity = cv::Point2f(0, 0);
            result.is_tracking = false;
            result.lost_count = 0;
        }
        results_.push_back(result);
    }

    last_position_ = filtered;
    return results_;
}

const std::vector<KalmanResult>& KalmanDetector::detect(const std::vector<PnPResult>& pnp_results)
{
    // 从PnP结果中提取TraditionalArmorData，复用上面的detect方法
    std::vector<TraditionalArmorData> armors;
    armors.reserve(pnp_results.size());
    for (const auto& pnp : pnp_results) {
        armors.push_back(pnp.armor);
    }
    return detect(armors);
}

cv::Point2f KalmanDetector::update(const cv::Point2f& measurement)
{
    // 预测
    cv::Mat prediction = kf_.predict();

    // 更新
    cv::Mat meas = (cv::Mat_<float>(2, 1) << measurement.x, measurement.y);
    cv::Mat estimated = kf_.correct(meas);

    return cv::Point2f(estimated.at<float>(0), estimated.at<float>(1));
}

cv::Point2f KalmanDetector::predict()
{
    cv::Mat prediction = kf_.predict();
    // 由于没有测量值，需要手动复制预测值到后验状态
    prediction.copyTo(kf_.statePost);
    return cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
}

void KalmanDetector::clear()
{
    results_.clear();
    primary_result_ = KalmanResult{};
}

void KalmanDetector::reset()
{
    initKalmanFilter();
    clear();
}

}  // namespace detection
