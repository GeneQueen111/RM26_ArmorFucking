#include "kalman_detector.hpp"
#include <algorithm>
#include <iostream>

namespace detection
{

KalmanDetector::KalmanDetector()
{
    initKalmanFilter();
    resetRotationState();
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

void KalmanDetector::resetRotationState()
{
    measurement_history_.clear();
    last_output_center_ = cv::Point2f();
    has_last_output_center_ = false;
    motion_ref_dir_ = cv::Point2f();
    motion_dir_score_ = 0;
    is_rotating_ = false;
}

bool KalmanDetector::updateRotationState(bool has_measurement, bool is_jump, const cv::Point2f& output_center)
{
    if (!is_tracking_) {
        resetRotationState();
        return false;
    }

    measurement_history_.push_back(has_measurement);
    if (measurement_history_.size() > kRotationWindowFrames) {
        measurement_history_.pop_front();
    }

    if (has_last_output_center_) {
        const cv::Point2f motion_vec = output_center - last_output_center_;
        const float motion_norm = static_cast<float>(cv::norm(motion_vec));

        if (motion_norm >= kRotationMinMotionPx) {
            const cv::Point2f motion_dir = motion_vec * (1.0f / motion_norm);

            const float ref_norm = static_cast<float>(cv::norm(motion_ref_dir_));
            if (ref_norm < 1e-3f) {
                motion_ref_dir_ = motion_dir;
                const int inc = is_jump ? 2 : 1;
                motion_dir_score_ = std::min(motion_dir_score_ + inc, kRotationDirScoreMax);
            } else {
                const float dot = motion_dir.dot(motion_ref_dir_);
                if (dot >= kRotationDirDotThreshold) {
                    const int inc = is_jump ? 3 : 2;
                    motion_dir_score_ = std::min(motion_dir_score_ + inc, kRotationDirScoreMax);

                    // 轻微更新参考方向，增强对噪声的鲁棒性
                    constexpr float alpha = 0.2f;
                    motion_ref_dir_ = motion_ref_dir_ * (1.0f - alpha) + motion_dir * alpha;
                    const float new_norm = static_cast<float>(cv::norm(motion_ref_dir_));
                    if (new_norm > 1e-3f) {
                        motion_ref_dir_ *= 1.0f / new_norm;
                    }
                } else {
                    const int dec = is_jump ? 1 : 2;
                    motion_dir_score_ = std::max(motion_dir_score_ - dec, 0);
                    if (dot <= -kRotationDirDotThreshold) {
                        motion_ref_dir_ = motion_dir;
                    }
                }
            }
        }
    }

    last_output_center_ = output_center;
    has_last_output_center_ = true;

    int toggle_count = 0;
    int missing_frames = 0;
    for (size_t i = 1; i < measurement_history_.size(); ++i) {
        if (measurement_history_[i] != measurement_history_[i - 1]) {
            toggle_count++;
        }
    }
    for (const bool has_meas : measurement_history_) {
        if (!has_meas) {
            missing_frames++;
        }
    }

    const bool direction_ok = motion_dir_score_ >= kRotationDirScoreThreshold;
    is_rotating_ = direction_ok && (toggle_count >= kRotationToggleThreshold) &&
                   (missing_frames >= kRotationMinMissingFrames);
    return is_rotating_;
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
                cv::Point2f velocity(
                    kf_.statePost.at<float>(2),
                    kf_.statePost.at<float>(3));
                if (lead_time_ > 0.0f) {
                    predicted += velocity * lead_time_;
                }
                primary_result_.predicted_center = predicted;
                primary_result_.filtered_center = predicted;
                primary_result_.velocity = velocity;
                primary_result_.is_tracking = true;
                primary_result_.is_rotating = updateRotationState(false, false, primary_result_.filtered_center);
                primary_result_.lost_count = lost_count_;
                results_.push_back(primary_result_);
            }
        } else {
            primary_result_ = KalmanResult{};
            resetRotationState();
        }
        return results_;
    }

    // 有检测结果，使用第一个目标进行滤波
    const auto& target = traditional_results[0];
    cv::Point2f measurement = target.center;
    // 以左右灯条中心距近似装甲板长度，用于判断是否跳到下一块装甲
    float armor_length = cv::norm(target.left_light.center - target.right_light.center);
    float jump_threshold = armor_length * 0.5f;

    cv::Point2f filtered;
    bool jumped = false;
    if (!is_initialized_) {
        // 首次初始化
        kf_.statePost.at<float>(0) = measurement.x;
        kf_.statePost.at<float>(1) = measurement.y;
        kf_.statePost.at<float>(2) = 0;  // 初始速度为0
        kf_.statePost.at<float>(3) = 0;
        kf_.statePre = kf_.statePost.clone();
        is_initialized_ = true;
        is_tracking_ = true;
        filtered = measurement;
        last_position_ = measurement;
    } else {
        // 判断是否发生大跳变（旋转到下一块装甲），若是则重置滤波器
        float displacement = cv::norm(measurement - last_position_);
        if (displacement > jump_threshold) {
            jumped = true;
            initKalmanFilter();
            kf_.statePost.at<float>(0) = measurement.x;
            kf_.statePost.at<float>(1) = measurement.y;
            kf_.statePost.at<float>(2) = 0;
            kf_.statePost.at<float>(3) = 0;
            kf_.statePre = kf_.statePost.clone();
            is_initialized_ = true;
            is_tracking_ = true;
            filtered = measurement;
        } else {
            // 更新滤波器
            filtered = update(measurement);
            is_tracking_ = true;
        }
    }

    lost_count_ = 0;

    // 填充主目标结果
    cv::Point2f predicted_center(
        kf_.statePre.at<float>(0),
        kf_.statePre.at<float>(1));
    cv::Point2f velocity(
        kf_.statePost.at<float>(2),
        kf_.statePost.at<float>(3));
    if (lead_time_ > 0.0f) {
        predicted_center += velocity * lead_time_;
    }
    primary_result_.filtered_center = filtered;
    primary_result_.predicted_center = predicted_center;
    primary_result_.velocity = velocity;
    primary_result_.is_tracking = true;
    primary_result_.is_rotating = updateRotationState(!jumped, jumped, primary_result_.filtered_center);
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
            result.is_rotating = false;
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
    resetRotationState();
}

void KalmanDetector::reset()
{
    initKalmanFilter();
    clear();
}

}  // namespace detection
