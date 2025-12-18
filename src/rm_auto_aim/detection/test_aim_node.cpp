#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <numeric>
#include <algorithm>

#include "include/detect.hpp"
#include "include/data.hpp"
#include "include/pnp.hpp"
#include "include/kalman_detector.hpp"

///////////////////////////////////////////////////////////////////////////
// 文件工具函数
///////////////////////////////////////////////////////////////////////////

/**
 * @brief 检查文件或目录是否存在
 * @param path 文件或目录路径
 * @return bool 存在返回true，否则返回false
 */
bool file_exists(const std::string& path) {
    struct stat buffer;
    return (stat(path.c_str(), &buffer) == 0);
}

/**
 * @brief 从完整路径中提取文件名
 * @param path 完整文件路径
 * @return string 文件名
 */
std::string get_filename(const std::string& path) {
    size_t last_slash = path.find_last_of("/");
    if (last_slash != std::string::npos) {
        return path.substr(last_slash + 1);
    }
    return path;
}

/**
 * @brief 获取文件扩展名
 * @param path 文件路径
 * @return string 扩展名（如.mp4）
 */
std::string get_extension(const std::string& path) {
    size_t last_dot = path.find_last_of(".");
    if (last_dot != std::string::npos) {
        return path.substr(last_dot);
    }
    return "";
}

///////////////////////////////////////////////////////////////////////////
// 测试结果结构体
///////////////////////////////////////////////////////////////////////////

/**
 * @brief 存储单个视频的测试结果
 */
struct TestResult {
    std::string video_name;
    int total_frames = 0;
    int detected_frames = 0;
    int total_armors = 0;
    int detected_armors = 0;
    double avg_fps = 0.0;
    double processing_time_ms = 0.0;
    std::vector<double> frame_fps_list;
    std::vector<int> armor_count_per_frame;
};

///////////////////////////////////////////////////////////////////////////
// 统计工具函数
///////////////////////////////////////////////////////////////////////////

/**
 * @brief 计算向量的平均值
 * @param values 数值向量
 * @return double 平均值
 */
template <typename T>
double calculate_average(const std::vector<T>& values) {
    if (values.empty()) return 0.0;
    return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
}

/**
 * @brief 计算向量的最小值
 * @param values 数值向量
 * @return T 最小值
 */
template <typename T>
T calculate_minimum(const std::vector<T>& values) {
    if (values.empty()) return T();
    return *std::min_element(values.begin(), values.end());
}

/**
 * @brief 计算向量的最大值
 * @param values 数值向量
 * @return T 最大值
 */
template <typename T>
T calculate_maximum(const std::vector<T>& values) {
    if (values.empty()) return T();
    return *std::max_element(values.begin(), values.end());
}

/**
 * @brief 计算检测率
 * @param total 总数量
 * @param detected 检测到的数量
 * @return double 检测率百分比
 */
double calculate_detection_rate(int total, int detected) {
    return (total > 0) ? (100.0 * detected / total) : 0.0;
}

/**
 * @brief 更新帧统计信息
 * @param result 测试结果
 * @param armors 检测到的装甲板列表
 * @param processing_time_ms 处理时间（毫秒）
 */
void update_frame_statistics(
    TestResult& result,
    const std::vector<detection::TraditionalArmorData>& armors,
    double processing_time_ms
) {
    if (!armors.empty()) {
        result.detected_frames++;
        result.total_armors += armors.size();
        result.detected_armors += armors.size();
        result.armor_count_per_frame.push_back(armors.size());
    } else {
        result.armor_count_per_frame.push_back(0);
    }

    result.processing_time_ms += processing_time_ms;

    // 计算帧率
    double fps = 1000.0 / (processing_time_ms > 0 ? processing_time_ms : 1);
    result.frame_fps_list.push_back(fps);
}

/**
 * @brief 输出最近的帧率统计（每100帧输出一次）
 * @param frame_idx 当前帧索引
 * @param frame_fps_list 帧率列表
 */
void print_frame_statistics(int frame_idx, const std::vector<double>& frame_fps_list) {
    if (frame_idx % 100 == 0) {
        size_t start_idx = (frame_fps_list.size() > 100) ? (frame_fps_list.size() - 100) : 0;
        std::vector<double> last_100(frame_fps_list.begin() + start_idx, frame_fps_list.end());

        double avg = calculate_average(last_100);
        double min_val = calculate_minimum(last_100);
        double max_val = calculate_maximum(last_100);

        std::cout << "平均: " << std::fixed << std::setprecision(1) << avg
                  << " | 最低: " << min_val
                  << " | 最高: " << max_val << " FPS" << std::endl;
    }
}

/**
 * @brief 显示检测结果（仅在测试模式下启用）
 * @param frame 原始帧
 * @param armors 检测到的装甲板列表 (Traditional)
 * @param pnp_results PnP解算结果列表
 * @param kalman_results Kalman滤波结果列表
 */
void display_detection_results(const cv::Mat& frame,
                               const std::vector<detection::TraditionalArmorData>& armors,
                               const std::vector<PnPResult>& pnp_results,
                               const std::vector<detection::KalmanResult>& kalman_results) {
    cv::Mat display_frame = frame.clone();


    // 绘制传统检测的灯条和装甲板
    for (size_t i = 0; i < armors.size(); ++i) {
        const auto& armor = armors[i];
        // 绘制左右灯条
        const auto& left_light = armor.left_light;
        const auto& right_light = armor.right_light;

        // 绘制灯条顶部和底部标记点
        cv::circle(display_frame, left_light.top, 3, cv::Scalar(255, 255, 255), 1);
        cv::circle(display_frame, left_light.bottom, 3, cv::Scalar(255, 255, 255), 1);
        cv::circle(display_frame, right_light.top, 3, cv::Scalar(255, 255, 255), 1);
        cv::circle(display_frame, right_light.bottom, 3, cv::Scalar(255, 255, 255), 1);

        // 绘制灯条线（根据颜色选择不同颜色）
        auto left_color = left_light.color == EnemyColor::RED ? cv::Scalar(255, 0, 255) : cv::Scalar(255, 255, 0);
        auto right_color = right_light.color == EnemyColor::RED ? cv::Scalar(255, 0, 255) : cv::Scalar(255, 255, 0);
        cv::line(display_frame, left_light.top, left_light.bottom, left_color, 5);
        cv::line(display_frame, right_light.top, right_light.bottom, right_color, 5);

        // 绘制装甲板对角线
        cv::line(display_frame, left_light.top, right_light.bottom, cv::Scalar(0, 255, 0), 2);
        cv::line(display_frame, left_light.bottom, right_light.top, cv::Scalar(0, 255, 0), 2);

        // 绘制装甲板中心点
        cv::circle(display_frame, armor.center, 5, cv::Scalar(255, 0, 0), -1);

        // 显示分类结果
        cv::putText(display_frame, armor.classfication_result, left_light.top,
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);

        // // 显示PnP距离（在装甲板上方）
        // if (i < pnp_results.size() && pnp_results[i].success) {
        //     // 计算装甲板上方位置（取左右灯条顶部的中点再往上偏移）
        //     cv::Point2f top_center = (left_light.top + right_light.top) / 2;
        //     cv::Point text_pos(static_cast<int>(top_center.x - 30), static_cast<int>(top_center.y - 10));

        //     // 格式化距离字符串（保留2位小数，单位米）
        //     std::ostringstream oss;
        //     oss << std::fixed << std::setprecision(2) << pnp_results[i].distance << "m";

        //     cv::putText(display_frame, oss.str(), text_pos,
        //                 cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        // }
    }

    // 绘制Kalman滤波结果
    for (const auto& kalman : kalman_results) {
        if (kalman.is_tracking) {
            // 绘制滤波后的中心点（黄色实心圆）
            cv::circle(display_frame, kalman.filtered_center, 8, cv::Scalar(0, 255, 255), -1);

            // 绘制预测的中心点（橙色空心圆）
            cv::circle(display_frame, kalman.predicted_center, 10, cv::Scalar(0, 165, 255), 2);

            // 绘制从滤波点到预测点的速度向量线（白色）
            cv::line(display_frame, kalman.filtered_center, kalman.predicted_center,
                     cv::Scalar(255, 255, 255), 2);

            // 显示速度信息（在预测点旁边）
            std::ostringstream oss;
            oss << "v:(" << std::fixed << std::setprecision(1)
                << kalman.velocity.x << "," << kalman.velocity.y << ")";
            cv::putText(display_frame, oss.str(),
                        cv::Point(static_cast<int>(kalman.predicted_center.x + 15),
                                  static_cast<int>(kalman.predicted_center.y)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        } else if (kalman.lost_count > 0) {
            // 目标丢失时，用红色空心圆显示预测位置
            cv::circle(display_frame, kalman.predicted_center, 10, cv::Scalar(0, 0, 255), 2);

            // 显示丢失帧数
            std::ostringstream oss;
            oss << "lost:" << kalman.lost_count;
            cv::putText(display_frame, oss.str(),
                        cv::Point(static_cast<int>(kalman.predicted_center.x + 15),
                                  static_cast<int>(kalman.predicted_center.y)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        }
    }

    cv::imshow("Armor Detection Test", display_frame);

    // 处理按键输入：'q'退出，空格暂停/继续
    char key = cv::waitKey(1);
    if (key == 'q' || key == 'Q') {
        std::cout << "用户中断测试" << std::endl;
        cv::destroyAllWindows();
        exit(0);
    }
}

///////////////////////////////////////////////////////////////////////////
// 报告生成函数
///////////////////////////////////////////////////////////////////////////

/**
 * @brief 处理视频中的每一帧
 * @param cap 视频捕获对象
 * @param detector 装甲板检测器
 * @param result 测试结果
 * @return bool 处理是否成功（false表示视频结束）
 */
bool process_video_frames(cv::VideoCapture& cap, detection::Detect& detector, TestResult& result) {
    cv::Mat frame;
    int frame_idx = 0;

    while (true) {
        // 读取帧
        cap >> frame;
        if (frame.empty()) {
            break;  // 视频结束
        }

        // 记录帧开始时间
        auto frame_start_time = std::chrono::high_resolution_clock::now();

        // 执行检测（完整流程：YOLO + 传统检测 + PnP）
        detector.detect(frame);

        // 记录帧结束时间并计算处理时间
        auto frame_end_time = std::chrono::high_resolution_clock::now();
        auto frame_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            frame_end_time - frame_start_time).count();


        auto yolo_armors = detector.yolo_results();
        auto armors = detector.traditional_results();
        auto pnp_results = detector.pnp_results();
        auto kalman_results = detector.kalman_results();

        // 更新统计信息
        update_frame_statistics(result, armors, frame_duration);

        // 显示进度统计
        print_frame_statistics(frame_idx, result.frame_fps_list);

        // 显示检测结果
        display_detection_results(frame, armors, pnp_results, kalman_results);

        frame_idx++;
    }

    // 计算平均帧率
    if (!result.frame_fps_list.empty()) {
        result.avg_fps = calculate_average(result.frame_fps_list);
    }

    return true;
}

/**
 * @brief 测试单个视频文件（完整流程：YOLO + 传统检测 + PnP）
 * @param video_path 视频文件路径
 * @param detector 装甲板检测器
 * @return TestResult 测试结果
 */
TestResult testVideo(const std::string& video_path, detection::Detect& detector) {
    TestResult result;
    result.video_name = get_filename(video_path);

    std::cout << "========================================" << std::endl;
    std::cout << "正在测试视频: " << video_path << std::endl;

    // 打开视频文件
    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        std::cerr << "错误: 无法打开视频文件 " << video_path << std::endl;
        return result;
    }

    // 获取视频基本信息
    int total_frames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = cap.get(cv::CAP_PROP_FPS);

    std::cout << "视频信息: " << width << "x" << height
              << " @ " << fps << " FPS, 共 " << total_frames << " 帧" << std::endl;
    std::cout << "========================================" << std::endl;

    result.total_frames = total_frames;

    // 处理视频帧
    process_video_frames(cap, detector, result);

    // 释放视频资源
    cap.release();

    #ifdef TEST_MODE
    cv::destroyAllWindows();
    #endif

    return result;
}

///////////////////////////////////////////////////////////////////////////
// 初始化和扫描相关函数
///////////////////////////////////////////////////////////////////////////

/**
 * @brief 获取当前工作目录
 * @return string 当前工作目录路径
 */
std::string get_current_working_directory() {
    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd))) {
        return std::string(cwd);
    } else {
        std::cerr << "错误: 无法获取当前工作目录" << std::endl;
        return "";
    }
}

/**
 * @brief 设置模型和视频路径
 * @param current_path 当前工作目录路径
 * @param model_path [out] 模型文件路径
 * @param video_dir [out] 视频目录路径
 */
void setup_paths(const std::string& current_path, std::string& model_path, std::string& video_dir) {
    model_path = current_path + "/src/rm_auto_aim/detection/model/IR_MODEL/new.xml";
    // model_path = current_path + "/src/rm_auto_aim/detection/model/IR_MODEL/new_fp16.xml";
    // model_path = current_path + "/src/rm_auto_aim/detection/model/IR_MODEL/new_int8.xml";
    video_dir = current_path + "/src/rm_auto_aim/detection/video/video_640x640/";
}

/**
 * @brief 扫描视频目录，获取所有视频文件
 * @param video_dir 视频目录路径
 * @return std::vector<std::string> 视频文件路径列表
 */
std::vector<std::string> scan_video_files(const std::string& video_dir) {
    std::vector<std::string> video_files;

    DIR* dir = opendir(video_dir.c_str());
    if (!dir) {
        std::cerr << "错误: 无法打开视频目录 " << video_dir << std::endl;
        return video_files;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string filename = entry->d_name;
        if (filename == "." || filename == "..") continue;

        std::string full_path = video_dir + filename;
        struct stat path_stat;
        if (stat(full_path.c_str(), &path_stat) == 0 && S_ISREG(path_stat.st_mode)) {
            std::string ext = get_extension(filename);
            if (ext == ".mp4" || ext == ".avi" || ext == ".mov") {
                video_files.push_back(full_path);
            }
        }
    }
    closedir(dir);

    // 按文件名排序
    std::sort(video_files.begin(), video_files.end());

    return video_files;
}

/**
 * @brief 初始化检测器
 * @param model_path 模型文件路径
 * @return detection::Detect 初始化后的检测器
 */
detection::Detect initialize_detector(const std::string& model_path) {
    detection::Detect detector(model_path, EnemyColor::BLUE);
    return detector;
}

///////////////////////////////////////////////////////////////////////////
// 主函数
///////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
    // 获取当前工作目录
    std::string current_path = get_current_working_directory();
    if (current_path.empty()) {
        return -1;
    }

    // 设置模型路径和视频路径
    std::string model_path, video_dir;
    setup_paths(current_path, model_path, video_dir);

    // 检查模型文件是否存在
    if (!file_exists(model_path)) {
        std::cerr << "错误: 模型文件不存在: " << model_path << std::endl;
        return -1;
    }

    // 检查视频目录是否存在
    if (!file_exists(video_dir)) {
        std::cerr << "错误: 视频目录不存在: " << video_dir << std::endl;
        return -1;
    }

    // 扫描视频文件
    std::vector<std::string> video_files = scan_video_files(video_dir);

    if (video_files.empty()) {
        std::cerr << "错误: 在 " << video_dir << " 中没有找到视频文件" << std::endl;
        return -1;
    }

    try {
        // 初始化检测器
        detection::Detect detector = initialize_detector(model_path);

        // 测试所有视频文件
        std::vector<TestResult> all_results;
        for (const auto& video : video_files) {
            auto result = testVideo(video, detector);
            all_results.push_back(std::move(result));
        }

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
