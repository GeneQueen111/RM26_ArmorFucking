#ifndef YOLO_DETECTION_HPP_
#define YOLO_DETECTION_HPP_

#include <iostream>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include "traditional_detector.hpp"
#include "data.hpp"
#include "inference_engine.hpp"

// 测试模式宏定义
#define TEST_MODE

// 枪管与相机偏移量常量（单位：米）
constexpr double GUN_CAM_DISTANCE_X = 0.0;
constexpr double GUN_CAM_DISTANCE_Y = 0.0;
constexpr double GUN_CAM_DISTANCE_Z = 0.0;

namespace detection
{
    /**
     * @brief 装甲板检测类
     */
    class YoloDetector
    {
    public:
        double fps;  ///< 帧率

        YoloDetector() = default;  ///< 默认构造函数
        YoloDetector(const YoloDetector&) = delete;  ///< 禁止拷贝构造
        YoloDetector& operator=(const YoloDetector&) = delete;  ///< 禁止拷贝赋值
        YoloDetector(YoloDetector&&) = default;  ///< 默认移动构造
        YoloDetector& operator=(YoloDetector&&) = default;  ///< 默认移动赋值

        /**
         * @brief 构造函数
         * @param model_path 模型路径
         * @param detect_color 检测颜色
         * @param video_path 视频路径（可选）
         */
        YoloDetector(std::string& model_path, EnemyColor detect_color, std::string video_path = "");

        /**
         * @brief 构造函数重载（匹配 TX2 分支）
         * @param model_path 模型路径
         * @param detect_color 检测颜色
         * @param ifcountTime 是否计时（TX2 分支参数）
         */
        YoloDetector(std::string& model_path, EnemyColor detect_color, bool ifcountTime);

        ~YoloDetector();

        /**
         * @brief 设置检测颜色
         */
        void set_detect_color(EnemyColor color) { detect_color_ = color; }

        /**
         * @brief 获取检测颜色
         */
        EnemyColor get_detect_color() const { return detect_color_; }

        std::vector<YoloArmorData>& detect(const cv::Mat& inputMat);

        /**
         * @brief 在图像上绘制检测结果
         * @param image 输入图像
         * @param datas 装甲板数据
         */
        void drawObject(cv::Mat& image, std::vector<YoloArmorData>& datas);
        
        /**
         * @brief Sigmoid函数
         * @param x 输入值
         * @return Sigmoid计算结果
         */
        static double sigmoid(double x);
        
        /**
         * @brief 获取当前帧的装甲板数据
         * @return 装甲板数据引用
         */
        std::vector<YoloArmorData>& getdata();

        /**
         * @brief 清空检测结果
         */
        void clear() { m_armors_datas.clear(); }

        /**
         * @brief 开始持续检测（使用视频或默认摄像头）
         */
        void start_detection();
        
        /**
         * @brief 开始检测单帧图像
         * @param input_image 单帧输入
         */
        void start_detection(const cv::Mat& input_image);

        #ifdef TEST_MODE
        /**
         * @brief 格式化打印测试数据
         */
        void format_print_data_test();

        /**
         * @brief 显示图像（测试模式）
         */
        void showImage();
        #endif

        /**
         * @brief 运行检测主循环
         */
        void run();

    private:
        /// 推理引擎
        InferenceEngine m_inference_engine;
        /// 视频捕获对象
        cv::VideoCapture m_cap;

        /// 原始帧
        cv::Mat m_frame;
        /// 处理后的图像
        cv::Mat m_img;


        /// 当前帧的装甲板数据
        std::vector<YoloArmorData> m_armors_datas;

        /// 检测颜色
        EnemyColor detect_color_;

        /// 传统视觉检测器
        std::unique_ptr<TraditionalDetector> m_traditional_detector;

        /**
         * @brief 推理过程
         */
        void infer();

        /**
         * @brief 解析检测结果
         * @param output_buffer 网络输出buffer
         * @param boxes 输出边界框
         * @param num_class 输出数字类别
         * @param color_class 输出颜色类别
         * @param confidences 输出置信度
         * @param fourPointModel 输出四点坐标
         */
        void parseDetections(const cv::Mat& output_buffer,
                           std::vector<cv::Rect>& boxes,
                           std::vector<int>& num_class,
                           std::vector<int>& color_class,
                           std::vector<float>& confidences,
                           std::vector<std::vector<cv::Point>>& fourPointModel);

        /**
         * @brief 应用非极大值抑制
         * @param boxes 边界框
         * @param confidences 置信度
         * @param indices 输出有效索引
         */
        void applyNMS(const std::vector<cv::Rect>& boxes,
                     const std::vector<float>& confidences,
                     std::vector<int>& indices);

        /**
         * @brief 构建装甲板数据
         * @param indices NMS后的有效索引
         * @param fourPointModel 四点坐标
         * @param num_class 数字类别
         * @param color_class 颜色类别
         */
        void buildArmorData(const std::vector<int>& indices,
                          const std::vector<std::vector<cv::Point>>& fourPointModel,
                          const std::vector<int>& num_class,
                          const std::vector<int>& color_class);

        /**
         * @brief 清理堆内存
         */
        void clearHeap();
    };

}  // namespace detection

#endif  // YOLO_DETECTION_HPP_
