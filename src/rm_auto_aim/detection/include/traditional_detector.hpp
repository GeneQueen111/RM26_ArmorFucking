#ifndef DETECTOR_HPP
#define DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include "data.hpp"
// #include "number_classifier.hpp"
#include <vector>

// 前向声明，避免循环依赖
namespace detection { struct YoloArmorData; }

class TraditionalDetector
{
public:
  struct LightParams
  {
    // width / height
    double min_ratio = 0.1;
    double max_ratio = 0.4;
    //垂直于y轴的最大倾斜角
    double max_angle = 40.0;
  };

  struct ArmorParams
  {
    double min_light_ratio = 0.7;
    //灯条间距与灯条高度比值
    double min_small_center_distance = 0.8;
    double max_small_center_distance = 3.2;
    double min_large_center_distance = 3.2;
    double max_large_center_distance = 5.5;
    //水平夹角最大值
    double max_angle = 40.0;
  };

  TraditionalDetector(const int & bin_thres, const LightParams & l, const ArmorParams & a, EnemyColor detect_color);

  /**
   * @brief 设置检测颜色
   */
  void set_detect_color(EnemyColor color) { detect_color_ = color; }

  /**
   * @brief 获取检测颜色
   */
  EnemyColor get_detect_color() const { return detect_color_; }

  /**
   * @brief 结合 YOLO 结果的检测入口；必须先完成 YOLO 检测
   * @param input 原始图像
   * @param yolo_armors YOLO 检测结果（为空则直接返回空）
   * @return 检测到的装甲板引用
   */
  const std::vector<detection::TraditionalArmorData>& detect(const cv::Mat & input, const std::vector<detection::YoloArmorData> & yolo_armors);

  /**
   * @brief 清空检测结果
   */
  void clear() { armors_.clear(); lights_.clear(); }

  cv::Mat preprocessImage(const cv::Mat & input); //预处理，得到二值图
  std::vector<Light> findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img); //检测灯条
  std::vector<TraditionalArmorData> matchLights(const std::vector<Light> & lights); //匹配灯条，得到装甲板

  //调试
  cv::Mat getAllNumbersImage(); //获取所有装甲板数字图像的拼接图

  /**
   * @brief 获取检测到的装甲板
   */
  const std::vector<TraditionalArmorData>& getArmors() const { return armors_; }

  /**
   * @brief 获取图像中的感兴趣区域(ROI)
   * @param image 输入图像
   * @param points 四个角点坐标
   * @param ROI 输出的感兴趣区域
   */
  void get_roi(cv::Mat& image, std::vector<cv::Point>& points, cv::Mat& ROI);

  int binary_thres = 50;
  EnemyColor detect_color_;  // 检测颜色
  LightParams l; //灯条参数
  ArmorParams a; //装甲板参数

  //std::unique_ptr<NumberClassifier> classifier;

  

private:
  bool isLight(const Light & possible_light); //判断是否为灯条
  bool containLight(const Light & light_1, const Light & light_2, const std::vector<Light> & lights); 
  ArmorType isArmor(const Light & light_1, const Light & light_2); //判断是否为装甲板及类型

  std::vector<Light> lights_; //检测到的灯条
  std::vector<TraditionalArmorData> armors_; //检测到的装甲板

};




#endif
