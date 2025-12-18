#ifndef DATA_HPP
#define DATA_HPP

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <vector>
#include <string>

/**
 * @brief 全局颜色枚举
 */
enum class EnemyColor { RED = 0, BLUE = 1 };

enum class ArmorType { SMALL, LARGE, INVALID }; //装甲板类型: 小、大、无效
const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

class Light : public cv::RotatedRect
{
public:
    Light() = default;
    explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
    {
        cv::Point2f p[4];
        box.points(p);
        std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
        top = (p[0] + p[1]) / 2; 
        bottom = (p[2] + p[3]) / 2;

        length = cv::norm(top - bottom); //灯条长度
        width = cv::norm(p[0] - p[1]); //灯条宽度

        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y)); //弧度制，与y轴夹角
        tilt_angle = tilt_angle / CV_PI * 180; //弧度转角度
    }


    EnemyColor color;
    cv::Point2f top, bottom;
    double length;
    double width;
    float tilt_angle;
};

struct TraditionalArmorData
{
  TraditionalArmorData() = default;
  TraditionalArmorData(const Light & l1, const Light & l2) 
  {
    if (l1.center.x < l2.center.x) {
      left_light = l1, right_light = l2; 
    } else {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
  }

  //灯条部分
  Light left_light, right_light;
  cv::Point2f center;
  ArmorType type;

  //数字识别部分
  cv::Mat number_img;
  std::string number;
  float confidence;
  std::string classfication_result;
};

namespace detection
{
  // Alias to allow using TraditionalArmorData within the detection namespace
  using TraditionalArmorData = ::TraditionalArmorData;

  /**
   * @brief 装甲板数据结构
   */
  struct YoloArmorData
  {
      cv::Point center_point;    ///< 目标中心点
      cv::Point optical_center;  ///< 光心点
      float delta_x;             ///< 目标中心点与光心点的x差值
      float delta_y;             ///< 目标中心点与光心点的y差值
      int flag = 0;              ///< 判断是否有装甲板

      cv::Point p1;  ///< 左上角点
      cv::Point p2;  ///< 右上角点
      cv::Point p3;  ///< 右下角点
      cv::Point p4;  ///< 左下角点

      int ID = 0;    ///< 装甲板ID
      EnemyColor color;   ///< 装甲板颜色
  };
}

#endif
