#include "traditional_detector.hpp"
#include "yolo_detection.hpp"
#include <algorithm>
#include <cmath>
TraditionalDetector::TraditionalDetector(const int & bin_thres, const LightParams & l, const ArmorParams & a, EnemyColor detect_color)
: binary_thres(bin_thres), l(l), a(a), detect_color_(detect_color){};

cv::Mat TraditionalDetector::preprocessImage(const cv::Mat & rgb_img)
{
  if(detect_color_ == EnemyColor::BLUE)
  {
    cv::Mat gray_img;
    cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);

    cv::Mat binary_img;
    cv::threshold(gray_img, binary_img, 50, 255, cv::THRESH_BINARY);
    return binary_img;
    
  }
  
  else if(detect_color_ == EnemyColor::RED)
  {
    int flag = 2; // 0-蓝色通道，1-绿色通道，2-红色通道
    std::vector<cv::Mat> channels;
    cv::Mat frame,thresh_img,binary_img;
    cv::split(rgb_img, channels); //通道分离BGR
    cv::GaussianBlur(channels[flag], frame, cv::Size(5, 5), 0); //高斯模糊，去除细小噪点
    cv::threshold(frame, thresh_img, 50, 255, cv::THRESH_BINARY);   //二值化
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)); 
    cv::morphologyEx(thresh_img, binary_img, cv::MORPH_OPEN, kernel); //开运算，去除灯条边缘噪点

    return binary_img;
  }
  return cv::Mat();
  
}

bool TraditionalDetector::isLight(const Light & light)
{
  // The ratio of light (short side / long side)
  float ratio = light.width / light.length;
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

  bool angle_ok = light.tilt_angle < l.max_angle;

  bool is_light = ratio_ok && angle_ok;

  return is_light;
}

std::vector<Light> TraditionalDetector::findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img)
{
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  vector<Light> lights;
  // this->debug_lights.data.clear();

  for (const auto & contour : contours) {
    if (contour.size() < 5) continue; //过滤轮廓

    auto r_rect = cv::minAreaRect(contour);
    auto light = Light(r_rect);

    if (isLight(light)) {
      auto rect = light.boundingRect();
      if (  // Avoid assertion failed
        0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rbg_img.cols && 0 <= rect.y &&
        0 <= rect.height && rect.y + rect.height <= rbg_img.rows) {
        int sum_r = 0, sum_b = 0;
        auto roi = rbg_img(rect);
        // Iterate through the ROI
        for (int i = 0; i < roi.rows; i++) //行
        { 
          for (int j = 0; j < roi.cols; j++) //列
          {
            if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) 
            {
              
              sum_r += roi.at<cv::Vec3b>(i, j)[2]; //红色
              sum_b += roi.at<cv::Vec3b>(i, j)[0]; //蓝色
              
            }
          }
        }
        // Sum of red pixels > sum of blue pixels ?
        light.color = sum_r > sum_b ? EnemyColor::RED : EnemyColor::BLUE;
        lights.emplace_back(light);
      }
    }
  }

  return lights;
}

std::vector<TraditionalArmorData> TraditionalDetector::matchLights(const std::vector<Light> & lights)
{
  std::vector<TraditionalArmorData> armors;
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      if (light_1->color != detect_color_ || light_2->color != detect_color_) {
        continue;
      };

      if (containLight(*light_1, *light_2, lights)) {
        continue;
      }

      auto type = isArmor(*light_1, *light_2);
      if (type != ArmorType::INVALID) {
        auto armor = TraditionalArmorData(*light_1, *light_2);
        armor.type = type;
        armors.emplace_back(armor);
      }
    }
  }
  return armors;
}

bool TraditionalDetector::containLight(const Light & light_1, const Light & light_2, const std::vector<Light> & lights)  
{
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);

  for (const auto & test_light : lights) 
  {
    if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

    if (
      bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
      bounding_rect.contains(test_light.center)) {
      return true;
    }
  }

  return false;
}

ArmorType TraditionalDetector::isArmor(const Light & light_1, const Light & light_2)//判断是否为装甲板及类型
{
    // Ratio of the length of 2 lights (short side / long side)
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                             : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

  // Distance between the center of 2 lights (unit : light length)
  float avg_light_length = (light_1.length + light_2.length) / 2;
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (a.min_small_center_distance <= center_distance &&
                             center_distance < a.max_small_center_distance) ||
                            (a.min_large_center_distance <= center_distance &&
                             center_distance < a.max_large_center_distance);

  // Angle of light center connection
  cv::Point2f diff = light_1.center - light_2.center; 
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < a.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // Judge armor type
  ArmorType type;
  if (is_armor) {
    type = center_distance > a.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
  } else {
    type = ArmorType::INVALID;
  }
  
  return type;
}

cv::Mat TraditionalDetector::getAllNumbersImage() ///获取所有数字图片
{
  if (armors_.empty()) {
    return cv::Mat(cv::Size(20, 28), CV_8UC1);
  } else {
    std::vector<cv::Mat> number_imgs;
    number_imgs.reserve(armors_.size());
    for (auto & armor : armors_) {
      number_imgs.emplace_back(armor.number_img);
    }
    cv::Mat all_num_img;
    cv::vconcat(number_imgs, all_num_img);
    return all_num_img;
  }
}

cv::Point2f getLineIntersection(const std::pair<cv::Point2f, cv::Point2f>& line1, const std::pair<cv::Point2f, cv::Point2f>& line2) {
    // 提取直线1的两点坐标
    float x1 = line1.first.x, y1 = line1.first.y;
    float x2 = line1.second.x, y2 = line1.second.y;
    // 提取直线2的两点坐标
    float x3 = line2.first.x, y3 = line2.first.y;
    float x4 = line2.second.x, y4 = line2.second.y;

    // 直线1的一般式参数
    float A1 = y2 - y1;
    float B1 = x1 - x2;
    float C1 = x2 * y1 - x1 * y2;

    // 直线2的一般式参数
    float A2 = y4 - y3;
    float B2 = x3 - x4;
    float C2 = x4 * y3 - x3 * y4;

    // 计算分母 D
    float D = A1 * B2 - A2 * B1;

    // 平行或重合返回(-1, -1)
    if (fabs(D) < 1e-6) {
        return cv::Point2f(-1, -1);  // 用(-1,-1)表示无交点
    }

    // 计算并返回交点
    return cv::Point2f(
        (B1 * C2 - B2 * C1) / D,
        (A2 * C1 - A1 * C2) / D
    );
}

void TraditionalDetector::get_roi(cv::Mat& image, std::vector<cv::Point>& points, cv::Mat& ROI)
{
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::Point lt = points[0];  // 左上角
    cv::Point rb = points[2];  // 右下角
    cv::Point lb = points[1];  // 左下角
    cv::Point rt = points[3];  // 右上角
    std::vector<cv::Point> roi_points = {lt, rt, rb, lb}; 
    cv::fillConvexPoly(mask, roi_points, cv::Scalar(255,0,0));
    cv::bitwise_and(image, image,ROI, mask);
}

const std::vector<detection::TraditionalArmorData>& TraditionalDetector::detect(
    const cv::Mat & input,
    const std::vector<detection::YoloArmorData> & yolo_armors)
{
  armors_.clear();
  lights_.clear();

  // std::cout << "[Traditional] yolo_armors.size=" << yolo_armors.size() << std::endl;

  if (input.empty() || yolo_armors.empty()) {
    return armors_;
  }

  // 在 YOLO 框附近扩大一定像素后，使用已有的传统方法做二次确认
  constexpr int kPadding = 20;

  for (const auto & d : yolo_armors) {

    cv::Point lt = cv::Point(d.p1.x-20, d.p1.y-20);  // 左上角
    cv::Point rb = cv::Point(d.p3.x+20, d.p3.y+20);  //

    cv::Point lb = cv::Point(d.p2.x-20, d.p2.y+20);  // 左下角
    cv::Point rt = cv::Point(d.p4.x+20, d.p4.y-20);  // 右上角
    std::vector<cv::Point> points = {lt, rt, rb, lb};

    cv::Mat roi_src = input.clone();
    cv::Mat roi;
    get_roi(roi_src, points, roi);

    cv::Mat binary_img;
    binary_img = preprocessImage(roi);
    lights_ = findLights(roi, binary_img);

    auto detected_armors = matchLights(lights_);
    // 将检测到的装甲板添加到armors_成员变量中
    armors_.insert(armors_.end(), detected_armors.begin(), detected_armors.end());

    // 提取所有装甲板的数字图像
    auto num_img = getAllNumbersImage();
  }

  return armors_;
}
