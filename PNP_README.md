# PnP模块集成说明

## 创建的文件

### 1. include/pnp.hpp
- 头文件，包含PnPSolver类的声明
- 使用`detection`命名空间
- 包含完整的函数声明和常量定义

### 2. src/pnp.cpp
- 实现文件，包含PnPSolver类的所有成员函数实现
- 自动被CMakeLists.txt包含（通过GLOB_RECURSE）

### 3. src/pnp_example.cpp（可选）
- 使用示例代码

## 主要功能

### PnPSolver类提供以下功能：

1. **构造函数**
   - 初始化相机内参矩阵和畸变系数
   - 预计算小装甲板和大装甲板的3D坐标点

2. **solvePnP()**
   - 根据装甲板的2D图像点计算3D位姿
   - 输入：Armor对象、输出：rvec（旋转向量）、tvec（平移向量）
   - 返回：bool（求解是否成功）

3. **calculateDistanceToCenter()**
   - 计算装甲板中心到图像中心的距离
   - 用于评估目标偏离程度

4. **rvecToEuler()**
   - 将旋转向量转换为欧拉角（pitch、yaw、roll）
   - 便于理解和控制

5. **calculateDistance()**
   - 计算相机到目标的3D距离（单位：米）

## 装甲板尺寸常量

- 小装甲板：135mm × 55mm
- 大装甲板：225mm × 55mm

## 坐标系统

- 模型坐标系：x向前，y向左，z向上（右手坐标系）
- 装甲板3D点坐标按顺时针顺序定义

## 使用示例

```cpp
#include "pnp.hpp"

// 1. 初始化相机参数
std::array<double, 9> camera_matrix = {
    1000.0, 0.0, 320.0,    // fx, 0, cx
    0.0, 1000.0, 240.0,    // 0, fy, cy
    0.0, 0.0, 1.0          // 0, 0, 1
};

std::vector<double> dist_coeffs = {0.1, -0.2, 0.0, 0.0, 0.0};

// 2. 创建PnPSolver
detection::PnPSolver solver(camera_matrix, dist_coeffs);

// 3. 求解位姿
cv::Mat rvec, tvec;
bool success = solver.solvePnP(armor, rvec, tvec);

if (success) {
    // 4. 转换为欧拉角
    double pitch, yaw, roll;
    solver.rvecToEuler(rvec, pitch, yaw, roll);

    // 5. 计算距离
    double distance = solver.calculateDistance(tvec);

    std::cout << "Pitch: " << pitch << ", Yaw: " << yaw << ", Roll: " << roll << std::endl;
    std::cout << "Distance: " << distance << "m" << std::endl;
}
```

## CMake集成

由于CMakeLists.txt使用了：
```cmake
file(GLOB_RECURSE SOURCES "src/*.cpp")
```

新创建的`pnp.cpp`文件会自动被包含在构建中，无需手动修改CMakeLists.txt。

## 注意事项

1. 需要包含`armor.hpp`头文件（已存在）
2. 使用OpenCV库（已在CMakeLists.txt中配置）
3. 所有的API都位于`detection`命名空间中
