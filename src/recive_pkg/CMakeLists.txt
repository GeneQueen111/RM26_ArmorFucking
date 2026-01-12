cmake_minimum_required(VERSION 3.8)
project(recive_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置策略以支持 IN_LIST 操作符
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

# 自动定位安装在工作区父目录的 OpenVINO 路径
if(NOT COMMAND rm26_find_workspace_root)
  function(rm26_find_workspace_root _result)
    set(_dir "${CMAKE_CURRENT_LIST_DIR}")
    while(TRUE)
      if(EXISTS "${_dir}/.git")
        set(${_result} "${_dir}" PARENT_SCOPE)
        return()
      endif()
      get_filename_component(_parent "${_dir}" DIRECTORY)
      if(_parent STREQUAL _dir)
        break()
      endif()
      set(_dir "${_parent}")
    endwhile()
    set(${_result} "" PARENT_SCOPE)
  endfunction()
endif()

if(NOT COMMAND rm26_find_openvino_dir)
  function(rm26_find_openvino_dir _result)
    rm26_find_workspace_root(_workspace_root)
    if(NOT _workspace_root)
      set(${_result} "" PARENT_SCOPE)
      return()
    endif()

    get_filename_component(_workspace_parent "${_workspace_root}" DIRECTORY)

    set(_candidates "")
    if(DEFINED ENV{OPENVINO_DIR})
      list(APPEND _candidates "$ENV{OPENVINO_DIR}")
    endif()
    list(APPEND _candidates
      "${_workspace_parent}/openvino_toolkit_2024.6.0"
      "${_workspace_parent}/openvino_toolkit"
      "${_workspace_parent}/openvino_2024.6.0"
      "${_workspace_parent}/openvino"
      "${_workspace_parent}/openvino_venv"
    )

    set(_suffixes ""
      "/runtime/cmake"
      "/runtime"
    )

    foreach(_base IN LISTS _candidates)
      foreach(_suffix IN LISTS _suffixes)
        set(_candidate "${_base}${_suffix}")
        if(_candidate AND EXISTS "${_candidate}/openvinoConfig.cmake")
          set(${_result} "${_candidate}" PARENT_SCOPE)
          return()
        endif()
      endforeach()
    endforeach()

    set(${_result} "" PARENT_SCOPE)
  endfunction()
endif()

if(NOT DEFINED OpenVINO_DIR OR OpenVINO_DIR STREQUAL "")
  rm26_find_openvino_dir(_detected_openvino_dir)
  if(_detected_openvino_dir)
    set(OpenVINO_DIR "${_detected_openvino_dir}" CACHE PATH "OpenVINO runtime cmake directory")
  else()
    message(WARNING "OpenVINO directory not auto-detected; set OpenVINO_DIR if needed")
  endif()
endif()

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

rm26_find_workspace_root(_workspace_root)
if(NOT _workspace_root)
  message(FATAL_ERROR "Unable to locate workspace root for dependency paths")
endif()

get_filename_component(DETECTION_DIR "${_workspace_root}/src/rm_auto_aim/detection" ABSOLUTE)
get_filename_component(RECIVE_DIR "${_workspace_root}/src/recive_pkg" ABSOLUTE)

set(SRC
      ${DETECTION_DIR}/src/yolo_detection.cpp
      ${DETECTION_DIR}/src/traditional_detector.cpp
      ${DETECTION_DIR}/src/detect.cpp
      ${DETECTION_DIR}/src/inference_engine.cpp
      ${DETECTION_DIR}/src/pnp.cpp)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenVINO REQUIRED COMPONENTS Runtime)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio dnn calib3d)
find_package(cv_bridge REQUIRED)


# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${OpenCV_INCLUDE_DIRS}
                    ${sensor_msgs_INCLUDE_DIRS}
                    ${geometry_msgs_INCLUDE_DIRS}
                    ${std_msgs_INCLUDE_DIRS}
                    ${DETECTION_DIR}/include
                    ${RECIVE_DIR}/include
                    )



#add_executable
add_executable(${PROJECT_NAME}
  src/image_subscirber_node.cpp
  main.cpp
  ${SRC}
)

add_executable(debug_display_node
  src/debug_display_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  openvino::runtime
)

# 自动处理ROS2依赖与导入库
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
)

ament_target_dependencies(debug_display_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# 仅链接必要的第三方库，避免拉入大量不必要模块
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  openvino::runtime
)

target_link_libraries(debug_display_node
  ${OpenCV_LIBS}
)

# 按需链接，避免无用的大量 OpenCV 模块被强行拉入
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_link_options(${PROJECT_NAME} PRIVATE -Wl,--as-needed -Wl,--no-undefined)
endif()


# 安装目标到lib目录
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS debug_display_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
