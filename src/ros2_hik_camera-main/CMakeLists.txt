cmake_minimum_required(VERSION 3.8)
project(hik_camera)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set
if(NOT COMMAND ros2_find_workspace_root)
  function(ros2_find_workspace_root _result)
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

if(NOT COMMAND ros2_find_openvino_dir)
  function(ros2_find_openvino_dir _result)
    ros2_find_workspace_root(_workspace_root)
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
  ros2_find_openvino_dir(_detected_openvino_dir)
  if(_detected_openvino_dir)
    set(OpenVINO_DIR "${_detected_openvino_dir}" CACHE PATH "OpenVINO runtime cmake directory")
  else()
    message(WARNING "OpenVINO directory not auto-detected; set OpenVINO_DIR if needed")
  endif()
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ros2_find_workspace_root(_workspace_root)
if(NOT _workspace_root)
  message(FATAL_ERROR "Unable to locate workspace root for dependency paths")
endif()

get_filename_component(DETECTION_DIR "${_workspace_root}/src/rm_auto_aim/detection" ABSOLUTE)
get_filename_component(RECIVE_DIR "${_workspace_root}/src/recive_pkg" ABSOLUTE)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/hik_camera_node.cpp
)

# find dependencies
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(Eigen3 REQUIRED)


# Include directories
include_directories(${EIGEN3_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include  
                    ${OpenCV_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${sensor_msgs_INCLUDE_DIRS}
                    ${geometry_msgs_INCLUDE_DIRS}
                    ${std_msgs_INCLUDE_DIRS}
                    ${DETECTION_DIR}/include
                    ${DETECTION_DIR}/Eigen
                    ${DETECTION_DIR}/TrackerHeader
                    ${RECIVE_DIR}/include
                    )




target_include_directories(${PROJECT_NAME} PUBLIC hikSDK/include ${OpenVINO_LIBRARIES})

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/amd64)
  install(
    DIRECTORY hikSDK/lib/amd64
    DESTINATION lib
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/amd64)
  install(
    DIRECTORY hikSDK/lib/amd64
    DESTINATION lib
  )
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(${PROJECT_NAME}
  FormatConversion
  MediaProcess
  MvCameraControl
  MVRender
  MvUsb3vTL
  openvino::runtime
  ${OpenCV_INCLUDE_DIRS}
  ${OpenVINO_LIBRARY_DIRS}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN hik_camera::HikCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
