cmake_minimum_required(VERSION 3.20)
project(detection VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


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
      "/opt/openvino"
    )

    set(_suffixes ""
      "/runtime/cmake"
      "/runtime"
    )

    set(_config_names
      "OpenVINOConfig.cmake"
      "openvinoConfig.cmake"
    )

    foreach(_base IN LISTS _candidates)
      foreach(_suffix IN LISTS _suffixes)
        set(_candidate "${_base}${_suffix}")
        foreach(_config IN LISTS _config_names)
          if(_candidate AND EXISTS "${_candidate}/${_config}")
            set(${_result} "${_candidate}" PARENT_SCOPE)
            return()
          endif()
        endforeach()
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

find_package(OpenVINO REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS} 
    ${OpenVINO_INCLUDE_DIRS}
)

include_directories(./include)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(${PROJECT_NAME} main.cpp ${SOURCES})
add_executable(test_aim_node test_aim_node.cpp ${SOURCES})
# message(${SOURCES})


target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${OpenCV_INCLUDE_DIRS}
    ${OpenVINO_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${OpenVINO_LIBRARIES}  
    ${OpenCV_LIBS}              
    openvino::runtime
)

target_compile_definitions(test_aim_node PRIVATE TEST_MODE)

target_link_libraries(test_aim_node PRIVATE
    ${OpenVINO_LIBRARIES}
    ${OpenCV_LIBS}
    openvino::runtime
)
