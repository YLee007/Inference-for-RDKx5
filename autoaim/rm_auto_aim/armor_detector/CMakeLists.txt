cmake_minimum_required(VERSION 3.10)
project(armor_detector)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(hobot_cv REQUIRED)  # 添加对 hobot_cv 库的依赖
find_package(hbm_img_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

# 添加对 hobot_cv 的支持
target_include_directories(${PROJECT_NAME} PUBLIC 
  ${OpenCV_INCLUDE_DIRS}
  ${hobot_cv_INCLUDE_DIRS}  
)

target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBS}
  ${hobot_cv_LIBRARIES}  
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
)

# Register Yolo11Node component and provide an executable for convenience
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::Yolo11Node
  EXECUTABLE yolo11_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(test_node_startup test/test_node_startup.cpp)
  target_link_libraries(test_node_startup ${PROJECT_NAME})


endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  model
)

