cmake_minimum_required(VERSION 3.8)
project(assistant_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


# Specify the path to your source code
add_executable(assistant_camera_node src/assistant_camera_node.cpp)

# Include the OpenCV directories
target_include_directories(assistant_camera_node PUBLIC ${OpenCV_INCLUDE_DIRS})

# Link OpenCV libraries
target_link_libraries(assistant_camera_node ${OpenCV_LIBS})

# Declare ament dependencies
ament_target_dependencies(assistant_camera_node
        rclcpp
        cv_bridge
        sensor_msgs
        OpenCV
)

# Install executable
install(TARGETS
  assistant_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
