cmake_minimum_required(VERSION 3.8)
project(clarius_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
# openCV
# find_package(OpenCV REQUIRED)
# Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/cast_qt)
# Include openCV headers
include_directories(${OpenCV_INCLUDE_DIRS})
# Link directory
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/cast)

# Executable
add_executable(clarius_wrapper src/clarius_wrapper/clarius_wrapper.cpp)
add_executable(contact_detector src/contact_detector/contact_detector.cpp)


# Link the shared library
target_link_libraries(clarius_wrapper cast ${OpenCV_LIBS})

ament_target_dependencies(clarius_wrapper
  rclcpp
  sensor_msgs
  std_srvs
  cv_bridge
)
ament_target_dependencies(contact_detector
  rclcpp
  sensor_msgs
  std_srvs
)

# Install the executable
install(TARGETS clarius_wrapper 
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS contact_detector 
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
