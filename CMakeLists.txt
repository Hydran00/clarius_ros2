cmake_minimum_required(VERSION 3.8)
project(clarius_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
# openCV
find_package(OpenCV REQUIRED)
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
add_executable(caster src/caster/main.cpp)

# Link the shared library and pthread
target_link_libraries(caster cast pthread ${OpenCV_LIBS})

ament_target_dependencies(caster
  rclcpp
  sensor_msgs
  std_srvs
  cv_bridge
)
# Install the executable
install(TARGETS caster
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
