cmake_minimum_required(VERSION 3.8)
project(clarius_ros2)

find_package(ament_cmake REQUIRED)
# Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Link directory
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/cast)

# Executable
add_executable(${PROJECT_NAME} src/main.cpp)

# Link the shared library and pthread
target_link_libraries(${PROJECT_NAME} cast pthread)

# Install the executable
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# ROS 2 support
ament_package()
