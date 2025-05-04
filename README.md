# clarius_ros2
This is a ROS2 wrapper for the Clarius ultrasound devices. Currenlty, the wrapper publishes the ultrasound image on a ROS2 topic and provides a service to start and stop the ultrasound stream. New features will be added in the future to port all the Clarius API functionalities to ROS2 using topics and services.
## Supported Devices
Every devices that is supported by the Clarius Cast API should work with this wrapper. To be able to use the wrapper you need the buy the [Clarius Research Toolkit](https://clarius.com/scanners/research/).
## Tested platforms
- Ubuntu 22.04
- ROS2 Humble
## Installation
1. Clone the repository into your ROS2 workspace:
```bash
cd <your_ros2_workspace>/src
git clone git@github.com:Hydran00/clarius_ros2.git
```
2. Build your workspace
```bash
cd <your_ros2_workspace>
colcon build --symlink-install
```
## Run the wrapper
1. Source your workspace
```bash
source <your_ros2_workspace>/install/setup.bash
```
2. Run the wrapper
```bash
ros2 launch clarius_ros2 us_stream.launch.py us_image_topic_name:=us_image
```
3. check the image topic in Rviz and available services with `ros2 service list`