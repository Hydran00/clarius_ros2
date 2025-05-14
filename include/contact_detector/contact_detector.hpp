#ifndef CONTACT_DETECTOR_HPP
#define CONTACT_DETECTOR_HPP

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>

class ContactDetector : public rclcpp::Node {
public:
  // Constructor
  ContactDetector(
      const std::string &node_name,
      const rclcpp::NodeOptions &options =
          rclcpp::NodeOptions()
              .allow_undeclared_parameters(true)
              .automatically_declare_parameters_from_overrides(true));
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr freeze_service_client_;

private:
  // Parameters
  std::string ft_sensor_topic_name_;
  std::string us_freeze_service_name_;
  double force_threshold_ = 10.0; // Default threshold, can be adjusted
  bool in_contact_ = false;
  bool changed_state_ = false;
  // ROS 2 Interfaces
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      ft_sensor_subscriber_;

  // Callback function for force torque sensor data
  void ftSensorCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
};

#endif // CONTACT_DETECTOR_HPP
