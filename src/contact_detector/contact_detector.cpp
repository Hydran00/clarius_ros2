#include "contact_detector/contact_detector.hpp"

// Constructor
ContactDetector::ContactDetector(const std::string &node_name,
                                 const rclcpp::NodeOptions &options)
    : Node(node_name, "", options) {
  // Retrieve parameters
  ft_sensor_topic_name_ =
      this->get_parameter("ft_sensor_topic_name").as_string();
  us_freeze_service_name_ =
      this->get_parameter("us_freeze_service_name").as_string();
  force_threshold_ = this->get_parameter("force_threshold").as_double();

  RCLCPP_WARN(this->get_logger(),
              "Reading Force Torque sensor data from topic: %s",
              ft_sensor_topic_name_.c_str());

  freeze_service_client_ =
      this->create_client<std_srvs::srv::SetBool>(us_freeze_service_name_);
  RCLCPP_WARN(this->get_logger(), "Contact Detector initialized.");

  // Create subscriber to the force torque sensor topic
  ft_sensor_subscriber_ =
      this->create_subscription<geometry_msgs::msg::WrenchStamped>(
          ft_sensor_topic_name_, rclcpp::SensorDataQoS(),
          std::bind(&ContactDetector::ftSensorCallback, this,
                    std::placeholders::_1));
}
void ContactDetector::ftSensorCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  // Check if the force exceeds a threshold
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  if (!in_contact_) {
    if (std::abs(msg->wrench.force.x) > force_threshold_ ||
        std::abs(msg->wrench.force.y) > force_threshold_ ||
        std::abs(msg->wrench.force.z) > force_threshold_) {
      RCLCPP_WARN(this->get_logger(), "Contact detected!");
      // Call the freeze service
      request->data = false; // Set to false to unfreeze
      in_contact_ = true;
      changed_state_ = true;
    }
  }
  if (in_contact_) {
    if (std::abs(msg->wrench.force.x) < force_threshold_ &&
        std::abs(msg->wrench.force.y) < force_threshold_ &&
        std::abs(msg->wrench.force.z) < force_threshold_) {
      RCLCPP_WARN(this->get_logger(), "Contact released!");
      in_contact_ = false;
      request->data = true; // Set to true to freeze
      changed_state_ = true;
    }
  }
  if (!changed_state_) {
    return;
  }
  auto result_future = freeze_service_client_->async_send_request(request);
  
  changed_state_ = false;
  // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
  //                                        result_future) ==
  //     rclcpp::FutureReturnCode::SUCCESS) {
  //   RCLCPP_WARN(this->get_logger(), "Freeze service called successfully.");
  // } else {
  //   RCLCPP_ERROR(this->get_logger(), "Failed to call freeze service.");
  // }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ContactDetector>("contact_detector");
  while (rclcpp::ok()) {
    // check for freeze service
    if (node->freeze_service_client_->wait_for_service(
            std::chrono::seconds(1))) {
      RCLCPP_WARN(node->get_logger(), "Found freeze service");
      break;
    }
  }
  auto multithreaded_executor =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  multithreaded_executor->add_node(node);
  multithreaded_executor->spin();
  rclcpp::shutdown();
  return 0;
}