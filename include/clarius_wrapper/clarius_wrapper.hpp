#ifndef CLARIUS_WRAPPER_HPP
#define CLARIUS_WRAPPER_HPP

#define KEYDIR "/tmp/clarius"

// Standard & OpenCV
#include <opencv2/opencv.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>

// CvBridge
#include <cv_bridge/cv_bridge.h>

// Clarius SDK
#include "cast_app.h"

// ProcessedImageFn cannot be a member function of the class ImagePublisher and
// the only way to pass the image to the class is to use a global variable. This
// is not ideal, but it is the only way to make it work with the current design
// of the Clarius APIs.
// Struct to hold the latest ultrasound image context
struct ImgContext {
  cv::Mat us_image;
  int width = 0;
  int height = 0;
  int channels = 0;
  bool newImageReceived = false;
};

// Global image context for callback usage
extern ImgContext imgContext;

// Callback function for receiving images from Clarius
void StoreImageFn(const void *newImage, const CusProcessedImageInfo *nfo,
                  int npos, const CusPosInfo *pos);

class ImagePublisher : public rclcpp::Node {
public:
  ImagePublisher(
      const std::string &node_name,
      const rclcpp::NodeOptions &options =
          rclcpp::NodeOptions()
              .allow_undeclared_parameters(true)
              .automatically_declare_parameters_from_overrides(true));

  int initializeParameters();
  int createConnection();
  int destroyConnection();

private:
  void publishUSImage();
  void
  enableFreeze(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // ROS2 entities
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr us_image_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_freeze_service_;
  rclcpp::TimerBase::SharedPtr image_publisher_timer_;

  // Parameters
  std::string frame_id_;
  std::string ipAddr_;
  std::string us_image_topic_name_;
  unsigned int port_ = 0;

  // Clarius SDK initialization parameters
  CusInitParams initParams_;
};

#endif // CLARIUS_WRAPPER_HPP
