#ifndef __IMAGE_PUBLISHER_HPP__
#define __IMAGE_PUBLISHER_HPP__
#define KEYDIR "/tmp/clarius"
#include <opencv2/opencv.hpp>
// ros2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
// cv_bridge
#include <cv_bridge/cv_bridge.h>

#include "cast_app.h"

// Global variables should be avoided, but in this case, they are needed sinche
// ProcessedImageFn cannot be a member function of the class ImagePublisher and
// the only way to pass the image to the class is to use a global variable. This
// is not ideal, but it is the only way to make it work with the current design
// of the Clarius APIs.
struct ImgContext {
  cv::Mat us_image;
  int width, height, channels;
  bool newImageReceived = false;
};
ImgContext imgContext;

void StoreImageFn(const void* newImage, const CusProcessedImageInfo* nfo,
                  int npos, const CusPosInfo* pos) {
  (void)pos;
  imgContext.width = nfo->width;
  imgContext.height = nfo->height;
  imgContext.channels = nfo->bitsPerPixel / 8;
  // load us image
  imgContext.us_image = cv::Mat(imgContext.height, imgContext.width, CV_8UC4,
                                const_cast<void*>(newImage));
  imgContext.newImageReceived = true;
  // cv::imshow("Clarius US Image", us_image);
  // cv::waitKey(1);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Image received");
  if (!newImage || !nfo) {
    return;
  }
  // if (streamOutput_) {
  //   RCLCPP_INFO_STREAM(
  //       this->get_logger(),
  //       "new image (" << counter_++ << "): " << nfo->width << " x "
  //                     << nfo->height << " @ " << nfo->bitsPerPixel << " bpp.
  //                     @ "
  //                     << nfo->imageSize << "bytes. @ " <<
  //                     nfo->micronsPerPixel
  //                     << " microns per pixel. imu points: " << npos
  //                     << std::flush);
  // }
}
class ImagePublisher : public rclcpp::Node {
 public:
  ImagePublisher(const std::string& node_name,
                 const rclcpp::NodeOptions& options =
                     rclcpp::NodeOptions()
                         .allow_undeclared_parameters(true)
                         .automatically_declare_parameters_from_overrides(true))
      : Node(node_name, "", options) {
    // get parameters
    us_image_topic_name_ =
        this->get_parameter("us_image_topic_name").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    ipAddr_ = this->get_parameter("ip_address").as_string();
    port_ = (uint)this->get_parameter("port").as_int();
    // print parameters
    RCLCPP_INFO(this->get_logger(), "Publishing US image to topic: %s",
                us_image_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "Connecting to Clarius with ip_address: %s, port: %d",
                ipAddr_.c_str(), port_);
    // create publishers and services
    us_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        us_image_topic_name_, 10);
    enable_freeze_service_ = this->create_service<std_srvs::srv::SetBool>(
        "enable_freeze",
        std::bind(&ImagePublisher::enableFreeze, this, std::placeholders::_1,
                  std::placeholders::_2));
    // initialize parameters
    initParams_ = cusCastDefaultInitParams();
    // create wall timer
    image_publisher_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(0.01),
        std::bind(&ImagePublisher::publishUSImage, this));
    RCLCPP_INFO(this->get_logger(), "Node started");
  }
  void enableFreeze(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Incoming to %s the probe",
                request->data ? "freeze" : "unfreeze");
    if (request->data) {
      if (cusCastUserFunction(Freeze, 0, nullptr) < 0)
        RCLCPP_ERROR(this->get_logger(), "Error toggling freeze");
    } else {
      if (cusCastUserFunction(Freeze, 0, nullptr) < 0)
        RCLCPP_ERROR(this->get_logger(), "Error toggling unfreeze");
    }
    response->success = true;
    response->message = "Freeze state changed";
  }
  void publishUSImage() {
    // publish image
    if (imgContext.newImageReceived) {
      cv::imshow("Clarius US Image", imgContext.us_image);
      cv::waitKey(1);
      auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgra8",
                                          imgContext.us_image)
                           .toImageMsg();
      image_msg->header.frame_id = frame_id_;
      image_msg->header.stamp = this->now();
      us_image_publisher_->publish(*image_msg);
      imgContext.newImageReceived = false;
    }
  }
  int initializeParameters() {
    RCLCPP_INFO(this->get_logger(), "Initializing params");
    // overwrite just the important function
    // *initParams_ = cusCastDefaultInitParams();
    initParams_.args.argc = 0;
    initParams_.args.argv = nullptr;
    initParams_.storeDir = KEYDIR;
    initParams_.newProcessedImageFn = StoreImageFn;
    initParams_.newRawImageFn = cast_app::newRawImageFn;
    initParams_.newSpectralImageFn = cast_app::newSpectralImageFn;
    initParams_.newImuDataFn = cast_app::newImuData;
    initParams_.freezeFn = cast_app::freezeFn;
    initParams_.buttonFn = cast_app::buttonFn;
    initParams_.progressFn = cast_app::progressFn;
    initParams_.errorFn = cast_app::errorFn;
    initParams_.width = 1280;
    initParams_.height = 720;
    RCLCPP_INFO(this->get_logger(), "Callback functions assigned");
    if (cusCastInit(&initParams_) < 0) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "could not initialize caster" << std::endl);
      return -1;
    }
    return 0;
  }
  int createConnection() {
    RCLCPP_INFO(this->get_logger(), "Creating connection");
    if (cusCastConnect(
            ipAddr_.c_str(), port_, "research",
            [](int imagePort, int imuPort, int swRevMatch) {
              if (imagePort == CUS_FAILURE)
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                                   "could not connect to scanner" << std::endl);
              else {
                PRINT << "...connected, streaming port: " << imagePort
                      << " -- try rebooting the probe or check firewall "
                         "settings if no image "
                         "callback received";
                if (imuPort > 0) {
                  PRINT << "imu now streaming at port: " << imuPort;
                } else {
                  PRINT << "imu streaming off";
                }
                if (swRevMatch == CUS_FAILURE)
                  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                                     "software revisions do not match, that "
                                     "is not necessarily a problem"
                                         << std::endl);
              }
            }) < 0) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "connection attempt failed" << std::endl);
      return CUS_FAILURE;
    }
    return 0;
  }
  int destroyConnection() { return cusCastDestroy(); }
  // public attributes

 private:
  // private attributes
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr us_image_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_freeze_service_;
  rclcpp::TimerBase::SharedPtr image_publisher_timer_;
  std::string frame_id_, ipAddr_;
  unsigned int port_ = 0;
  std::string us_image_topic_name_;
  // std::shared_ptr<CusInitParams> initParams_;
  CusInitParams initParams_;
};
#endif  // __IMAGE_PUBLISHER_HPP__getInitParams