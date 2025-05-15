#include "clarius_wrapper/clarius_wrapper.hpp"

// Global instance for image data
ImgContext imgContext;
bool freeze_state = true;

// Mutexes for thread safety
std::mutex imgMutex;
std::mutex freezeMutex;
// Image callback from Clarius SDK
void StoreImageFn(const void *newImage, const CusProcessedImageInfo *nfo,
                  int npos, const CusPosInfo *pos) {
  (void)pos; // Unused

  if (!newImage || !nfo)
    return;
  std::lock_guard<std::mutex> lock(imgMutex);
  imgContext.width = nfo->width;
  imgContext.height = nfo->height;
  imgContext.channels = nfo->bitsPerPixel / 8;
  // Correct the format from ARGB to BGRA
  cv::Mat rawImage(nfo->height, nfo->width, CV_8UC4,
                   const_cast<void *>(newImage));
  cv::cvtColor(rawImage, imgContext.us_image,
               cv::COLOR_BGRA2RGBA); // Swaps BGR to RGB and keeps alpha

  imgContext.newImageReceived = true;
}
void FreezeCallbackFn(int val) {
  std::lock_guard<std::mutex> lock(freezeMutex);
  // Update the freeze state
  freeze_state = val;
}

// Constructor
ImagePublisher::ImagePublisher(const std::string &node_name,
                               const rclcpp::NodeOptions &options)
    : Node(node_name, "", options) {
  // Retrieve parameters
  us_image_topic_name_ = this->get_parameter("us_image_topic_name").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  ipAddr_ = this->get_parameter("ip_address").as_string();
  port_ = static_cast<uint>(this->get_parameter("port").as_int());

  RCLCPP_INFO(this->get_logger(), "Publishing US image to topic: %s",
              us_image_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Connecting to Clarius at %s:%u",
              ipAddr_.c_str(), port_);

  us_image_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>(us_image_topic_name_, 10);
  enable_freeze_service_ = this->create_service<std_srvs::srv::SetBool>(
      "enable_freeze", std::bind(&ImagePublisher::enableFreeze, this,
                                 std::placeholders::_1, std::placeholders::_2));

  // Setup timer
  image_publisher_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / 30.0),
                              std::bind(&ImagePublisher::publishUSImage, this));

  RCLCPP_INFO(this->get_logger(), "Node initialized.");
}

void ImagePublisher::publishUSImage() {
  std::lock_guard<std::mutex> lock(imgMutex);
  if (imgContext.us_image.empty()) {
    RCLCPP_WARN(this->get_logger(), "No image received yet.");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Image size: %d x %d", imgContext.width,
              imgContext.height);
  cv::imshow("US Image", imgContext.us_image);
  cv::waitKey(1);
  auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgra8",
                                      imgContext.us_image.clone())
                       .toImageMsg();
  image_msg->header.frame_id = frame_id_;
  image_msg->header.stamp = this->now();

  us_image_publisher_->publish(*image_msg);
  // imgContext.newImageReceived = false;
}

void ImagePublisher::enableFreeze(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  std::lock_guard<std::mutex> lock(freezeMutex);
  std::string req = request->data ? "freeze" : "unfreeze";
  RCLCPP_INFO(this->get_logger(), "Request to %s probe", req.c_str());
  if (freeze_state == request->data) {
    response->success = false;
    response->message =
        "Freeze state already set to " + req + ". No action taken.";
    return;
  } else {
    auto result = cusCastUserFunction(Freeze, 0, nullptr);
    if (result != 0) {
      response->success = false;
      response->message = "Freeze state failed to change.";
      return;
    }
    freeze_state = request->data;
    response->success = true;
    response->message = "Freeze state changed to " + req + ".";
    RCLCPP_INFO(this->get_logger(), "Probe set to %s", req.c_str());
  }
  // freeze_state value is updated by the callback function
}

int ImagePublisher::initializeParameters() {
  RCLCPP_INFO(this->get_logger(), "Initializing Clarius parameters...");

  initParams_ = cusCastDefaultInitParams();
  initParams_.args.argc = 0;
  initParams_.args.argv = nullptr;
  initParams_.storeDir = KEYDIR;

  initParams_.newProcessedImageFn = StoreImageFn;
  initParams_.newRawImageFn = cast_app::newRawImageFn;
  initParams_.newSpectralImageFn = cast_app::newSpectralImageFn;
  initParams_.newImuDataFn = cast_app::newImuData;
  initParams_.freezeFn = FreezeCallbackFn;
  initParams_.buttonFn = cast_app::buttonFn;
  initParams_.progressFn = cast_app::progressFn;
  initParams_.errorFn = cast_app::errorFn;

  initParams_.width = 1280;
  initParams_.height = 720;

  if (cusCastInit(&initParams_) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize Clarius caster");
    return -1;
  }

  return 0;
}

int ImagePublisher::createConnection() {
  RCLCPP_INFO(this->get_logger(), "Creating Clarius connection...");

  return cusCastConnect(ipAddr_.c_str(), port_, "research",
                        [](int imagePort, int imuPort, int swRevMatch) {
                          if (imagePort == CUS_FAILURE) {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                                         "Could not connect to scanner");
                          } else {
                            RCLCPP_INFO_STREAM(
                                rclcpp::get_logger("rclcpp"),
                                "Connected: image port = "
                                    << imagePort << ", imu port = " << imuPort
                                    << ", software revision match = "
                                    << swRevMatch);
                          }
                        });
}

int ImagePublisher::destroyConnection() { return cusCastDestroy(); }

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::cout << "Opencv version: " << CV_VERSION << std::endl;
  auto node = std::make_shared<ImagePublisher>("image_publisher");
  int success = node->initializeParameters();
  if (success < 0) {
    RCLCPP_ERROR(node->get_logger(), "Failed to init params");
    return CUS_FAILURE;
  }
  node->createConnection();

  RCLCPP_INFO(node->get_logger(), "Spinning node");
  rclcpp::spin(node);
  node->destroyConnection();
  return 0;
}