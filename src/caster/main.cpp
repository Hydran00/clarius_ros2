#include "cast/cast.h"
#include "clarius_wrapper/image_publisher.hpp"

/// main entry point
/// @param[in] argc # of program arguments
/// @param[in] argv list of arguments
int main(int argc, char* argv[]) {
  // int rcode = init(argc, argv);
  rclcpp::init(argc, argv);
  // create window
  cv::namedWindow("Clarius US Image",
                  cv::WINDOW_NORMAL);  // Makes the window resizable
  // instantiate black window
  // cv::Mat empty_img = cv::Mat::zeros(640, 480, CV_8UC4);
  // cv::imshow("Clarius US Image", empty_img);
  // cv::waitKey(0);

  auto node = std::make_shared<ImagePublisher>("image_publisher");
  int success = node->initializeParameters();
  node->createConnection();
  if (success < 0) {
    ERROR << "failed to create connection" << std::endl;
    return CUS_FAILURE;
  }

  // if (rcode == CUS_SUCCESS) {
  std::atomic_bool quitFlag(false);
  std::thread eventLoop(cast_app::processEventLoop, std::ref(quitFlag));
  eventLoop.join();
  // }

  cusCastDestroy();
  return 0;
}