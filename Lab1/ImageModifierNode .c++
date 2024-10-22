// #include <chrono>
// #include <functional>
// #include <memory>

// using namespace std::chrono_literals;

// #include "rclcpp/rclcpp.hpp"
// #include "/camera/image_raw.hpp"


// class publisher : public rclcpp::Node


// public : 

//     publisher() : node("publisher");
//     publisher_ = this->create_publisher<camera::image_raw>("topic", 10);
//     timer_ = this->create_wall_timer(
//     500ms, std::bind(&MinimalPublisher::timer_callback, this));

// private:

//     void callback()
//     {
//         auto variable = camera::image_raw
//     }


// int main(){

//     return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

class ImageModifierNode : public rclcpp::Node {
public:
  ImageModifierNode() : Node("image_modifier_node") {
    // Subscriber to the input image topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&ImageModifierNode::imageCallback, this, std::placeholders::_1));

    // Publisher for the modified image
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_modified", 10);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert ROS image message to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Get the image dimensions
    int height = cv_ptr->image.rows;
    int width = cv_ptr->image.cols;

    // Draw a circle at the center of the image
    cv::Point center(width / 2, height / 2);
    int radius = std::min(width, height) / 4;  // Circle with a radius a quarter of the smallest dimension
    cv::circle(cv_ptr->image, center, radius, CV_RGB(255, 0, 0), 3);  // Red circle with thickness 3

    // Convert modified image back to ROS image message and publish
    publisher_->publish(*cv_ptr->toImageMsg());
  }

  // Subscriber and Publisher
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  // Create and spin the node
  rclcpp::spin(std::make_shared<ImageModifierNode>());
  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
