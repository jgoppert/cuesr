#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using std::placeholders::_1;

class Align : public rclcpp::Node
{
  public:
    Align()
    : Node("align")
    {
      sub_ir_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/thermal/camera/image_rect", 10,
        std::bind(&Align::ir_callback, this, _1));

      sub_ir_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/thermal/camera/camera_info", 10,
        std::bind(&Align::ir_info_callback, this, _1));

      sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth/image_rect_raw", 10,
        std::bind(&Align::depth_callback, this, _1));

      sub_depth_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/depth/camera_info", 10,
        std::bind(&Align::depth_info_callback, this, _1));

      pub_align_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/thermal/camera/depth_aligned", 10);
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_align_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_ir_, sub_depth_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_ir_info_, sub_depth_info_;

    void ir_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "I heard an ir image");
      (void)msg;
    }

    void ir_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "I heard an ir info image");
      (void)msg;
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "I heard a depth image");
      (void)msg;
    }

    void depth_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "I heard a depth info image");
      (void)msg;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Align>());
  rclcpp::shutdown();
  return 0;
}
