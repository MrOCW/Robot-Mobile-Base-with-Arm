#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class RosImuTransformer : public rclcpp::Node
{
  public:
    RosImuTransformer()
    : Node("imu_to_ros_frame")
    {
      imu_msg.header.frame_id = "camera_imu_ros_frame";
      publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/camera/imu_ros", 1);
      subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/camera/imu", 1, std::bind(&RosImuTransformer::imu_callback, this, _1));
    }

  private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_optical_msg)
    {
      imu_msg.header.stamp = imu_optical_msg->header.stamp;
      imu_msg.angular_velocity.x = imu_optical_msg->angular_velocity.z;
      imu_msg.angular_velocity.y = -imu_optical_msg->angular_velocity.x;
      imu_msg.angular_velocity.z = -imu_optical_msg->angular_velocity.y;
      imu_msg.linear_acceleration.x = -imu_optical_msg->linear_acceleration.z;
      imu_msg.linear_acceleration.y = imu_optical_msg->linear_acceleration.x;
      imu_msg.linear_acceleration.z = -imu_optical_msg->linear_acceleration.y;
      publisher_->publish(imu_msg);
    }
    sensor_msgs::msg::Imu imu_msg = sensor_msgs::msg::Imu();
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosImuTransformer>());
    rclcpp::shutdown();
    return 0;
  }