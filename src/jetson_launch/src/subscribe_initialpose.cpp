#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using std::placeholders::_1;

class InitialPose : public rclcpp::Node
{
  public:
    InitialPose() : Node("pub_rtab_ipose")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/rtabmap/initialpose", 1);
      subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1, std::bind(&InitialPose::pub_rtab_ipose, this, _1));
    }

  private:
    void pub_rtab_ipose(const geometry_msgs::msg::PoseWithCovarianceStamped &ipose_msg)
    {
      rtab_ipose_msg.header.frame_id = ipose_msg.header.frame_id;
      rtab_ipose_msg.header.stamp = ipose_msg.header.stamp;
      rtab_ipose_msg.pose.pose.position.x = ipose_msg.pose.pose.position.x;
      rtab_ipose_msg.pose.pose.position.y = ipose_msg.pose.pose.position.y;
      rtab_ipose_msg.pose.pose.position.z = ipose_msg.pose.pose.position.z;
      rtab_ipose_msg.pose.pose.orientation.x = ipose_msg.pose.pose.orientation.x;
      rtab_ipose_msg.pose.pose.orientation.y = ipose_msg.pose.pose.orientation.y;
      rtab_ipose_msg.pose.pose.orientation.z = ipose_msg.pose.pose.orientation.z;
      rtab_ipose_msg.pose.pose.orientation.w = ipose_msg.pose.pose.orientation.w;
      rtab_ipose_msg.pose.covariance = ipose_msg.pose.covariance;
      publisher_->publish(rtab_ipose_msg);
    }
    geometry_msgs::msg::PoseWithCovarianceStamped rtab_ipose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialPose>());
    rclcpp::shutdown();
    return 0;
  }
