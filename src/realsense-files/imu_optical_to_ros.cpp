#include "ros/ros.h"
#include <sensor_msgs/Imu.h>

class RosImuTransformer{
  public:
    RosImuTransformer()
    {
      imu_optical_sub = nh.subscribe("/camera/imu", 1, &RosImuTransformer::pub_ros_imu,this);
      imu_ros_pub = nh.advertise<sensor_msgs::Imu>("/camera/imu_ros", 1);
      imu_msg.header.frame_id = "camera_imu_ros_frame";
    };
  void pub_ros_imu(const sensor_msgs::Imu::ConstPtr &imu_optical_msg)
  {
      imu_msg.header.stamp = imu_optical_msg->header.stamp;
      imu_msg.angular_velocity.x = imu_optical_msg->angular_velocity.z;
      imu_msg.angular_velocity.y = -imu_optical_msg->angular_velocity.x;
      imu_msg.angular_velocity.z = -imu_optical_msg->angular_velocity.y;
      imu_msg.linear_acceleration.x = -imu_optical_msg->linear_acceleration.z;
      imu_msg.linear_acceleration.y = imu_optical_msg->linear_acceleration.x;
      imu_msg.linear_acceleration.z = -imu_optical_msg->linear_acceleration.y;
      imu_ros_pub.publish(imu_msg);
  }
  private:
    ros::NodeHandle nh;
    sensor_msgs::Imu imu_msg;
    ros::Subscriber imu_optical_sub;
    ros::Publisher imu_ros_pub;
};


int main(int argc, char **argv){
  ros::init(argc,argv, "imu_to_ros_frame");
  RosImuTransformer RIP = RosImuTransformer();
  ros::spin();
  return 0;
};
