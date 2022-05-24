#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class InitialPose{
  public:
    InitialPose()
    {
      ipose_sub = nh.subscribe("/initialpose", 1, &InitialPose::pub_rtab_ipose,this);
      rtab_ipose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/rtabmap/initialpose", 1);
      
    };
  void pub_rtab_ipose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &ipose_msg)
  {
      rtab_ipose_msg.header.seq = ipose_msg->header.seq;
      rtab_ipose_msg.header.frame_id = ipose_msg->header.frame_id;
      rtab_ipose_msg.header.stamp = ipose_msg->header.stamp;
      rtab_ipose_msg.pose.pose.position.x = ipose_msg->pose.pose.position.x;
      rtab_ipose_msg.pose.pose.position.y = ipose_msg->pose.pose.position.y;
      rtab_ipose_msg.pose.pose.position.z = ipose_msg->pose.pose.position.z;
      rtab_ipose_msg.pose.pose.orientation.x = ipose_msg->pose.pose.orientation.x;
      rtab_ipose_msg.pose.pose.orientation.y = ipose_msg->pose.pose.orientation.y;
      rtab_ipose_msg.pose.pose.orientation.z = ipose_msg->pose.pose.orientation.z;
      rtab_ipose_msg.pose.pose.orientation.w = ipose_msg->pose.pose.orientation.w;
      rtab_ipose_msg.pose.covariance = ipose_msg->pose.covariance;
      rtab_ipose_pub.publish(rtab_ipose_msg);
  }
  private:
    ros::NodeHandle nh;
    geometry_msgs::PoseWithCovarianceStamped rtab_ipose_msg;
    ros::Subscriber ipose_sub;
    ros::Publisher rtab_ipose_pub;
};


int main(int argc, char **argv){
  ros::init(argc,argv, "pub_rtab_ipose");
  InitialPose ip = InitialPose();
  ros::spin();
  return 0;
};
