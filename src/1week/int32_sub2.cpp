#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <config.h>

void cb(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("[SUB2] %d", msg->data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "int32_sub2");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/int32_topic_x2", AIM_HW_QSIZE, cb);
  ros::spin();
  return 0;
}

