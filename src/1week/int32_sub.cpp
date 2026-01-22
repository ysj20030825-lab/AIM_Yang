#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <config.h>
#include <topics.hpp>

void int32Callback(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("[SUB] %d", msg->data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "int32_sub");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe(aim_hw::int32_topic(), AIM_HW_QSIZE, int32Callback);

  ros::spin();
  return 0;
}

