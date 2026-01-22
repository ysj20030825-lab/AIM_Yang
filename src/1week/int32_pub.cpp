#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <config.h>
#include <topics.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "int32_pub");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Int32>(aim_hw::int32_topic(), AIM_HW_QSIZE);

  ros::Rate rate(10);
  int count = 0;

  while (ros::ok())
  {
    std_msgs::Int32 msg;
    msg.data = count++;

    pub.publish(msg);
    ROS_INFO("[PUB] %d", msg.data);

    rate.sleep();
  }
  return 0;
}

