#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <config.h>
#include <topics.hpp>

ros::Publisher g_pub;

void relayCallback(const std_msgs::Int32::ConstPtr& msg)
{
  std_msgs::Int32 out;
  out.data = msg->data * 2;  

  g_pub.publish(out);
  ROS_INFO("[RELAY] in=%d -> out=%d", msg->data, out.data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "int32_relay");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe(aim_hw::int32_topic(), AIM_HW_QSIZE, relayCallback);

  g_pub = nh.advertise<std_msgs::Int32>("/int32_topic_x2", AIM_HW_QSIZE);

  ros::spin();
  return 0;
}

