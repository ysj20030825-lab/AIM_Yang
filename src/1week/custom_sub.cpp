#include <ros/ros.h>
#include <aim_hw/MyData.h>

void cb(const aim_hw::MyData::ConstPtr& msg)
{
  ROS_INFO("[SUB] a=%.2f b=%.6f arr_size=%lu", msg->a, msg->b, msg->arr.size());
  if (!msg->arr.empty())
    ROS_INFO("      arr[0]=%.3f", msg->arr[0]);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "custom_sub");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/mydata_topic", 10, cb);
  ros::spin();
  return 0;
}

