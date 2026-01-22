#include <ros/ros.h>
#include <aim_hw/MyData.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "custom_pub");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<aim_hw::MyData>("/mydata_topic", 10);
  ros::Rate rate(5);

  int k = 0;
  while (ros::ok())
  {
    aim_hw::MyData msg;
    msg.a = 1.0f * k;
    msg.b = 0.001 * (double)k;

    msg.arr.clear();
    msg.arr.push_back(0.1f * k);
    msg.arr.push_back(0.2f * k);
    msg.arr.push_back(0.3f * k);

    pub.publish(msg);
    ROS_INFO("[PUB] a=%.2f b=%.6f arr_size=%lu", msg.a, msg.b, msg.arr.size());

    k++;
    rate.sleep();
  }
  return 0;
}

