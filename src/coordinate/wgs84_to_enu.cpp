// wgs84_to_enu_node.cpp
#include <ros/ros.h>
#include <morai_msgs/GPSMessage.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "geo_utils.hpp"

class Wgs84ToEnuNode {
public:
  Wgs84ToEnuNode(ros::NodeHandle& nh) {
    // 토픽 이름 
    wgs_topic_ = "/gps";
    enu_topic_ = "/enu_from_wgs";

    sub_ = nh.subscribe(wgs_topic_, 10, &Wgs84ToEnuNode::wgsCb, this);
    pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(enu_topic_, 10);
    //std::cout<<"wgs: "<<wgs_topic_<<" enu: "<<enu_topic_<<std::endl;
  }

private:
  void wgsCb(const morai_msgs::GPSMessageConstPtr& msg) {

    WGS wgs{msg->latitude, msg->longitude, msg->altitude};

    // 첫 GPS를 origin으로 저장
    if (!origin_tf) {
      origin_ = wgs;
      origin_tf = true;
      std::cout<<origin_.lat_deg<<", "<<origin_.lon_deg<<", "<<origin_.alt_m<<std::endl;
    }
    Vec enu = GeoUtils::Wgs84ToEnu(wgs, origin_);

    geometry_msgs::Vector3Stamped out;
    out.header = msg->header;
    out.vector.x = enu.x;
    out.vector.y = enu.y;
    out.vector.z = enu.z;
    pub_.publish(out);
    std::cout<<out.vector.x<<", "<<out.vector.y<<", "<<out.vector.z<<std::endl;
  }

  ros::Subscriber sub_;
  ros::Publisher  pub_;
  std::string wgs_topic_, enu_topic_;

  bool origin_tf{false};
  WGS origin_{};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "wgs84_to_enu_node");
  ros::NodeHandle nh;
  Wgs84ToEnuNode node(nh);
  ros::spin();
  return 0;
}
