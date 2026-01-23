#include <ros/ros.h>
#include <morai_msgs/GPSMessage.h>  
#include <geometry_msgs/PointStamped.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include "geo_utils.hpp"

class Wgs84ToUtmNode {
public:
  Wgs84ToUtmNode(ros::NodeHandle& nh) {
    wgs_topic_ = "/gps";
    utm_topic_ = "/utm_from_wgs";
    ego_topic_ = "/Ego_topic";
    sub_ego_ = nh.subscribe(ego_topic_, 10, &Wgs84ToUtmNode::egoCb, this);
    sub_ = nh.subscribe(wgs_topic_, 10, &Wgs84ToUtmNode::wgsCb, this);
    pub_ = nh.advertise<geometry_msgs::PointStamped>(utm_topic_, 10);
    std::cout<<"wgs:"<<wgs_topic_<<" utm:"<<utm_topic_<<std::endl;
  }

private:
  void egoCb(const morai_msgs::EgoVehicleStatusConstPtr& msg) {
    if (!has_ego_origin_) {
      ego_origin_x_ = msg->position.x;
      ego_origin_y_ = msg->position.y;
      has_ego_origin_ = true;
    }
  }
  void wgsCb(const morai_msgs::GPSMessageConstPtr& msg) {
    WGS wgs{msg->latitude, msg->longitude, msg->altitude};
    if (!has_origin_) {
      origin_utm_ = GeoUtils::Wgs84ToUtm(wgs);       
      has_origin_ = true;
      std::cout<<origin_utm_.E<<", "<<origin_utm_.N<<std::endl; 
    }
    UTM utm = GeoUtils::Wgs84ToUtm(wgs);
    geometry_msgs::PointStamped out;
    out.header = msg->header;
    out.point.x = utm.E-origin_utm_.E+ego_origin_x_;
    out.point.y = utm.N-origin_utm_.N+ego_origin_y_;
    std::cout<<out.point.x<<", "<<out.point.y<<std::endl;
    pub_.publish(out);
  }
  
  ros::Subscriber sub_;
  ros::Subscriber sub_ego_;
  ros::Publisher  pub_;
  std::string wgs_topic_, utm_topic_, ego_topic_;
  bool has_origin_{false};
  UTM origin_utm_{};
  bool has_ego_origin_{false};
  double ego_origin_x_{0.0}, ego_origin_y_{0.0};

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "wgs84_to_utm_node");
  ros::NodeHandle nh;
  Wgs84ToUtmNode node(nh);
  ros::spin();
  return 0;
}
