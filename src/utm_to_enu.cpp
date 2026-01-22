// egoutm_to_wgs_to_enu_node.cpp
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "geo_utils.hpp"

class UtmToEnuNode {
public:
  UtmToEnuNode(ros::NodeHandle& nh) {
    // 토픽 이름
    wgs_topic_     = "/wgs84";                 // origin 잡기용
    ego_utm_topic_ = "/ego_utm";               // 입력: x=E, y=N, z=alt
    wgs_out_topic_ = "/wgs_from_ego";          // 출력 WGS
    enu_out_topic_ = "/enu_from_ego_via_wgs";  // 출력 ENU

    utm_zone_   = 52;
    utm_northp_ = true;

    sub_wgs_ = nh.subscribe(wgs_topic_, 10, &UtmToEnuNode::wgsCb, this);
    sub_ego_ = nh.subscribe(ego_utm_topic_, 10, &UtmToEnuNode::egoCb, this);

    pub_wgs_ = nh.advertise<sensor_msgs::NavSatFix>(wgs_out_topic_, 10);
    pub_enu_ = nh.advertise<geometry_msgs::Vector3Stamped>(enu_out_topic_, 10);
    std::cout<<"utm: "<<ego_utm_topic_<<" enu: "<<enu_out_topic_<<std::endl;
  }

private:
  // origin : 첫 WGS를 저장
  void wgsCb(const sensor_msgs::NavSatFixConstPtr& msg) {
    if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) return;
    if (origin_tf) return;

    origin_ = WGS{msg->latitude, msg->longitude, msg->altitude};
    origin_tf = true;
  }

  void egoCb(const geometry_msgs::PointStampedConstPtr& msg) {
    if (!origin_tf) return; 
    UTM ego{msg->point.x, msg->point.y, utm_zone_, utm_northp_, msg->point.z};

    // Ego UTM -> WGS
    WGS ego_wgs = GeoUtils::UtmToWgs84(ego);

    sensor_msgs::NavSatFix wgs_msg;
    wgs_msg.header = msg->header;
    wgs_msg.latitude  = ego_wgs.lat_deg;
    wgs_msg.longitude = ego_wgs.lon_deg;
    wgs_msg.altitude  = ego_wgs.alt_m;
    pub_wgs_.publish(wgs_msg);

    // Ego WGS -> ENU
    Vec enu = GeoUtils::Wgs84ToEnu(ego_wgs, origin_);

    geometry_msgs::Vector3Stamped enu_msg;
    enu_msg.header = msg->header;
    enu_msg.vector.x = enu.x;
    enu_msg.vector.y = enu.y;
    enu_msg.vector.z = enu.z;
    pub_enu_.publish(enu_msg);
  }

  // ROS 통신 객체들
  ros::Subscriber sub_wgs_, sub_ego_;
  ros::Publisher  pub_wgs_, pub_enu_;

  // 토픽 이름
  std::string wgs_topic_, ego_utm_topic_, wgs_out_topic_, enu_out_topic_;

  // UTM 설정(고정)
  int  utm_zone_{52};
  bool utm_northp_{true};

  // ENU origin
  bool origin_tf{false};
  WGS origin_{};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "utm_to_enu_node");
  ros::NodeHandle nh;
  UtmToEnuNode node(nh);
  ros::spin();
  return 0;
}
