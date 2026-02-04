// egoutm_to_wgs_to_enu_node.cpp
#include <ros/ros.h>
#include <morai_msgs/GPSMessage.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "geo_utils.hpp"

class UtmToEnuNode {
public:
  UtmToEnuNode(ros::NodeHandle& nh) {
    // 토픽 이름
    wgs_topic_     = "/gps";                 // origin 잡기용
    ego_utm_topic_ = "/Ego_topic";               // 입력: x=E, y=N, z=alt
    wgs_out_topic_ = "/wgs_from_ego";          // 출력 WGS
    enu_out_topic_ = "/enu_from_ego_via_wgs";  // 출력 ENU

    utm_zone_   = 52;
    utm_northp_ = true;

    sub_wgs_ = nh.subscribe(wgs_topic_, 10, &UtmToEnuNode::wgsCb, this);
    sub_ego_ = nh.subscribe(ego_utm_topic_, 10, &UtmToEnuNode::egoCb, this);

    pub_wgs_ = nh.advertise<morai_msgs::GPSMessage>(wgs_out_topic_, 10);
    pub_enu_ = nh.advertise<geometry_msgs::Vector3Stamped>(enu_out_topic_, 10);
    std::cout<<"utm: "<<ego_utm_topic_<<" enu: "<<enu_out_topic_<<std::endl;
  }

private:
  // origin : 첫 WGS를 저장
  void wgsCb(const morai_msgs::GPSMessageConstPtr& msg) {
    WGS wgs{msg->latitude, msg->longitude, msg->altitude};
    if (!origin_tf) {
      origin_ = wgs;
      origin_tf = true;
     //std::cout<<origin_.lat_deg<<", "<<origin_.lon_deg<<", "<<origin_.alt_m<<std::endl;
    }
  }

  void egoCb(const morai_msgs::EgoVehicleStatusConstPtr& msg) {
    if (!has_ego_origin_) {
      ego_origin_x_ = msg->position.x;
      ego_origin_y_ = msg->position.y;
      ego_origin_z_ = msg->position.z;
      has_ego_origin_ = true;
    }
    double dx = msg->position.x - ego_origin_x_;
    double dy = msg->position.y - ego_origin_y_;
    double dz = msg->position.z - ego_origin_z_;
    UTM origin_utm_abs_ = GeoUtils::Wgs84ToUtm(origin_);
    UTM ego_abs{};
    ego_abs.zone   = origin_utm_abs_.zone;
    ego_abs.northp = origin_utm_abs_.northp;
    ego_abs.E      = origin_utm_abs_.E + dx;
    ego_abs.N      = origin_utm_abs_.N + dy;
    ego_abs.alt_m  = origin_utm_abs_.alt_m + dz;  // z는 사실 비교 제외 추천

    // Ego UTM -> WGS
    WGS ego_wgs = GeoUtils::UtmToWgs84(ego_abs);
    morai_msgs::GPSMessage wgs_msg;
    wgs_msg.header = msg->header;
    wgs_msg.latitude  = ego_wgs.lat_deg;
    wgs_msg.longitude = ego_wgs.lon_deg;
    wgs_msg.altitude  = ego_wgs.alt_m;
    pub_wgs_.publish(wgs_msg);
    //std::cout<<wgs_msg.latitude<<", "<<wgs_msg.longitude<<", "<<wgs_msg.altitude<<std::endl;

    // Ego WGS -> ENU
    Vec enu = GeoUtils::Wgs84ToEnu(ego_wgs, origin_);
    geometry_msgs::Vector3Stamped enu_msg;
    enu_msg.header = msg->header;
    enu_msg.vector.x = enu.x;
    enu_msg.vector.y = enu.y;
    enu_msg.vector.z = enu.z;
    pub_enu_.publish(enu_msg);
    std::cout<<enu_msg.vector.x<<", "<<enu_msg.vector.y<<", "<<enu_msg.vector.z<<std::endl;
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
  bool has_ego_origin_{false};
  double ego_origin_x_{0.0}, ego_origin_y_{0.0}, ego_origin_z_{0.0};
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
