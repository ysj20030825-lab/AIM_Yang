#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include "geo_utils.hpp"

class Wgs84ToUtmNode {
public:
  Wgs84ToUtmNode(ros::NodeHandle& nh) {
    wgs_topic_ = "/wgs84";
    utm_topic_ = "/utm_from_wgs";

    sub_ = nh.subscribe(wgs_topic_, 10, &Wgs84ToUtmNode::wgsCb, this);
    pub_ = nh.advertise<geometry_msgs::PointStamped>(utm_topic_, 10);
    std::cout<<"wgs: "<<wgs_topic_<<" utm: "<<utm_topic_<<std::endl;
  }

private:
  void wgsCb(const sensor_msgs::NavSatFixConstPtr& msg) {
    if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) return;

    WGS wgs{msg->latitude, msg->longitude, msg->altitude};
    UTM utm = GeoUtils::Wgs84ToUtm(wgs);

    geometry_msgs::PointStamped out;
    out.header = msg->header;
    out.point.x = utm.E;
    out.point.y = utm.N;
    out.point.z = utm.alt_m;
    pub_.publish(out);
  }
  
  ros::Subscriber sub_;
  ros::Publisher  pub_;
  std::string wgs_topic_, utm_topic_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "wgs84_to_utm_node");
  ros::NodeHandle nh;
  Wgs84ToUtmNode node(nh);
  ros::spin();
  return 0;
}
