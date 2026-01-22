#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "geo_utils.hpp"  

class EnuToNed1 {
public:
  EnuToNed1(ros::NodeHandle& nh) {
    odom_topic_ = "/odom";
    rpy_ned_topic_ = "/rpy_ned";

    sub_ = nh.subscribe(odom_topic_, 10, &EnuToNed1::odomCb, this);
    pub_rpy_ned_ = nh.advertise<geometry_msgs::Vector3Stamped>(rpy_ned_topic_, 10);

    std::cout<<"ned: "<<rpy_ned_topic_<<std::endl;
  }

private:
  void odomCb(const nav_msgs::OdometryConstPtr& msg) {
    const auto& q = msg->pose.pose.orientation;
    tf2::Quaternion q_tf(q.x, q.y, q.z, q.w);

    double roll_enu, pitch_enu, yaw_enu;
    tf2::Matrix3x3(q_tf).getRPY(roll_enu, pitch_enu, yaw_enu);

    RPY enu{roll_enu, pitch_enu, yaw_enu};
    RPY ned = GeoUtils::EnuToNed(enu);

    geometry_msgs::Vector3Stamped out;
    out.header = msg->header;
    out.vector.x = ned.roll;
    out.vector.y = ned.pitch;
    out.vector.z = ned.yaw;
    pub_rpy_ned_.publish(out);

    RPY enu2 = GeoUtils::NedToEnu(ned);
    const double dr = enu2.roll  - enu.roll;
    const double dp = enu2.pitch - enu.pitch;
    const double dy = enu2.yaw   - enu.yaw;

    ROS_INFO_THROTTLE(1.0,
      " enu(%.3f,%.3f,%.3f) ned(%.3f,%.3f,%.3f) roundtrip_err(%.6f,%.6f,%.6f)",
      enu.roll, enu.pitch, enu.yaw,
      ned.roll, ned.pitch, ned.yaw,
      dr, dp, dy
    );
  }

  ros::Subscriber sub_;
  ros::Publisher  pub_rpy_ned_;

  std::string odom_topic_;
  std::string rpy_ned_topic_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "enu_to_ned_node");
  ros::NodeHandle nh;
  EnuToNed1 node(nh);
  ros::spin();
  return 0;
}
