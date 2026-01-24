#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "geo_utils.hpp"  

class EnuToNed1 {
public:
  EnuToNed1(ros::NodeHandle& nh):L{0.5}{
    tf_topic_ = "/tf";
    rpy_ned_topic_ = "/rpy_ned";

    sub_ = nh.subscribe(tf_topic_, 10, &EnuToNed1::tfCb, this);
    pub_rpy_ned_ = nh.advertise<geometry_msgs::Vector3Stamped>(rpy_ned_topic_, 10);

    std::cout<<"ned: "<<rpy_ned_topic_<<std::endl;
  }

private:
  void tfCb(const tf2_msgs::TFMessageConstPtr& msg) {
    tf2::Quaternion q_tf;     
    for (const auto& t : msg->transforms) {
      if (t.header.frame_id == "/map" && t.child_frame_id == "/base_link") {
        const auto& q = t.transform.rotation;
        q_tf = tf2::Quaternion(q.x, q.y, q.z, q.w);
        break;
    }
  }

    double roll_enu, pitch_enu, yaw_enu;
    tf2::Matrix3x3(q_tf).getRPY(roll_enu, pitch_enu, yaw_enu);

    RPY enu{roll_enu, pitch_enu, yaw_enu};
    RPY ned = GeoUtils::EnuToNed(enu);

    geometry_msgs::Vector3Stamped out;
    out.vector.x = ned.roll;
    out.vector.y = ned.pitch;
    out.vector.z = ned.yaw;
    pub_rpy_ned_.publish(out);

    RPY enu2 = GeoUtils::NedToEnu(ned);
    const double dr = enu2.roll  - enu.roll;
    const double dp = enu2.pitch - enu.pitch;
    const double dy = enu2.yaw   - enu.yaw;

    std::cout<<"ENU: "<<enu.roll<<", "<<enu.pitch<<", "<<enu.yaw<<std::endl<<"NED: "<<ned.roll
    <<", "<<ned.pitch<<", "<<ned.yaw<<std::endl<<"dR: "<<dr<<", dP: "<<dp<<", dY: "<<dy<<std::endl;
  }
  double L;
  ros::Subscriber sub_;
  ros::Publisher  pub_rpy_ned_;

  std::string tf_topic_;
  std::string rpy_ned_topic_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "enu_to_ned_node");
  ros::NodeHandle nh;
  EnuToNed1 node(nh);
  ros::spin();
  return 0;
}
