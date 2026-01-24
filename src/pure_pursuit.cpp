#include <cmath>
#include <vector>
#include <utility>
#include <limits>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_msgs/TFMessage.h>

struct Pose2D {
    double x;
    double y;
    double yaw;  
};
Pose2D cur1;

class PurePursuit {
public:
    PurePursuit(ros::NodeHandle& nh) : L{0.5} { // wheelbase = 0.5m
        sub_ego_ = nh.subscribe("/utm_from_wgs", 10, &PurePursuit::utmCb, this);
        sub_ = nh.subscribe("/tf", 10, &PurePursuit::tfCb, this);
    }

    void utmCb(const geometry_msgs::PointStampedConstPtr& msg) {
        cur1.x = msg->point.x;
        cur1.y = msg->point.y;
    }

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
        cur1.yaw = yaw_enu;
    }

    double steer(const Pose2D& cur,const std::vector<std::pair<double,double>>& path,double lookahead_m)
    {
        int closest_i = 0;
        int i = 0;
        double dx, dy, d2;
        double index = std::numeric_limits<double>::infinity();
        for (i = 0; i < path.size(); i++) { /// 개선사항. 만약 이전 루프에서 초기화된 index를 알고있다면 ?
            dx = path[i].first  - cur.x;
            dy = path[i].second - cur.y;
            d2 = sqrt(dx*dx + dy*dy);
            if (d2 < index) {
                index = d2;
                closest_i = i;
            }
        }
        // 2) lookahead point 찾기 (누적거리 >= lookahead_m)
        double lx = path.back().first;
        double ly = path.back().second;
        double acc = 0.0;
        for (i = closest_i; i < path.size() - 1; i++) {
            double x1 = path[i].first,y1 = path[i].second;
            double x2 = path[i+1].first,y2 = path[i+1].second; // OUT OF INDEX ERROR 

            double seg = std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
            acc += seg;

            if (acc >= lookahead_m) {
                lx = x2;
                ly = y2;
                break;
            }
        }
        // 3) Pure Pursuit 조향각 계산
        double dx1 = lx - cur.x;
        double dy1 = ly - cur.y;

        double Ld = std::sqrt(dx1*dx1 + dy1*dy1);
        if (Ld < 1e-6) return 0.0;

        double target_angle = std::atan2(dy1, dx1);
        double alpha = target_angle - cur.yaw;

        // delta = atan2(2L sin(alpha), Ld)
        return std::atan2(2.0 * L * std::sin(alpha), Ld);
    }

private:
    double L; // wheelbase;
    ros::Subscriber sub_ego_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wgs84_to_utm_node");
    ros::NodeHandle nh;
    PurePursuit node(nh);
    double delta;
    ros::Rate rate(20);
    while (ros::ok()) {
        ros::spinOnce();                 // 콜백 처리 (cur1 갱신)
        delta = node.steer(cur1, path, 1.5);  // 조향 계산
        std::cout << delta << std::endl; // 출력 or CtrlCmd publish
        rate.sleep();
    }
    return 0;
}