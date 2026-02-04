#include "global.hpp"
using namespace std;

const float pi{3.141592};
const float L{0.5};
float k{5};
int last_closest_i = 0;
int window_pts = 30;
int max_steer=30;
struct Waypoint {
  float x;
  float y;
  float yaw;
};
vector<Waypoint> path;
Waypoint curr_pos;

void utmCb(const geometry_msgs::PointStampedConstPtr& msg) {
        curr_pos.x = msg->point.x;
        curr_pos.y = msg->point.y;  
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
    curr_pos.yaw = yaw_enu;
}

vector<Waypoint> loadCSV(const string& file_path) {
    path.clear();
    ifstream fin(file_path);
    vector<float> xs, ys;
    string line;

    while (getline(fin, line)) {
        if (line.empty()) continue;
        if (!line.empty() && (line[0] < '0' || line[0] > '9') && line[0] != '-' && line[0] != '+')
        continue;
        stringstream ss(line);
        string sx, sy, s3;
        if (!getline(ss, sx, ',')) continue;
        if (!getline(ss, sy, ',')) continue;
        xs.push_back(stod(sx));
        ys.push_back(stod(sy));
    }
    path.reserve(xs.size());
    for (size_t i = 0; i < xs.size(); ++i) {
        float yaw = 0.0;
        if (xs.size() >= 2) {
            size_t j = (i + 1 < xs.size()) ? (i + 1) : i;       // 마지막 점은 자기 자신
            size_t k = (i + 1 < xs.size()) ? i : (i - 1);       // 마지막 점은 이전 점으로 방향
            yaw = std::atan2(ys[j] - ys[k], xs[j] - xs[k]);
        }
    path.push_back({xs[i], ys[i], yaw});
    }
    return path;
}

double getDist(Waypoint point1, Waypoint point2){
    double dist = sqrt(pow(point2.x - point1.x,2) + pow(point2.y - point1.y,2));
    return dist;
}

int getCloseIndex(const Waypoint& curr_pos,const vector<Waypoint>& path){
    int closest_i = 0;
    int i = 0;
    double dx, dy, d2;
    double index = FLT_MAX;

    int start = last_closest_i - window_pts;
    int end   = last_closest_i + window_pts;
        
    // int end = std::max(path.size() -1, last_closest_i + window_pts);

    if (start < 0) start = 0;
    if (end >= (int)path.size()) end = (int)path.size() - 1;

    for (i = start; i <= end; i++) { 
        dx = path[i].x  - curr_pos.x;
        dy = path[i].y - curr_pos.y;
        d2 = std::sqrt(dx*dx + dy*dy);
        if (d2 < index) {
            index = d2;
            closest_i = i;
        }
    }
    last_closest_i = closest_i;
    return last_closest_i;
}

float getTrackError(const Waypoint& curr_pos, float curr_spd, int close_index,const vector<Waypoint>& path){
    if (close_index==path.size()-1){
        close_index-=1;
    }
    const Waypoint& curr_wp = path[close_index], next_wp = path[close_index+1];
    double e_numerator = (next_wp.x - curr_wp.x) * (curr_wp.y - curr_pos.y) - (curr_wp.x - curr_pos.x) * (next_wp.y - curr_wp.y);
    double e_denominator = sqrt(pow(next_wp.x - curr_wp.x, 2) + pow(next_wp.y - curr_wp.y, 2));
    double track_error = atan2(k * (e_numerator / e_denominator), curr_spd);
    return track_error;    
}

float wrapPi(float a){
    while(a >  M_PI) a -= 2*M_PI;
    while(a < -M_PI) a += 2*M_PI;
    return a;
}

float getHeadingError(Waypoint curr_pos, Waypoint curr_wp){
    float heading_error = wrapPi(curr_wp.yaw - curr_pos.yaw);
    return heading_error;
}

float getSteeringAngle(float track_error, float heading_error, int max_steer){
    float steering_angle = track_error + heading_error;
    if (steering_angle > (max_steer*pi/180)) steering_angle = max_steer*pi/180;
    else if (steering_angle < -(max_steer*pi/180)) steering_angle = -max_steer*pi/180;
    return steering_angle;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "stanley");
    ros::NodeHandle nh;
    ros::Subscriber sub_ego_ = nh.subscribe("/utm_from_wgs", 10, utmCb);
    ros::Subscriber sub_ = nh.subscribe("/tf", 10, tfCb);
    ros::Rate rate(20);
    ros::Publisher pub_ctrl = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0", 10);
    string csv_path;
    nh.param<string>("waypoints_csv", csv_path, "/home/aim/aim_ws/waypoints_raw.csv");
    path=loadCSV(csv_path);
    float heading_error, track_error;
    int close_index;
    float delta;
    float v=22.0;
    Waypoint curr_front_pos;
    while (ros::ok()) {
        ros::spinOnce();
        morai_msgs::CtrlCmd cmd;
        cmd.longlCmdType = 2;         
        cout<<"current_position : "<<curr_pos.x<<", "<<curr_pos.y<<", "<<curr_pos.yaw<<endl;
        cout<<"heading : "<<(curr_pos.yaw/pi*180)<<endl;
        curr_front_pos = {curr_pos.x + L * cos(curr_pos.yaw), curr_pos.y + L * sin(curr_pos.yaw), curr_pos.yaw};
        close_index = getCloseIndex(curr_front_pos,path);
        track_error = getTrackError(curr_front_pos, v, close_index,path);
        heading_error = getHeadingError(curr_front_pos, path[close_index]);
        delta=getSteeringAngle(track_error, heading_error, max_steer);
        cmd.longlCmdType = 2;  
        cmd.steering = delta;
        cmd.accel = 0.0;
        cmd.brake = 0.0;
        cmd.acceleration = 0.0;
        cmd.velocity = 22.0;
        pub_ctrl.publish(cmd);
        cout << delta << endl;
        std::cout << "cte_term(rad)=" << track_error
          << " heading(rad)=" << heading_error
          << " delta(rad)=" << delta
          << " idx=" << close_index << "\n";
        rate.sleep();         
    }
    return 0;
}