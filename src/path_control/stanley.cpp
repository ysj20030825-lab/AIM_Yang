#include "global.hpp"
using namespace std;

const float L{1.5};
float k{10};
int last_closest_i = 0;
int window_pts = 30;
int max_steer=40;
struct Waypoint {
  float x;
  float y;
  float yaw;
};
vector<Waypoint> path;
Waypoint curr_pos;

void enuCb(const geometry_msgs::Vector3StampedConstPtr& msg) {
  curr_pos.x = msg->vector.x;
  curr_pos.y = msg->vector.y;
}

void imuCb(const sensor_msgs::ImuConstPtr& msg) {
    tf2::Quaternion q_imu(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );

    if (q_imu.length2() < 1e-8) return;

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_imu).getRPY(roll, pitch, yaw);

    curr_pos.yaw = yaw;
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
            size_t j = (i + 1 < xs.size()) ? (i + 1) : i;       
            size_t k = (i + 1 < xs.size()) ? i : (i - 1);       
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
    double cross = (next_wp.x - curr_wp.x) * (curr_wp.y - curr_pos.y) - (curr_wp.x - curr_pos.x) * (next_wp.y - curr_wp.y);
    double segment_len_m = sqrt(pow(next_wp.x - curr_wp.x, 2) + pow(next_wp.y - curr_wp.y, 2));
    float v = max(curr_spd, 1.0f);
    double track_error = atan2(k * (cross / segment_len_m), v);
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
    steering_angle = std::clamp(steering_angle, (float)(-max_steer * M_PI / 180.0), (float)(max_steer * M_PI / 180.0));
    return steering_angle;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "stanley");
    ros::NodeHandle nh;
    ros::Subscriber sub_ego_ = nh.subscribe("/enu_from_wgs", 10, enuCb);
    ros::Subscriber sub_imu = nh.subscribe("/imu", 10, imuCb);
    ros::Rate rate(20);
    ros::Publisher pub_ctrl = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0", 10);
    string csv_path;
    nh.param<string>("waypoints_csv", csv_path, "/home/aim/aim_ws/src/AIM_Yang/paths/path.csv");
    path=loadCSV(csv_path);
    float heading_error, track_error;
    int close_index=0;
    float delta_rad;
    float v=22.0;
    Waypoint curr_front_pos;
    while (ros::ok()) {
        ros::spinOnce();
        morai_msgs::CtrlCmd cmd;
        cmd.longlCmdType = 2;         
        const Waypoint& curr_wp = path[close_index];
        cout<<"current_position : "<<curr_pos.x<<", "<<curr_pos.y<<", "<<curr_pos.yaw<<endl;
        cout<<"closet_index : "<<curr_wp.x<<", "<<curr_wp.y<<", "<<curr_wp.yaw<<endl;
        cout<<"heading : "<<(curr_pos.yaw/M_PI*180)<<endl;
        curr_front_pos = {curr_pos.x + L * cos(curr_pos.yaw), curr_pos.y + L * sin(curr_pos.yaw), curr_pos.yaw};
        close_index = getCloseIndex(curr_front_pos,path);
        track_error = getTrackError(curr_front_pos, v, close_index,path);
        heading_error = getHeadingError(curr_front_pos, path[close_index]);
        delta_rad=getSteeringAngle(track_error, heading_error, max_steer);
        cmd.longlCmdType = 2;  
        cmd.steering = delta_rad;
        cmd.accel = 0.0;
        cmd.brake = 0.0;
        cmd.acceleration = 0.0;
        int N = 45;  
        int idx2 = std::min(close_index + N, (int)path.size()-1);
        float yaw_future = path[idx2].yaw;
        float dyaw = std::abs(wrapPi(yaw_future - path[close_index].yaw));
        if( dyaw < 0.03 ){
            cmd.velocity = 30.0;
            v=cmd.velocity;
        }
        else if(dyaw < 0.18){
            cmd.velocity = 22.0;
            v=cmd.velocity;
        }
        else {
            cmd.velocity = 17.0;
            v=cmd.velocity;
        }
        pub_ctrl.publish(cmd);
        cout << delta_rad << endl;
        std::cout << "cte_term(rad)=" << track_error
          << " heading(rad)=" << heading_error
          << " delta(rad)=" << delta_rad
          << " idx=" << close_index << "\n";
        rate.sleep();         
    }
    return 0;
}