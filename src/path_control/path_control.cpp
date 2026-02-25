#include "global.hpp"
using namespace std;

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
const float L{1.5};
float k{20};

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

Waypoint getLookahead(int close_i,float ld){
    float lx=path[close_i].x;
    float ly=path[close_i].y;
    float x1,y1;
    float seg;
    Waypoint ld_point;
    int i=0;
    for (i = close_i; i < path.size() - 1; i++) { 
            x1 = path[i].x,y1 = path[i].y;
            seg = sqrt((x1-lx)*(x1-lx) + (y1-ly)*(y1-ly));
            if (seg >= ld) {
                ld_point=path[i];
                break;
            }
    }
    return ld_point;
}

float getPPAngle(Waypoint ld_point,const Waypoint& curr_pos){
    float dx=ld_point.x-curr_pos.x;
    float dy=ld_point.y-curr_pos.y;
    float Ld=sqrt(dx*dx+dy*dy);
    if(Ld <1e-6) return 0.0;
    float target_angle=atan2(dy,dx);
    float alpha=target_angle-curr_pos.yaw;
    return atan2(2.0*L*sin(alpha),Ld);
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

float getStanleyAngle(float track_error, float heading_error, int max_steer){
    float steering_angle = track_error + heading_error;
    steering_angle = std::clamp(steering_angle, (float)(-max_steer * M_PI / 180.0), (float)(max_steer * M_PI / 180.0));
    return steering_angle;
}

float sigmoid(float z) { return 1.0f / (1.0f + std::exp(-z)); }

int main(int argc, char** argv){
    ros::init(argc, argv, "path_control");
    ros::NodeHandle nh;
    ros::Subscriber sub_ego_ = nh.subscribe("/enu_from_wgs", 10, enuCb);
    ros::Subscriber sub_imu = nh.subscribe("/imu", 10, imuCb);
    ros::Rate rate(20);
    ros::Publisher pub_ctrl = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0", 10);

    string csv_path;
    nh.param<string>("waypoints_csv", csv_path, "/home/aim/aim_ws/src/AIM_Yang/paths/path.csv");
    path=loadCSV(csv_path);

    
    
    float heading_error, track_error,stanley_angle_rad,pp_angle_rad,stanley_ratio;
    Waypoint lookahead;
    int close_index=0;
    float v=22.0;
    float ld=3.0;
    Waypoint curr_front_pos;

    while (ros::ok()) {
        ros::spinOnce();
        morai_msgs::CtrlCmd cmd;
        cmd.longlCmdType = 2;

        curr_front_pos = {curr_pos.x + L * cos(curr_pos.yaw), curr_pos.y + L * sin(curr_pos.yaw), curr_pos.yaw};
        close_index = getCloseIndex(curr_front_pos,path);
        const Waypoint& curr_wp = path[close_index];
        track_error = getTrackError(curr_front_pos, v, close_index,path);
        heading_error = getHeadingError(curr_front_pos, path[close_index]);
        lookahead=getLookahead(close_index,ld);
        pp_angle_rad=getPPAngle(lookahead,curr_pos);
        stanley_angle_rad=getStanleyAngle(track_error, heading_error, max_steer);
        float dyaw = std::abs(wrapPi(lookahead.yaw - path[close_index].yaw));

        //cout<<"current_position : "<<curr_pos.x<<", "<<curr_pos.y<<", "<<curr_pos.yaw<<endl;
        //cout<<"closet_index : "<<curr_wp.x<<", "<<curr_wp.y<<", "<<curr_wp.yaw<<endl;
        //cout<<"heading : "<<(curr_pos.yaw/M_PI*180)<<endl;

        if( dyaw < 0.03 ){
            cmd.velocity = 30.0;
            v=cmd.velocity;
        }
        else if(dyaw < 0.07){
            cmd.velocity = 22.0;
            v=cmd.velocity;
        }
        else {
            cmd.velocity = 17.0;
            v=cmd.velocity;
        }
        cout<<"dyaw: "<<dyaw<<endl<<"stanley_ratio: "<<stanley_ratio<<endl;
        float vc = 25.0f;   // km/h, 고속의 기준
        float sv = 12.0f;   // km/h, 민감도
        float dc=0.08f; //커브의 기준
        float sd= 0.03; //민감도
        float w_curve = sigmoid((dyaw - dc) / sd);
        float w_speed = 1.0f - sigmoid((v - vc) / sv);
        stanley_ratio = std::clamp( 0.85f*w_curve + 0.15f*w_speed , 0.0f, 1.0f);
        double steer=stanley_ratio*stanley_angle_rad+(1-stanley_ratio)*pp_angle_rad;

        ld = std::clamp(v * 0.12f, 4.0f, 12.0f);
        cmd.longlCmdType = 2;  
        cmd.steering = steer;
        cmd.accel = 0.0;
        cmd.brake = 0.0;
        cmd.acceleration = 0.0;
        pub_ctrl.publish(cmd);
        //std::cout << "delta: "<<steer<<endl<<"stanley_ratio: "<<stanley_ratio<<endl<<"lookahead:"<<ld<<endl;
        rate.sleep();         
    }
    return 0;
}