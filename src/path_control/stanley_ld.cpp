#include "global.hpp"
using namespace std;

struct Waypoint {
  float x;
  float y;
  float yaw;
};

struct PID {
    float kp, ki, kd;
    float integral;     
    float differential;
    float integral_min, integral_max;
    float output_min, output_max;

    PID(float kp1, float ki1, float kd1)
        : kp(kp1), ki(ki1), kd(kd1),
          integral(0.0f), differential(0.0f),
          integral_min(-0.5f), integral_max(0.5f),
          output_min(-1.0f), output_max(1.0f) {}

    // target(목표) - current(현재) 를 받아서 [-1,1] 출력
    float update(float target, float current, float dt) {
        if (dt < 1e-4f) dt = 1e-4f;

        // 1) 오차
        float error = target - current;

        // 2) 적분(오차 누적) + 폭주 방지
        integral += error * dt;
        integral = std::clamp(integral, integral_min, integral_max);

        // 3) 미분(오차 변화율)
        float derivative = (error - differential) / dt;
        differential = error;

        // 4) PID 합산
        float u = kp * error + ki * integral + kd * derivative;

        // 5) 출력 제한
        u = std::clamp(u, output_min, output_max);
        return u;
    }

    void reset(float current_error = 0.0f) {
        integral = 0.0f;
        differential = current_error;
    }
};

vector<Waypoint> path;
Waypoint curr_pos;
const float L{1.5};
int last_closest_i = 0;
int window_pts = 30;
int max_steer=40;
float v_meas = 0.0f;

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

void egoCb(const morai_msgs::EgoVehicleStatusConstPtr& msg){
    float vx = msg->velocity.x;
    float vy = msg->velocity.y;
    v_meas = std::sqrt(vx*vx + vy*vy);
}

float wrapPi(float a){
    while(a >  M_PI) a -= 2*M_PI;
    while(a < -M_PI) a += 2*M_PI;
    return a;
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

Waypoint getLookahead(int close_i,float ld){
    float lx=path[close_i].x;
    float ly=path[close_i].y;
    float all=0.0;
    float x1,y1,x2,y2;
    float seg;
    Waypoint ld_point = path.back(); 
    int i=0;
    for (i = close_i; i < path.size() - 1; i++) { 
            x1 = path[i].x;
            y1 = path[i].y;
            x2 = path[i+1].x;
            y2 = path[i+1].y;
            seg = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
            all +=seg;
            if (all >= ld) {
                ld_point=path[i+1];
                break;
            }
    }
    return ld_point;
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

float getTrackError_ld(const Waypoint& curr_pos, float curr_spd, int close_index,const vector<Waypoint>& path,const Waypoint& lookahead,float ld){
    if (close_index==path.size()-1){
        close_index-=1;
    }
    const Waypoint& curr_wp = path[close_index];
    float angle=atan2(lookahead.y-curr_pos.y,lookahead.x-curr_pos.x);
    float alpha = wrapPi(angle - curr_pos.yaw);
    float e_ld = ld * std::sin(alpha);  
    float v = max(curr_spd, 1.0f);
    float k = std::clamp(6.5f + 0.15f * curr_spd, 4.0f, 16.0f);
    double track_error = atan2(k * e_ld, v);
    return track_error;
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

float speedFromDyaw(float dyaw,float v_max, float v_min,float dyaw_fast, float dyaw_slow)
{
    float t = (dyaw - dyaw_fast) / (dyaw_slow - dyaw_fast);
    t = std::clamp(t, 0.0f, 1.0f);
    t = t * t * (3.0f - 2.0f * t);
    return v_max + (v_min - v_max) * t;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "stanley_ld");
    ros::NodeHandle nh;
    ros::Subscriber sub_ego_ = nh.subscribe("/enu_from_wgs", 10, enuCb);
    ros::Subscriber sub_imu = nh.subscribe("/imu", 10, imuCb);
    ros::Subscriber sub_status = nh.subscribe("/Ego_topic", 10, egoCb);
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
    Waypoint ld_wp;
    PID spd_pid(0.7f, 0.05f, 0.02f);
    ros::Time prevT = ros::Time::now();
    bool pid_inited = false;
    while (ros::ok()) {
        ros::spinOnce();
        morai_msgs::CtrlCmd cmd;
        cmd.longlCmdType = 2;         
        const Waypoint& curr_wp = path[close_index];
        
        float ld_m = std::clamp(1.0f + 0.08f*v, 1.0f, 7.0f);
        curr_front_pos = {curr_pos.x + L * cos(curr_pos.yaw), curr_pos.y + L * sin(curr_pos.yaw), curr_pos.yaw};
        close_index = getCloseIndex(curr_front_pos,path);
        ld_wp=getLookahead(close_index,ld_m);
        track_error = getTrackError_ld(curr_front_pos, v, close_index,path,ld_wp,ld_m);
        heading_error = getHeadingError(curr_front_pos, path[close_index]);
        delta_rad=getSteeringAngle(track_error, heading_error, max_steer);
        
        cmd.longlCmdType = 1;  
        cmd.steering = delta_rad;
        cmd.accel = 0.0;
        cmd.brake = 0.0;
        cmd.acceleration = 0.0;
        cmd.velocity=0.0;
        int N = 150;  
        int idx2 = std::min(close_index + N, (int)path.size()-1);
        float yaw_future = path[idx2].yaw;
        float dyaw = std::abs(wrapPi(yaw_future - path[close_index].yaw));
        float v_ref_kmh=speedFromDyaw(dyaw, 50.0f, 19.0f, 0.00f, 1.0f);
        float v_ref = v_ref_kmh / 3.6f;
        // (B) dt 계산
        ros::Time nowT = ros::Time::now();
        if (!pid_inited) {
            prevT = nowT;
            spd_pid.reset(v_ref - v_meas);
            pid_inited = true;
        }
        float dt = (nowT - prevT).toSec();
        prevT = nowT;
        
        // (C) PID 출력 u 계산 (u는 -1~+1)
        float u = spd_pid.update(v_ref, v_meas, dt);

        if (u >= 0.0f) {
            cmd.accel = u;       // 0~1
            cmd.brake = 0.0f;
        } else {
            cmd.accel = 0.0f;
            cmd.brake = -u;      // 0~1
        }

        pub_ctrl.publish(cmd);
        cout << "v_ref(kmh)=" << v_ref_kmh << " v(kmh)=" << v_meas*3.6f << " u=" << u << " accel=" << cmd.accel << " brake=" << cmd.brake << endl;
        //cout << "cte_term(rad)=" << track_error<< " heading(rad)=" << heading_error<< " delta(rad)=" << delta_rad<< " idx=" << close_index << endl;
        rate.sleep();         
    }
    return 0;
}