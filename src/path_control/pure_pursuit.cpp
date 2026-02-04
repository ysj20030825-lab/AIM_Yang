#include "global.hpp"
using namespace std;

struct Pose2D {
    double x;
    double y;
    double yaw;  
};
Pose2D cur1;

// 파일을 예쁘게 관리해보기 
// 함수를 예쁘게 작성하기 

// 1. Stanley 제어기 구현
// 2. 코드 리팩토링 
// 3. PP 거동 특성과 연관지어서, Stanley 제어기 장단점 잘 생각해보기
//      ㄴ 4. 3.번과 연결해서, 그럼 두 방식의 장점을 어떻게 결합할 수 있을지 생각하기 
// 4. PID 제어기 적용해서, Cmd Type 1번 사용으로 전환하기
    // ㄴ 게인 항 뭐 쓸지는 본인 논리에 의거하여 결정, 코드 작성, 및 튜닝 진행 
    

class PurePursuit {
public:
    PurePursuit(ros::NodeHandle& nh) : L{0.5} { // wheelbase = 0.5m
        sub_ego_ = nh.subscribe("/utm_from_wgs", 10, &PurePursuit::utmCb, this);
        sub_ = nh.subscribe("/tf", 10, &PurePursuit::tfCb, this);
        string csv_path;
        nh.param<string>("waypoints_csv", csv_path, "/home/aim/aim_ws/waypoints_raw.csv");
        loadCSV(csv_path);
    }

    const vector<pair<double,double>>& getPath(){
        return path;
    }

    bool loadCSV(const string& file_path) {
        ifstream fin(file_path);
        string line;
        double x,y;
        while (getline(fin, line)) {
            stringstream ss(line); 
            string sx, sy;

            if (!getline(ss, sx, ',')) continue;
            if (!getline(ss, sy, ',')) continue;

            x = stod(sx);
            y = stod(sy);

            path.push_back({x, y});
        }
        return !path.empty();
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

    double steer(const Pose2D& cur,const vector<pair<double,double>>& path,double lookahead_m)
    {
        if (path.size() < 2) return 0.0;
        int closest_i = 0;
        int i = 0;
        double dx, dy, d2;
        double index = numeric_limits<double>::infinity(); //FLT_MAX

        int start = last_closest_i - window_pts;
        int end   = last_closest_i + window_pts;
        
        // int end = std::max(path.size() -1, last_closest_i + window_pts);

        if (start < 0) start = 0;
        if (end >= (int)path.size()) end = (int)path.size() - 1;

        for (i = start; i <= end; i++) { //개선사항: 이전 루프에서 초기화된 closet_i를 알고 있다면?
            dx = path[i].first  - cur.x;
            dy = path[i].second - cur.y;
            d2 = std::sqrt(dx*dx + dy*dy);
            if (d2 < index) {
                index = d2;
                closest_i = i;
            }
        }
        last_closest_i = closest_i;
        
        // 2) lookahead point 찾기 (누적거리 >= lookahead_m)
        double lx = path.back().first;
        double ly = path.back().second;
        double acc = 0.0;
        double x1,y1,x2,y2;
        double seg;
        for (i = closest_i; i < path.size() - 1; i++) { 
            x1 = path[i].first,y1 = path[i].second;
            x2 = path[i+1].first,y2 = path[i+1].second;

            seg = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
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

        double Ld = sqrt(dx1*dx1 + dy1*dy1);
        if (Ld < 1e-6) return 0.0;

        double target_angle = atan2(dy1, dx1);
        double alpha = target_angle - cur.yaw;

        // delta = atan2(2L sin(alpha), Ld)
        return atan2(2.0 * L * sin(alpha), Ld);
    }

private:
    double L; // wheelbase;
    ros::Subscriber sub_ego_;
    ros::Subscriber sub_;
    vector<pair<double,double>> path;
    int last_closest_i = 0; 
    int window_pts = 30;   
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    ros::NodeHandle nh;
    PurePursuit node(nh);
    double delta;
    ros::Rate rate(20);
    ros::Publisher pub_ctrl = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0", 10);
    float ld = 3.0;
    while (ros::ok()) {
        ros::spinOnce();                 // 콜백 처리 (cur1 갱신)
        delta = node.steer(cur1, node.getPath(), ld);  // 조향 계산
        morai_msgs::CtrlCmd cmd;
        cmd.longlCmdType = 2;   // velocity control
        if( delta <0.07 && delta > -0.07 ){
            cmd.velocity = 22.0;
        }
        else {
            cmd.velocity = 13.0;
        }
        ld=cmd.velocity *0.07;
        cout<<"ld: " << ld << endl;
        cmd.steering = delta;
        cmd.accel = 0.0;
        cmd.brake = 0.0;
        cmd.acceleration = 0.0;
        pub_ctrl.publish(cmd);
        cout << delta << endl;
        rate.sleep();
    }
    return 0;
}