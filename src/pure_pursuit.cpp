#include <cmath>
#include <vector>
#include <utility>
#include <limits>
#include <iostream>

struct Pose2D {
    double x;
    double y;
    double yaw;   // rad
};

class PurePursuit {
public:
    PurePursuit(double wheelbase) : L(wheelbase) {}
    
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
            double x1 = path[i].first,   y1 = path[i].second;
            double x2 = path[i+1].first, y2 = path[i+1].second; // OUT OF INDEX ERROR 

            double seg = std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
            acc += seg;

            if (acc >= lookahead_m) {
                lx = x2;
                ly = y2;
                break;
            }
        }
        // 3) Pure Pursuit 조향각 계산
        double dx = lx - cur.x;
        double dy = ly - cur.y;

        double Ld = std::sqrt(dx*dx + dy*dy);
        if (Ld < 1e-6) return 0.0;

        double target_angle = std::atan2(dy, dx);
        double alpha = target_angle - cur.yaw;

        // delta = atan2(2L sin(alpha), Ld)
        return std::atan2(2.0 * L * std::sin(alpha), Ld);
    }

private:
    double L; // wheelbase
};

int main() {
    std::vector<std::pair<double,double>> path = {
        {0,0}, {0,1}, {0,2}, {0,3}, {1,4}, {2,5}
    };

    Pose2D cur{0.2, 0.3, 1.57}; 
    PurePursuit pp(0.5);        // wheelbase = 0.5m

    double delta = pp.steer(cur, path, 1.5); // lookahead = 1.5m
    std::cout << "steer(rad) = " << delta << std::endl;
}