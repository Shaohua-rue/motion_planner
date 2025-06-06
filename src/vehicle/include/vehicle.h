#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <climits>
#include <numeric>
#include <string>
#include <vector>
#include "matplotlibcpp.h"
#include <fmt/core.h>
namespace plt = matplotlibcpp;

using std::string;
using namespace Eigen;


enum class Gear { GEAR_DRIVE, GEAR_REVERSE };

namespace vehicle {
template <typename T>
double pi_2_pi(T theta) {
    while (theta > M_PI) {
        theta -= 2.0 * M_PI;
    }
    while (theta < -M_PI) {
        theta += 2.0 * M_PI;
    }

    return theta;
}
class VehicleConfig {
public:
    double RF;  // [m] distance from rear to vehicle front end of vehicle
    double RB;  // [m] distance from rear to vehicle back end of vehicle
    double W;   // [m] width of vehicle
    double WD;  // [m] distance between left-right wheels
    double WB;  // [m] Wheel base
    double TR;  // [m] Tyre radius
    double TW;  // [m] Tyre width
    double MAX_STEER;
    double MAX_SPEED = 55.0 / 3.6;
    double MIN_SPEED = -20.0 / 3.6;
    // Trailer
    double RTR;  // [m] rear to trailer wheel
    double RTF;  // [m] distance from rear to vehicle front end of trailer
    double RTB;  // [m] distance from rear to vehicle back end of trailer

    VehicleConfig(double scale = 1.0)
        : RF(3.3),
          RB(0.8),
          W(2.4),
          WB(2.5),
          TR(0.44),
          TW(0.7),
          MAX_STEER(0.6),
          RTR(8.0),
          RTF(1.0),
          RTB(9.0) {
        WD = 0.7 * W;

        RF *= scale;
        RB *= scale;
        W *= scale;
        WD *= scale;
        WB *= scale;
        TR *= scale;
        TW *= scale;
    }
    VehicleConfig(double rf, double rb, double w = 2.4, double wb = 2.5, double tr = 0.44,
                  double tw = 0.7, double max_steer = 0.6, double rtr = 8.0, double rtf = 1.0,
                  double rtb = 9.0)
        : RF(rf),
          RB(rb),
          W(w),
          WB(wb),
          TR(tr),
          TW(tw),
          MAX_STEER(max_steer),
          RTR(rtr),
          RTF(rtf),
          RTB(rtb) {
        WD = 0.7 * W;
    }

    VehicleConfig(const VehicleConfig& other) {
        RF = other.RF;
        RB = other.RB;
        W = other.W;
        WD = other.WD;
        WB = other.WB;
        TR = other.TR;
        TW = other.TW;
        MAX_STEER = other.MAX_STEER;
        MAX_SPEED = other.MAX_SPEED;
        MIN_SPEED = other.MIN_SPEED;
        RTR = other.RTR;
        RTF = other.RTF;
        RTB = other.RTB;
    }

    VehicleConfig& operator=(const VehicleConfig& other) {
        if (this != &other) {
            RF = other.RF;
            RB = other.RB;
            W = other.W;
            WD = other.WD;
            WB = other.WB;
            TR = other.TR;
            TW = other.TW;
            MAX_STEER = other.MAX_STEER;
            MAX_SPEED = other.MAX_SPEED;
            MIN_SPEED = other.MIN_SPEED;
            RTR = other.RTR;
            RTF = other.RTF;
            RTB = other.RTB;
        }
        return *this;
    }

    ~VehicleConfig() {}
};

class VehicleState {
public:
    double x;
    double y;
    double yaw;
    double v;
    Gear gear;
    VehicleConfig vc;

    VehicleState() = delete;

    VehicleState(VehicleConfig _vc, double _x = 0., double _y = 0., double _yaw = 0.,
                 double _v = 0.)
        : vc(_vc), x(_x), y(_y), yaw(_yaw), v(_v), gear(Gear::GEAR_DRIVE) {}

    VehicleState(const VehicleState& other) {
        x = other.x;
        y = other.y;
        yaw = other.yaw;
        v = other.v;
        gear = other.gear;
        vc = other.vc;
    }
    VehicleState& operator=(const VehicleState& other) {
        if (this != &other) {
            x = other.x;
            y = other.y;
            yaw = other.yaw;
            v = other.v;
            gear = other.gear;
            vc = other.vc;
        }
        return *this;
    }

    ~VehicleState() {}

    void update(double acc, double delta, double dt = 0.1);
    double calc_distance(double point_x, double point_y);
};

void draw_arrow(double x, double y, double theta, double L, std::string color);

void draw_vehicle(Eigen::Vector3d state, double steer, VehicleConfig c, std::string color = "-k",
                  bool show_wheel = true, bool show_arrow = true);

void draw_trailer(Eigen::Vector4d state, double steer, VehicleConfig c, std::string color = "-k",
                  bool show_wheel = true, bool show_arrow = true);

}