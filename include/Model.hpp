#pragma once
#include <Eigen/Dense>
#include <cmath>

struct RobotState {
    double x{0.0};      // X position (meters)
    double y{0.0};      // Y position (meters)
    double theta{0.0};  // Heading angle (radians)
    double v{0.0};      // Linear velocity (m/s)
    double omega{0.0};  // Angular velocity (rad/s)
};

struct ControlInput {
    double v{0.0};      // Commanded linear velocity (m/s)
    double omega{0.0};  // Commanded angular velocity (rad/s)
};

class RobotKinematics {
public:
    static RobotState stepRK4(const RobotState& current, const ControlInput& u, double dt) {
        auto dynamics = [](const RobotState& s, const ControlInput& ctrl) -> Eigen::Vector3d {
            return Eigen::Vector3d(ctrl.v * std::cos(s.theta),
                                   ctrl.v * std::sin(s.theta),
                                   ctrl.omega);
        };

        Eigen::Vector3d k1 = dynamics(current, u);

        RobotState s2 = current;
        s2.x += 0.5 * dt * k1(0);
        s2.y += 0.5 * dt * k1(1);
        s2.theta += 0.5 * dt * k1(2);
        Eigen::Vector3d k2 = dynamics(s2, u);

        RobotState s3 = current;
        s3.x += 0.5 * dt * k2(0);
        s3.y += 0.5 * dt * k2(1);
        s3.theta += 0.5 * dt * k2(2);
        Eigen::Vector3d k3 = dynamics(s3, u);

        RobotState s4 = current;
        s4.x += dt * k3(0);
        s4.y += dt * k3(1);
        s4.theta += dt * k3(2);
        Eigen::Vector3d k4 = dynamics(s4, u);

        RobotState next = current;
        next.x += (dt / 6.0) * (k1(0) + 2.0 * k2(0) + 2.0 * k3(0) + k4(0));
        next.y += (dt / 6.0) * (k1(1) + 2.0 * k2(1) + 2.0 * k3(1) + k4(1));
        next.theta += (dt / 6.0) * (k1(2) + 2.0 * k2(2) + 2.0 * k3(2) + k4(2));
        
        // Normalize theta to [-pi, pi]
        while (next.theta > M_PI) next.theta -= 2.0 * M_PI;
        while (next.theta < -M_PI) next.theta += 2.0 * M_PI;

        next.v = u.v;
        next.omega = u.omega;
        return next;
    }
};
