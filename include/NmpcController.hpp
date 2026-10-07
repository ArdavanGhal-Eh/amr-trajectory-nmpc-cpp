#pragma once
#include "Model.hpp"
#include "DynamicObstacle.hpp"
#include <vector>
#include <Eigen/Dense>

struct Obstacle {
    double x{0.0};
    double y{0.0};
    double radius{0.5};
};

class NmpcController {
public:
    int horizon{10};          // Prediction steps N
    double dt{0.1};           // Sample time (seconds)
    double v_max{1.5};        // Max linear velocity (m/s)
    double v_min{0.0};
    double omega_max{1.2};    // Max angular velocity (rad/s)
    
    // Weights
    double w_pos{10.0};
    double w_theta{5.0};
    double w_u_v{0.1};
    double w_u_omega{0.2};
    double w_obs{50.0};

    std::vector<Obstacle> obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;

    NmpcController(int n = 10, double time_step = 0.1) : horizon(n), dt(time_step) {}

    ControlInput computeOptimalControl(
        const RobotState& current,
        const std::vector<RobotState>& reference_horizon
    );

    double computeTrajectoryCost(
        const RobotState& initial,
        const std::vector<ControlInput>& inputs,
        const std::vector<RobotState>& ref
    );
};
