#include "NmpcController.hpp"
#include <cmath>
#include <algorithm>

double NmpcController::computeTrajectoryCost(
    const RobotState& initial,
    const std::vector<ControlInput>& inputs,
    const std::vector<RobotState>& ref
) {
    double cost = 0.0;
    RobotState state = initial;

    for (int k = 0; k < horizon && k < static_cast<int>(ref.size()) && k < static_cast<int>(inputs.size()); ++k) {
        state = RobotKinematics::stepRK4(state, inputs[k], dt);

        double dx = state.x - ref[k].x;
        double dy = state.y - ref[k].y;
        double dtheta = state.theta - ref[k].theta;
        while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
        while (dtheta < -M_PI) dtheta += 2.0 * M_PI;

        cost += w_pos * (dx * dx + dy * dy) + w_theta * (dtheta * dtheta);
        cost += w_u_v * (inputs[k].v * inputs[k].v) + w_u_omega * (inputs[k].omega * inputs[k].omega);

        // Static obstacle avoidance barrier cost
        for (const auto& obs : obstacles) {
            double dist_sq = (state.x - obs.x) * (state.x - obs.x) + (state.y - obs.y) * (state.y - obs.y);
            double safe_dist = obs.radius + 0.35; // 0.35m robot safety buffer
            if (dist_sq < safe_dist * safe_dist) {
                double penetration = safe_dist - std::sqrt(std::max(1e-4, dist_sq));
                cost += w_obs * (penetration * penetration) * 100.0;
            }
        }

        // Dynamic moving obstacle barrier cost
        for (const auto& dyn_obs : dynamic_obstacles) {
            cost += dyn_obs.computeBarrierPenalty(state.x, state.y, dt, k, 0.35);
        }
    }
    return cost;
}

ControlInput NmpcController::computeOptimalControl(
    const RobotState& current,
    const std::vector<RobotState>& reference_horizon
) {
    if (reference_horizon.empty()) {
        return {0.0, 0.0};
    }

    std::vector<ControlInput> u_horizon(horizon);
    
    // Warm start with heuristic feedforward towards next reference
    double dx = reference_horizon[0].x - current.x;
    double dy = reference_horizon[0].y - current.y;
    double target_heading = std::atan2(dy, dx);
    double heading_err = target_heading - current.theta;
    while (heading_err > M_PI) heading_err -= 2.0 * M_PI;
    while (heading_err < -M_PI) heading_err += 2.0 * M_PI;

    double init_v = std::clamp(std::hypot(dx, dy), v_min, v_max * 0.8);
    double init_omega = std::clamp(heading_err * 2.0, -omega_max, omega_max);

    for (int k = 0; k < horizon; ++k) {
        u_horizon[k] = {init_v, init_omega};
    }

    // Iterative Gradient-Descent / Numerical Optimization over Control Sequence
    const int max_iters = 25;
    const double alpha_v = 0.05;
    const double alpha_omega = 0.08;
    const double eps = 1e-3;

    for (int iter = 0; iter < max_iters; ++iter) {
        for (int k = 0; k < std::min(4, horizon); ++k) {
            // Numerical gradient wrt v
            std::vector<ControlInput> u_plus = u_horizon;
            u_plus[k].v += eps;
            double grad_v = (computeTrajectoryCost(current, u_plus, reference_horizon) - 
                             computeTrajectoryCost(current, u_horizon, reference_horizon)) / eps;

            // Numerical gradient wrt omega
            std::vector<ControlInput> u_plus_w = u_horizon;
            u_plus_w[k].omega += eps;
            double grad_w = (computeTrajectoryCost(current, u_plus_w, reference_horizon) - 
                             computeTrajectoryCost(current, u_horizon, reference_horizon)) / eps;

            u_horizon[k].v = std::clamp(u_horizon[k].v - alpha_v * grad_v, v_min, v_max);
            u_horizon[k].omega = std::clamp(u_horizon[k].omega - alpha_omega * grad_w, -omega_max, omega_max);
        }
    }

    return u_horizon[0]; // Receding horizon: apply first control input
}
