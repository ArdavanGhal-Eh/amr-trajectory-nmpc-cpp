#include <iostream>
#include <cassert>
#include <cmath>
#include "Model.hpp"
#include "DynamicObstacle.hpp"
#include "NmpcController.hpp"

void test_kinematics_rk4() {
    RobotState init = {0.0, 0.0, 0.0};
    ControlInput u = {1.0, 0.0}; // 1 m/s straight
    double dt = 0.1;

    RobotState s = init;
    for (int i = 0; i < 10; ++i) {
        s = RobotKinematics::stepRK4(s, u, dt);
    }

    assert(std::abs(s.x - 1.0) < 1e-3);
    assert(std::abs(s.y) < 1e-4);
    assert(std::abs(s.theta) < 1e-4);
    std::cout << "✅ [PASS] test_kinematics_rk4 | Straight line RK4 position: " << s.x << " m\n";
}

void test_empty_reference_safety() {
    NmpcController nmpc(10, 0.1);
    RobotState current = {0.0, 0.0, 0.0};
    std::vector<RobotState> empty_ref;

    ControlInput u = nmpc.computeOptimalControl(current, empty_ref);
    assert(u.v == 0.0);
    assert(u.omega == 0.0);
    std::cout << "✅ [PASS] test_empty_reference_safety | Safe zero velocity on empty ref.\n";
}

void test_dynamic_obstacle_penalty() {
    NmpcController nmpc(10, 0.1);
    DynamicObstacle dyn_obs;
    dyn_obs.x = 2.0;
    dyn_obs.y = 0.0;
    dyn_obs.vx = -0.5; // Moving towards robot
    dyn_obs.vy = 0.0;
    dyn_obs.radius = 0.4;
    nmpc.dynamic_obstacles.push_back(dyn_obs);

    RobotState init = {0.0, 0.0, 0.0};
    std::vector<ControlInput> straight_inputs(10, {1.0, 0.0});
    std::vector<RobotState> ref_horizon(10, {2.0, 0.0, 0.0});

    double cost_with_obs = nmpc.computeTrajectoryCost(init, straight_inputs, ref_horizon);
    
    // Clear dynamic obstacles and compare cost
    nmpc.dynamic_obstacles.clear();
    double cost_without_obs = nmpc.computeTrajectoryCost(init, straight_inputs, ref_horizon);

    assert(cost_with_obs > cost_without_obs);
    std::cout << "✅ [PASS] test_dynamic_obstacle_penalty | Penalty detected: " 
              << (cost_with_obs - cost_without_obs) << "\n";
}

void test_optimal_control_actuator_limits() {
    NmpcController nmpc(10, 0.1);
    RobotState current = {0.0, 0.0, 0.0};
    std::vector<RobotState> ref_horizon;
    for (int k = 1; k <= 10; ++k) {
        ref_horizon.push_back({k * 0.5, 0.2, 0.0});
    }

    ControlInput u = nmpc.computeOptimalControl(current, ref_horizon);
    assert(u.v >= nmpc.v_min - 1e-4 && u.v <= nmpc.v_max + 1e-4);
    assert(u.omega >= -nmpc.omega_max - 1e-4 && u.omega <= nmpc.omega_max + 1e-4);
    std::cout << "✅ [PASS] test_optimal_control_actuator_limits | v=" << u.v << ", omega=" << u.omega << "\n";
}

int main() {
    std::cout << "=== Running AMR Trajectory NMPC Unit Tests ===\n";
    test_kinematics_rk4();
    test_empty_reference_safety();
    test_dynamic_obstacle_penalty();
    test_optimal_control_actuator_limits();
    std::cout << "All AMR NMPC tests passed successfully!\n";
    return 0;
}
