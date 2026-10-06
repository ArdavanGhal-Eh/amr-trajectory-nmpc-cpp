#pragma once
#include <cmath>
#include <vector>

struct DynamicObstacle {
    double x{0.0};
    double y{0.0};
    double vx{0.0};  // Velocity X (m/s)
    double vy{0.0};  // Velocity Y (m/s)
    double radius{0.4};

    // Propagate obstacle position k steps ahead into the prediction horizon
    void predictPosition(double dt, int k, double& pred_x, double& pred_y) const {
        pred_x = x + vx * (k * dt);
        pred_y = y + vy * (k * dt);
    }

    // Evaluates dynamic safety barrier margin
    double computeBarrierPenalty(double robot_x, double robot_y, double dt, int k, double safety_buffer = 0.35) const {
        double px, py;
        predictPosition(dt, k, px, py);
        double dist_sq = (robot_x - px) * (robot_x - px) + (robot_y - py) * (robot_y - py);
        double safe_dist = radius + safety_buffer;

        if (dist_sq < safe_dist * safe_dist) {
            double penetration = safe_dist - std::sqrt(std::max(1e-4, dist_sq));
            return penetration * penetration * 150.0;
        }
        return 0.0;
    }
};
