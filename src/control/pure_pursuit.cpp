// Pure pursuit.

#include <algorithm>
#include <cmath>
#include <fieldnav/control/pure_pursuit.hpp>

namespace fieldnav::control {
Cmd purePursuit(const odometry::Pose2D& pose, path::Path& path, const PurePursuitParams& params) {
    const path::Point2D target = path.lookahead(pose, params.lookahead);
    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    const double ld = std::hypot(dx, dy);
    if (ld < 1e-6) {
        return Cmd{0.0, 0.0};
    }
    const double alpha = odometry::normalizeAngle(std::atan2(dy, dx) - pose.theta);
    const double w = params.v_des * 2.0 * std::sin(alpha) / ld;
    return Cmd{params.v_des, std::clamp(w, -params.w_max, params.w_max)};
}

} // namespace fieldnav::control
