# One atomic command produced by one RPP control cycle.
#
# velocity_north_mps / velocity_east_mps:
#   local tangent-plane velocity components in m/s.
#
# yaw_enu_rad:
#   ROS ENU yaw.
#   0 = East, +pi/2 = North, +/-pi = West, -pi/2 = South.
#
# yaw_rate_enu_radps:
#   signed ROS ENU yaw-rate [rad/s].
#   positive = counter-clockwise in ENU.
#
# yaw_valid:
#   true  = BOTH yaw_enu_rad and yaw_rate_enu_radps are authoritative.
#   false = no yaw/yaw-rate authority; downstream must fail closed to a
#           no-rotation safety stop.

std_msgs/Header header
float64 velocity_north_mps
float64 velocity_east_mps
float64 yaw_enu_rad
float64 yaw_rate_enu_radps
bool yaw_valid
