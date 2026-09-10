# One atomic command produced by one RPP control cycle.
#
# velocity_north_mps / velocity_east_mps:
#   local tangent-plane velocity components in m/s.
#
# yaw_enu_rad:
#   ROS ENU yaw.
#   0 = East, +pi/2 = North, +/-pi = West, -pi/2 = South.
#
# yaw_valid:
#   true  = yaw_enu_rad is authoritative for normal B-side control.
#   false = no yaw authority; downstream must fail closed to a no-rotation
#           safety stop rather than silently reverting to vector steering.

std_msgs/Header header
float64 velocity_north_mps
float64 velocity_east_mps
float64 yaw_enu_rad
bool yaw_valid
