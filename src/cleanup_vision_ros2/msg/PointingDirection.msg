std_msgs/Header header
# 3D pointing in odom frame
geometry_msgs/Point origin
geometry_msgs/Vector3 direction
# A 2-D arm line defines a plane through the camera, not a spatial ray.
# Captured in odom with the image so it remains valid after head movement.
geometry_msgs/Point plane_origin
geometry_msgs/Vector3 plane_normal
# Half-space normals retain image forward direction and camera visibility.
geometry_msgs/Vector3 plane_forward
geometry_msgs/Vector3 camera_forward
bool plane_valid
# True only when direction was fitted from several synchronized RGB-D samples
# along the visible arm.  When false, direction is a pose/image fallback and
# must not be treated as measured depth geometry by downstream selectors.
bool depth_direction_valid
float32 depth_direction_fit_error
uint8 depth_direction_samples
# MediaPipe pose-world arm direction transformed to odom.  It is retained as
# a fallback when synchronized RGB-D samples cannot produce a valid fit.
geometry_msgs/Vector3 pose_direction
bool pose_direction_valid
# 2D pointing in image space (pixels)
float32 wrist_pixel_x
float32 wrist_pixel_y
float32 point_pixel_x
float32 point_pixel_y
# Avatar torso center in image space. This remains available before a
# pointing arm is recognized so the robot head can keep the Avatar in view.
float32 person_pixel_x
float32 person_pixel_y
bool person_is_valid
float32 confidence
bool is_valid

# Historical full-arm endpoint/pose estimate, only after arm-extension checks.
# Independent of hand-only crops and of the measured multi-point depth fit.
# Both vectors use the capture-time odom transform. Never label this measured depth.
bool arm_ray_valid
geometry_msgs/Point arm_ray_origin
geometry_msgs/Vector3 arm_ray_direction

# Baseline used for geometry: arm (elbow to index), hand (finger only), unknown.
# confidence is an observation weight, NOT a calibrated angular probability.
string geometry_source

# CLOCK_MONOTONIC at local RGB callback entry, before conversion/inference.
# Same-host phase ordering only; NOT sensor capture time or a TF timestamp.
int64 received_steady_ns
