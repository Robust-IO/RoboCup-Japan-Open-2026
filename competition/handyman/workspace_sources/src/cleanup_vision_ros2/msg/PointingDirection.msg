std_msgs/Header header
# 3D pointing in odom frame
geometry_msgs/Point origin
geometry_msgs/Vector3 direction
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
