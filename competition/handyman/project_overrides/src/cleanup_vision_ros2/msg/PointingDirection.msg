std_msgs/Header header
# 3D pointing in odom frame
geometry_msgs/Point origin
geometry_msgs/Vector3 direction
# 2D pointing in image space (pixels)
float32 wrist_pixel_x
float32 wrist_pixel_y
float32 point_pixel_x
float32 point_pixel_y
float32 confidence
bool is_valid
