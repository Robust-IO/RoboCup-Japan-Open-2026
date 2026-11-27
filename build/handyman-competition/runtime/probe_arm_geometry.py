"""Read-only current HSR TF geometry; not an IK solution."""
import json
import time
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener


def main():
    rclpy.init();node=rclpy.create_node('phase5_arm_geometry_probe');buf=Buffer();listener=TransformListener(buf,node)
    end=time.monotonic()+4
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    rows={}
    for name in ['arm_lift_link','arm_flex_link','arm_roll_link','wrist_flex_link','wrist_roll_link','hand_palm_link','hand_l_proximal_link','hand_r_proximal_link','hand_camera_rgb_frame']:
        try:
            m=buf.lookup_transform('base_link',name,Time());t=m.transform.translation;q=m.transform.rotation
            rows[name]=dict(xyz=[t.x,t.y,t.z],quaternion=[q.x,q.y,q.z,q.w],age_s=(node.get_clock().now()-Time.from_msg(m.header.stamp)).nanoseconds/1e9)
        except Exception as e:rows[name]=dict(error=str(e))
    print(json.dumps(rows,indent=2));node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
