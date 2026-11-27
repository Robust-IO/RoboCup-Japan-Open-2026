"""Read-only conservative circular base sweep; never authorizes arm/base motion."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer,TransformListener
from hand_camera_ray import rotation


def tf_parts(msg):
    t=msg.transform.translation;q=msg.transform.rotation
    return np.array([t.x,t.y,t.z]),rotation([q.x,q.y,q.z,q.w])


def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--scale',type=float,default=1.);p.add_argument('--translation',type=float,nargs=2);a=p.parse_args()
    if a.translation is not None and (a.scale!=1. or not np.isfinite(a.translation).all() or not 0<a.translation[0]<=.15 or abs(a.translation[1])>.05):raise ValueError('Invalid explicit translation')
    if not 0<a.scale<=1:raise ValueError('Scale must be in (0,1]')
    if a.output.exists():raise ValueError('Output exists')
    rclpy.init();n=rclpy.create_node('preapproach_clearance_readonly');buf=Buffer();listener=TransformListener(buf,n);data={}
    subs=[n.create_subscription(LaserScan,'/hsrb/base_scan',lambda m:data.update(scan=m),qos_profile_sensor_data),
          n.create_subscription(OccupancyGrid,'/map',lambda m:data.update(map=m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))]
    report=dict(motion_authorized=False,translation_m=[.143635067*a.scale,-.046090252*a.scale],radius_m=.38,margin_m=.03)
    if a.translation is not None:report['translation_m']=a.translation
    try:
        deadline=time.monotonic()+12;last='missing map/scan'
        while time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.03)
            if len(data)!=2:continue
            scan=data['scan'];stamp=Time.from_msg(scan.header.stamp)
            if not -.02<=(n.get_clock().now()-stamp).nanoseconds/1e9<=.4:continue
            try:
                sb=buf.lookup_transform('base_footprint',scan.header.frame_id,stamp)
                bm=buf.lookup_transform(data['map'].header.frame_id,'base_footprint',stamp)
                break
            except Exception as e:last=str(e)
        else:raise RuntimeError(last)
        values=np.asarray(scan.ranges);angles=scan.angle_min+np.arange(len(values))*scan.angle_increment
        valid=np.isfinite(values)&(values>=scan.range_min)&(values<=scan.range_max)
        if valid.sum()<20:raise ValueError('Insufficient laser returns')
        pts=np.stack([values[valid]*np.cos(angles[valid]),values[valid]*np.sin(angles[valid]),np.zeros(valid.sum())],1)
        t,r=tf_parts(sb);pts=(pts@r.T+t)[:,:2]
        delta=np.array(report['translation_m']);centers=np.linspace(0,1,51)[:,None]*delta
        distances=np.linalg.norm(pts[None,:,:]-centers[:,None,:],axis=2)
        report['laser']=dict(valid_returns=int(valid.sum()),start_nearest_m=float(distances[0].min()),
            swept_nearest_m=float(distances.min()),flagged_samples=int(np.count_nonzero(distances.min(1)<.41)))
        grid=data['map'];g=np.array(grid.data).reshape(grid.info.height,grid.info.width);res=grid.info.resolution
        origin=grid.info.origin;oq=origin.orientation
        ro=rotation([oq.x,oq.y,oq.z,oq.w]);ot=np.array([origin.position.x,origin.position.y,origin.position.z])
        t,r=tf_parts(bm);mc=np.c_[centers,np.zeros(len(centers))]@r.T+t
        gc=(mc-ot)@ro
        rows=[]
        for x,y,_ in gc:
            cells=int(np.ceil(.41/res))+1;cx=int(np.floor(x/res));cy=int(np.floor(y/res))
            yy,xx=np.mgrid[cy-cells:cy+cells+1,cx-cells:cx+cells+1]
            # Include cells intersecting the circle using half-cell diagonal.
            mask=np.hypot((xx+.5)*res-x,(yy+.5)*res-y)<=.41+res/np.sqrt(2)
            inside=(xx>=0)&(yy>=0)&(xx<grid.info.width)&(yy<grid.info.height)
            vals=g[yy[mask&inside],xx[mask&inside]]
            rows.append(dict(occupied=int(np.count_nonzero(vals>=65)),unknown=int(np.count_nonzero(vals<0)),out_of_map=int(np.count_nonzero(mask&~inside))))
        report.update(map_samples=rows,base_map_xyz=t.tolist(),scan_stamp_ns=stamp.nanoseconds,
          limitation='Static map plus single planar scan; conservative .41m circle, not certified physical footprint. Does not cover elevated arm, blind regions or future pedestrian motion.')
    except Exception as e:report['error']=str(e)
    finally:
        with a.output.open('x') as f:json.dump(report,f,indent=2)
        print(json.dumps({k:v for k,v in report.items() if k!='map_samples'}))
        if 'map_samples' in report:print('map flagged samples',sum(any(v for v in row.values()) for row in report['map_samples']))
        n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
