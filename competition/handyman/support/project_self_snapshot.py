"""Read-only image-time TF snapshot and diagnostic SVG projection. No masking."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np

def transform(t):
    q=t.rotation;x,y,z,w=q.x,q.y,q.z,q.w
    n=x*x+y*y+z*z+w*w
    if not np.isfinite(n) or abs(n-1)>.01:raise ValueError('Invalid TF quaternion')
    x,y,z,w=np.array([x,y,z,w])/np.sqrt(n)
    m=np.eye(4);m[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    m[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
    return m

def main():
    p=argparse.ArgumentParser();p.add_argument('--geometry',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();data=a.geometry.read_bytes();geometry=json.loads(data)
    if geometry['schema']!='handyman-self-geometry-v1':raise ValueError('Unexpected geometry')
    links=sorted({x['link'] for x in geometry['parts']})
    sys.path.insert(0,'/__HANDYMAN_PACKAGE__/project/training/scripts')
    from capture_rgb_readonly import encode_png
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo
    from tf2_ros import Buffer,TransformListener
    rclpy.init();node=Node('self_geometry_readonly_snapshot');buf=Buffer(cache_time=Duration(seconds=10));listener=TransformListener(buf,node)
    images=[];infos=[];depths=[];depth_infos=[]
    def depth_receive(msg):
        depths.append(msg)
        if len(depths)>12:depths.pop(0)
    def receive(msg):
        images.append(msg)
        if len(images)>12:images.pop(0)
    subscriptions=[node.create_subscription(Image,'/hsrb/head_rgbd_sensor/rgb/image_raw',receive,qos_profile_sensor_data),
        node.create_subscription(CameraInfo,'/hsrb/head_rgbd_sensor/rgb/camera_info',lambda m:infos.append(m) if not infos else infos.__setitem__(0,m),qos_profile_sensor_data),
        node.create_subscription(Image,'/hsrb/head_rgbd_sensor/depth_registered/image_raw',depth_receive,qos_profile_sensor_data),
        node.create_subscription(CameraInfo,'/hsrb/head_rgbd_sensor/depth_registered/camera_info',lambda m:depth_infos.append(m) if not depth_infos else depth_infos.__setitem__(0,m),qos_profile_sensor_data)]
    deadline=time.monotonic()+20;snapshot=None;error='No new RGB/CameraInfo'
    print('Read-only capture: waiting up to 20s for fresh RGB and exact-time TF',flush=True)
    try:
        while time.monotonic()<deadline and snapshot is None:
            rclpy.spin_once(node,timeout_sec=.03)
            if not infos or not depths or not depth_infos:continue
            ci=infos[0]
            for im in list(images):
                stamp=Time.from_msg(im.header.stamp);age=(node.get_clock().now()-stamp).nanoseconds/1e9
                if not 0<=age<=.5:continue
                if im.header.frame_id!=ci.header.frame_id or (im.width,im.height)!=(ci.width,ci.height):continue
                try:
                    dep=min(depths,key=lambda d:abs(Time.from_msg(d.header.stamp).nanoseconds-stamp.nanoseconds))
                    dns=Time.from_msg(dep.header.stamp).nanoseconds
                    if abs(dns-stamp.nanoseconds)>30_000_000:continue
                    di=depth_infos[0]
                    if (dep.width,dep.height)!=(im.width,im.height) or list(di.k)!=list(ci.k) or any(di.d) or dep.header.frame_id!=di.header.frame_id:continue
                    depth_to_rgb=transform(buf.lookup_transform(im.header.frame_id,dep.header.frame_id,stamp).transform)
                    if not np.allclose(depth_to_rgb,np.eye(4),atol=1e-5):raise ValueError('Depth/RGB extrinsics are not identity; registration required')
                    ts={link:buf.lookup_transform(im.header.frame_id,link,stamp) for link in links}
                    # Bound relative self motion during the RGB/depth time difference.
                    depth_ts={link:buf.lookup_transform(im.header.frame_id,link,Time.from_msg(dep.header.stamp)) for link in links}
                    if any(not np.allclose(transform(ts[l].transform),transform(depth_ts[l].transform),atol=1e-4) for l in links):continue
                    snapshot=(im,ci,ts,dep);break
                except Exception as exc:error=str(exc)
    finally:
        node.destroy_node();rclpy.shutdown()
    if snapshot is None:raise RuntimeError('No snapshot captured: '+error)
    im,ci,ts,dep=snapshot
    k=np.array(ci.k).reshape(3,3)
    if not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0 or any(ci.d):raise ValueError('Unsupported intrinsics')
    # HSRPubTf.UpdateTransformForLocal maps Unity local position (x,y,z) to (-x,y,z).
    # Apply the same basis conversion to mesh points in their link-local coordinates.
    mirror=np.diag([-1.,1.,1.,1.]);paths=[];records=[]
    for part in geometry['parts']:
        vs=np.array([[v['x'],v['y'],v['z'],1.] for v in part['vertices']])
        local=np.array(part['meshToLinkUnityRowMajor']).reshape(4,4)
        camera=transform(ts[part['link']].transform)@mirror@local
        pts=(camera@vs.T).T[:,:3]
        if not np.isfinite(pts).all():raise ValueError('Nonfinite geometry')
        tris=np.array(part['triangles'],dtype=int).reshape(-1,3)
        kept=0
        for tri in tris:
            v=pts[tri]
            if np.any(v[:,2]<=.01):continue # diagnostic only: near-plane crossing not rasterized
            uv=(k@v.T).T;uv=uv[:,:2]/uv[:,2,None]
            if np.any(uv.max(axis=0)<[0,0]) or np.any(uv.min(axis=0)>[im.width,im.height]):continue
            paths.append('M'+' L'.join(f'{x:.2f},{y:.2f}' for x,y in uv)+' Z');kept+=1
        records.append(dict(link=part['link'],renderer=part['rendererPath'],visible_candidate_triangles=kept))
    a.output.mkdir(exist_ok=False)
    png=encode_png(im);(a.output/'rgb.png').write_bytes(png)
    if dep.encoding not in ('16UC1','32FC1'):raise ValueError('Unsupported depth encoding')
    dtype=np.dtype(('>' if dep.is_bigendian else '<')+('u2' if dep.encoding=='16UC1' else 'f4'))
    if dep.step<dep.width*dtype.itemsize or len(dep.data)!=dep.step*dep.height:raise ValueError('Malformed depth buffer')
    depth=np.ndarray((dep.height,dep.width),dtype=dtype,buffer=bytes(dep.data),strides=(dep.step,dtype.itemsize)).astype(float)
    if dep.encoding=='16UC1':depth*=.001
    np.save(a.output/'observed_depth_m.npy',depth)
    report=dict(geometry_sha256=hashlib.sha256(data).hexdigest(),image_stamp_ns=Time.from_msg(im.header.stamp).nanoseconds,
        frame=im.header.frame_id,k=list(ci.k),parts=records,
        depth_stamp_ns=Time.from_msg(dep.header.stamp).nanoseconds,depth_encoding=dep.encoding,
        depth_registration='equal K, image shape and identity TF; relative self motion checked at both stamps',
        camera_from_link={link:transform(t.transform).tolist() for link,t in ts.items()},
        approved_for_runtime=False,limitations=['Projection convention is a hypothesis to verify visually.',
        'No z-buffer or scene depth: projected geometry is NOT certified visible self pixels.',
        'No near-plane triangle clipping. No runtime exclusion.'])
    (a.output/'snapshot.json').write_text(json.dumps(report,indent=2))
    image='data:image/png;base64,'+base64.b64encode(png).decode()
    page=f'''<!doctype html><meta charset="utf-8"><title>动态 TF 网格投影诊断</title>
<style>body{{font:18px sans-serif;margin:24px}}svg{{width:100%;max-width:1100px}}</style>
<h1>图像时间戳对应的 TF 网格投影（未上线）</h1>
<p>青色为几何投影假设，不是有效过滤遮罩。未做深度遮挡判断，不删除任何检测。</p>
<button onclick="let e=document.getElementById('mesh');e.style.display=e.style.display==='none'?'':'none'">切换网格</button>
<svg viewBox="0 0 {im.width} {im.height}"><image href="{image}" width="{im.width}" height="{im.height}"/>
<path id="mesh" d="{' '.join(paths)}" fill="none" stroke="#00dfff" stroke-width="0.35" opacity="0.65"/></svg>'''
    (a.output/'index.html').write_text(page,encoding='utf-8')
    print(json.dumps(dict(output=str(a.output),parts=records)),flush=True)

if __name__=='__main__':main()
