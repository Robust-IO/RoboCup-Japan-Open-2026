"""Offline local scan/map alignment in a user-confirmed room; no control/TF writes."""
import argparse,json,math
from pathlib import Path
import cv2
import numpy as np
from PIL import Image
from hand_camera_ray import rotation


def search(points,grid,origin,resolution,bounds,steps):
    # Endpoints should meet obstacle *surfaces*, not merely land anywhere
    # inside thick occupied cells. Otherwise pose score has broad flat minima.
    occupied=(grid<100).astype(np.uint8)
    border=occupied&(cv2.dilate((grid>240).astype(np.uint8),np.ones((3,3),np.uint8))>0)
    distance=cv2.distanceTransform((1-border).astype(np.uint8),cv2.DIST_L2,5)*resolution
    axes=[np.arange(lo,hi+step*.1,step) for (lo,hi),step in zip(bounds,steps)]
    poses=np.stack(np.meshgrid(*axes,indexing='ij'),-1).reshape(-1,3);rows=[]
    for chunk in np.array_split(poses,max(1,math.ceil(len(poses)/512))):
        c=np.cos(chunk[:,2])[:,None];s=np.sin(chunk[:,2])[:,None]
        x=chunk[:,0,None]+c*points[:,0]-s*points[:,1];y=chunk[:,1,None]+s*points[:,0]+c*points[:,1]
        u=((x-origin[0])/resolution-.5).astype(np.float32);v=(grid.shape[0]-.5-(y-origin[1])/resolution).astype(np.float32)
        d=cv2.remap(distance,u,v,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=1.)
        clipped=np.minimum(d,.35);k=int(len(points)*.7);score=np.partition(clipped,k,axis=1)[:,:k].mean(axis=1)+.1*clipped.mean(axis=1)
        cx=np.floor((chunk[:,0]-origin[0])/resolution).astype(int);cy=grid.shape[0]-1-np.floor((chunk[:,1]-origin[1])/resolution).astype(int)
        free=grid[np.clip(cy,0,grid.shape[0]-1),np.clip(cx,0,grid.shape[1]-1)]>240
        score[~free]=10
        for i in np.argsort(score)[:10]:rows.append((float(score[i]),chunk[i].tolist(),float((d[i]<.08).mean())))
    return sorted(rows,key=lambda x:x[0])


def main():
    p=argparse.ArgumentParser();p.add_argument('--scan',type=Path,required=True);p.add_argument('--map',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise ValueError('output_exists')
    scan=json.loads(a.scan.read_text());ranges=np.array([float('nan') if x is None else x for x in scan['ranges']]);angles=scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
    valid=np.isfinite(ranges)&(ranges>.15)&(ranges<9);idx=np.flatnonzero(valid)[::3];points=np.c_[ranges[idx]*np.cos(angles[idx]),ranges[idx]*np.sin(angles[idx]),np.zeros(len(idx))]
    tf=scan['base_from_scan'];points=(points@rotation(tf['quaternion']).T+tf['translation'])[:,:2]
    grid=np.array(Image.open(a.map));origin=[-1.52,-8.3420381546];res=.05
    coarse=search(points,grid,origin,res,[(8.,10.5),(-7.1,-5.1),(-.45,.45)],[.05,.05,.025]);best=coarse[0][1]
    fine=search(points,grid,origin,res,[(best[0]-.08,best[0]+.08),(best[1]-.08,best[1]+.08),(best[2]-.035,best[2]+.035)],[.005,.005,.0025])
    alternatives=[]
    for score,pose,inliers in coarse:
        if not alternatives or all(math.dist(pose[:2],x['pose'][:2])>.20 or abs(pose[2]-x['pose'][2])>.12 for x in alternatives):alternatives.append(dict(score=score,pose=pose,inlier_fraction=inliers))
        if len(alternatives)==5:break
    result=dict(best=dict(score=fine[0][0],pose=fine[0][1],inlier_fraction=fine[0][2]),alternatives=alternatives,points=len(points),scope='User-confirmed LayoutA bedroom, near last completed table',motion_authorized=False,automatic_localization_approved=False,odom_from_base=scan['odom_from_base'],map_origin=origin,resolution=res)
    a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()
