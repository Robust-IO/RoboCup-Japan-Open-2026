"""Tight enclosing disk of measured XY surface; no fixed class-size inflation."""
import math
import numpy as np

def enclosing_disk(points):
    p=np.asarray(points,float)
    if p.ndim!=2 or p.shape[1]!=2 or not len(p) or not np.isfinite(p).all():
        raise ValueError('finite_object_xy_required')
    p=np.unique(p,axis=0)
    if len(p)>2:
        from scipy.spatial import ConvexHull,QhullError
        try:p=p[ConvexHull(p).vertices]
        except QhullError:pass
    p=p[np.random.default_rng(0).permutation(len(p))]
    center=p[0].copy();radius=0.
    def outside(q):return np.linalg.norm(q-center)>radius+1e-10
    for i,a in enumerate(p):
        if not outside(a):continue
        center=a.copy();radius=0.
        for j,b in enumerate(p[:i]):
            if not outside(b):continue
            center=(a+b)/2;radius=float(np.linalg.norm(a-b)/2)
            for c in p[:j]:
                if not outside(c):continue
                mat=2*np.array([b-a,c-a]);rhs=np.array([b@b-a@a,c@c-a@a])
                if abs(np.linalg.det(mat))<1e-12:
                    pair=max(((a,b),(a,c),(b,c)),key=lambda v:np.linalg.norm(v[0]-v[1]))
                    center=(pair[0]+pair[1])/2
                else:center=np.linalg.solve(mat,rhs)
                radius=max(float(np.linalg.norm(v-center)) for v in (a,b,c))
    radius=float(np.max(np.linalg.norm(np.asarray(points)-center,axis=1)))
    return center,radius

def disk_inside(center,radius,bounds,gap=.005):
    center=np.asarray(center,float);c=bounds['center'];s=bounds['size'];yaw=bounds.get('yaw',0.)
    if center.shape!=(2,) or not np.isfinite(np.r_[center,radius,gap]).all() or radius<=0 or gap<0:
        raise ValueError('invalid_support_disk')
    co,si=math.cos(yaw),math.sin(yaw)
    local=np.array([[co,si],[-si,co]])@(center-[c['x'],c['y']])
    clear=np.asarray([s['x'],s['y']])/2-np.abs(local)-radius
    return dict(safe=bool((clear>=gap).all()),radius_m=float(radius),edge_clearance_m=float(clear.min()),
                required_edge_gap_m=gap,center_map_m=center.tolist(),basis='tight_measured_object_disk_inside_table')
