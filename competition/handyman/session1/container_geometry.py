"""Place the measured object's projection inside the actual inner opening."""
import numpy as np
from perception_bridge import inside

def projection_inside(points,polygon,margin=.005):
    p=np.asarray(points,float);poly=np.asarray(polygon,float)
    if p.ndim!=2 or p.shape[1]!=2 or not len(p) or not np.isfinite(p).all():return False
    # A convex object projection must not bridge a concavity of the mouth.
    from scipy.spatial import ConvexHull,QhullError
    try:p=p[ConvexHull(p).vertices]
    except QhullError:pass
    for a,b in zip(p,np.roll(p,-1,axis=0)):
        for q in np.linspace(a,b,max(2,int(np.linalg.norm(b-a)/.003)+1)):
            if not inside(*q,poly.tolist()):return False
            for c,d in zip(poly,np.roll(poly,-1,axis=0)):
                delta=d-c;t=np.clip(np.dot(q-c,delta)/max(np.dot(delta,delta),1e-12),0,1)
                if np.linalg.norm(q-c-t*delta)<margin-1e-9:return False
    return True

def fit_release_xy(opening,offsets,preferred,*,margin=.005):
    """Maximise the worst object-to-rim clearance, rather than shortest approach."""
    from scipy.optimize import linprog
    poly=np.asarray(opening['polygon_map_m'],float);offsets=np.asarray(offsets,float)
    center=poly.mean(0)-(offsets.min(0)+offsets.max(0))/2
    edges=np.roll(poly,-1,axis=0)-poly
    area=np.sum(poly[:,0]*np.roll(poly[:,1],-1)-poly[:,1]*np.roll(poly[:,0],-1))
    lengths=np.linalg.norm(edges,axis=1)
    if np.any(lengths<1e-9) or abs(area)<1e-10:raise ValueError('degenerate_container_opening')
    normals=np.c_[-edges[:,1],edges[:,0]]/lengths[:,None]*np.sign(area)
    convex=all(np.all((poly-a)@n>=-1e-9) for a,n in zip(poly,normals))
    if convex:
        # Every projected object point and the gripper center must remain in
        # each inward half-plane with a shared, maximised edge distance.
        support=np.min(offsets@normals.T,axis=0)-np.sum(poly*normals,axis=1)
        result=linprog([0.,0.,-1.],A_ub=np.c_[-normals,np.ones(len(poly))],b_ub=support,
                       bounds=[(None,None),(None,None),(0,None)],method='highs')
        if not result.success or result.x[2]<margin-1e-9:
            raise ValueError('held_object_projection_does_not_fit_opening')
        center_clearance=float(np.min(normals@center+support))
        # For equal-clearance solutions prefer the geometric center, not the
        # old seven-centimetre shortened approach point.
        q=center if center_clearance>=max(margin,result.x[2]-.001) else result.x[:2]
        return q.tolist()
    # Preserve support for non-convex reviewed mouths with a bounded grid.
    candidates=[center,np.asarray(preferred,float)]
    for x in np.arange(poly[:,0].min(),poly[:,0].max()+.005,.01):
        for y in np.arange(poly[:,1].min(),poly[:,1].max()+.005,.01):candidates.append(np.array([x,y]))
    best=None;score=None
    for q in candidates:
        points=offsets+q
        if not projection_inside(points,poly,margin=margin):continue
        low=margin;high=float(np.ptp(poly,axis=0).max())
        for _ in range(10):
            mid=(low+high)/2
            if projection_inside(points,poly,margin=mid):low=mid
            else:high=mid
        key=(round(low,3),-float(np.linalg.norm(q-center)))
        if score is None or key>score:best=q;score=key
    if best is not None:return best.tolist()
    raise ValueError('held_object_projection_does_not_fit_opening')
