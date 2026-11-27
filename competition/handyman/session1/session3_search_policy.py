"""Session3 room-shape selection and finite, explicit search completion."""
import math
import numpy as np

def aspect_ratio(polygon):
    p=np.asarray(polygon,float)
    if p.ndim!=2 or p.shape[1]!=2 or len(p)<3 or not np.isfinite(p).all():raise ValueError('invalid_room_polygon')
    from scipy.spatial import ConvexHull
    p=p[ConvexHull(p).vertices];best=None
    for a,b in zip(p,np.roll(p,-1,axis=0)):
        theta=math.atan2(*(b-a)[::-1]);c,s=math.cos(theta),math.sin(theta)
        q=p@np.array([[c,-s],[s,c]]);size=np.ptp(q,axis=0)
        if min(size)<=0:continue
        row=(float(np.prod(size)),float(max(size)/min(size)),size.tolist())
        if best is None or row[0]<best[0]-1e-8:best=row
    if best is None:raise ValueError('degenerate_room')
    return best[1],best[2]

def choose_points(room_row,polygon,threshold=1.35):
    if not math.isfinite(threshold) or threshold<1:raise ValueError('invalid_aspect_threshold')
    ratio,size=aspect_ratio(polygon)
    if not room_row.get('main'):raise ValueError('no_reachable_main_search_point')
    backups=room_row.get('backups',[]) if ratio>threshold else []
    if ratio>threshold and not backups:raise ValueError('elongated_room_backup_missing')
    points=[room_row['main']]+backups[:2]
    return dict(points=[{k:float(p[k]) for k in ('x','y','yaw')} for p in points],
        aspect_ratio=ratio,room_dimensions_m=size,aspect_ratio_threshold=threshold,
        strategy='main_and_backups' if backups else 'main_only',
        coverage_basis='configured_observation_views_not_visibility_proof')
