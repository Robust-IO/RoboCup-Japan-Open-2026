"""Import a reviewed inner mouth only; back up config, never rewrite the map."""
import argparse,json,math,shutil,datetime
from pathlib import Path
import numpy as np

def validate(row,destination):
    if row.get('destination_id')!=destination:raise ValueError('destination_id_mismatch')
    p=np.asarray(row['polygon_map_m'],float)
    if p.ndim!=2 or p.shape[1]!=2 or len(p)<3 or not np.isfinite(p).all():raise ValueError('invalid_opening_polygon')
    # The exporter picks an ordered boundary. Reject crossed or degenerate selections.
    area=abs(float(np.dot(p[:,0],np.roll(p[:,1],-1))-np.dot(p[:,1],np.roll(p[:,0],-1))))/2
    if area<.002 or area>2 or len(np.unique(p,axis=0))!=len(p):raise ValueError('invalid_inner_boundary_order_or_area')
    def cross(a,b,c):return float(np.cross(b-a,c-a))
    for i in range(len(p)):
        a,b=p[i],p[(i+1)%len(p)]
        for j in range(i+1,len(p)):
            if j==i+1 or i==0 and j==len(p)-1:continue
            c,d=p[j],p[(j+1)%len(p)]
            if max(min(a[0],b[0]),min(c[0],d[0]))>min(max(a[0],b[0]),max(c[0],d[0])):continue
            if max(min(a[1],b[1]),min(c[1],d[1]))>min(max(a[1],b[1]),max(c[1],d[1])):continue
            if cross(a,b,c)*cross(a,b,d)<=0 and cross(c,d,a)*cross(c,d,b)<=0:raise ValueError('crossed_inner_boundary')
    if not math.isfinite(row['rim_height_m']) or not 0<row['rim_height_m']<2:raise ValueError('invalid_rim_height')
    if not row.get('source'):raise ValueError('opening_provenance_required')
    return row

def main():
    p=argparse.ArgumentParser();p.add_argument('--opening',type=Path,required=True);p.add_argument('--config',type=Path,required=True);a=p.parse_args()
    config=json.loads(a.config.read_text());row=validate(json.loads(a.opening.read_text()),config['destination_id'])
    previous=config.get('container_opening')
    if previous:
        delta=np.linalg.norm(np.asarray(row['polygon_map_m']).mean(0)-np.asarray(previous['polygon_map_m']).mean(0))
        if delta>.5:raise ValueError('opening_map_frame_disagrees_over_50cm_check_layout_and_origin')
    backup=a.config.with_suffix('.json.backup-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    shutil.copy2(a.config,backup);config['container_opening']=row
    tmp=a.config.with_suffix('.tmp');tmp.write_text(json.dumps(config,indent=2));tmp.replace(a.config)
    print(json.dumps(dict(updated=str(a.config),backup=str(backup))))
if __name__=='__main__':main()
