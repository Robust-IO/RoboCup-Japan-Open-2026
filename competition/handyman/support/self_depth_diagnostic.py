"""CPU z-buffer and conservative depth comparison; offline only."""
import argparse
import base64
import json
import hashlib
from pathlib import Path
import numpy as np

def raster(vertices, triangles, k, shape):
    result=np.full(shape,np.inf)
    skipped=0
    for ids in triangles:
        v=vertices[ids]
        if np.any(v[:,2]<=.01):skipped+=1;continue
        p=(k@v.T).T;p=p[:,:2]/p[:,2,None]
        lo=np.maximum(np.floor(p.min(0)).astype(int),[0,0]);hi=np.minimum(np.ceil(p.max(0)).astype(int),[shape[1]-1,shape[0]-1])
        if np.any(lo>hi):continue
        a,b,c=p;den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
        if abs(den)<1e-10:continue
        # CameraInfo pixel coordinates are integer pixel centers.
        x,y=np.meshgrid(np.arange(lo[0],hi[0]+1),np.arange(lo[1],hi[1]+1))
        u=((b[1]-c[1])*(x-c[0])+(c[0]-b[0])*(y-c[1]))/den
        w=((c[1]-a[1])*(x-c[0])+(a[0]-c[0])*(y-c[1]))/den;t=1-u-w
        inside=(u>=-1e-8)&(w>=-1e-8)&(t>=-1e-8)
        inv=u/v[0,2]+w/v[1,2]+t/v[2,2]
        z=np.full_like(inv,np.inf);np.divide(1,inv,out=z,where=inside&(inv>0))
        dest=result[lo[1]:hi[1]+1,lo[0]:hi[0]+1];np.minimum(dest,z,out=dest)
    return result,skipped

def classify(expected,observed,tolerance=.015):
    projected=np.isfinite(expected);valid=np.isfinite(observed)&(observed>0)
    out=np.zeros(expected.shape,dtype=np.uint8)
    out[projected]=3 # invalid or discrepant depth: preserve, not self
    out[projected&valid&(observed<expected-tolerance)]=2 # foreground: preserve
    out[projected&valid&(np.abs(observed-expected)<=tolerance)]=1 # candidate agreement only
    return out

def main():
    p=argparse.ArgumentParser();p.add_argument('geometry',type=Path);p.add_argument('snapshot',type=Path);a=p.parse_args()
    s=json.loads((a.snapshot/'snapshot.json').read_text());data=a.geometry.read_bytes()
    if hashlib.sha256(data).hexdigest()!=s['geometry_sha256']:raise ValueError('Geometry changed')
    g=json.loads(data);observed=np.load(a.snapshot/'observed_depth_m.npy');expected=np.full(observed.shape,np.inf);k=np.array(s['k']).reshape(3,3);skips=0
    for part in g['parts']:
        vertices=np.array([[v['x'],v['y'],v['z'],1] for v in part['vertices']])
        m=np.array(s['camera_from_link'][part['link']])@np.diag([-1,1,1,1])@np.array(part['meshToLinkUnityRowMajor']).reshape(4,4)
        points=(m@vertices.T).T[:,:3]
        z,n=raster(points,np.array(part['triangles']).reshape(-1,3),k,observed.shape);skips+=n;np.minimum(expected,z,out=expected)
    labels=classify(expected,observed);np.save(a.snapshot/'self_surface_depth_m.npy',expected);np.save(a.snapshot/'depth_review_labels.npy',labels)
    residual=observed[np.isfinite(expected)&np.isfinite(observed)&(observed>0)]-expected[np.isfinite(expected)&np.isfinite(observed)&(observed>0)]
    report=dict(approved_for_runtime=False,tolerance_m=.015,counts={str(i):int((labels==i).sum()) for i in range(4)},
        projected_pixels=int(np.isfinite(expected).sum()),
        projected_invalid_depth_pixels=int((np.isfinite(expected)&(~np.isfinite(observed)|(observed<=0))).sum()),
        projected_depth_behind_pixels=int((np.isfinite(expected)&np.isfinite(observed)&(observed>expected+.015)).sum()),
        expected_depth_min_max_m=[float(expected[np.isfinite(expected)].min()),float(expected[np.isfinite(expected)].max())] if np.isfinite(expected).any() else None,
        residual_p05_p50_p95_m=np.percentile(residual,[5,50,95]).tolist() if len(residual) else None,
        near_plane_triangles_skipped=skips,limitations=['15mm is an uncalibrated diagnostic tolerance, not a release setting.',
        'No semantic held-object guarantee; near-contact objects may be indistinguishable in depth.',
        'Near-plane crossing triangles skipped; nonopaque material rendering not modeled.'])
    (a.snapshot/'depth-review.json').write_text(json.dumps(report,indent=2))
    raw=base64.b64encode((a.snapshot/'rgb.png').read_bytes()).decode();h,w=labels.shape
    page='''<!doctype html><meta charset="utf-8"><title>自身表面深度核对</title><style>body{font:18px sans-serif;margin:24px}canvas{width:100%;max-width:1100px}</style><h1>自身表面深度核对（未上线）</h1><p>绿色：深度近似吻合；橙色：实测更近，保留；紫色：无效深度或不吻合，保留。未投影到的空隙不涂色。绿色也不是已批准过滤区域。</p><button onclick="show=!show;draw()">切换叠加</button><canvas id="c"></canvas><script>'''
    page+=f'const W={w},H={h},L={json.dumps(labels.flatten().tolist())};const c=document.getElementById("c");c.width=W;c.height=H;const ctx=c.getContext("2d"),im=new Image();let show=true;im.onload=draw;im.src="data:image/png;base64,{raw}";'
    page+='function draw(){ctx.drawImage(im,0,0);if(!show)return;const d=ctx.getImageData(0,0,W,H),cs=[[0,0,0],[0,230,100],[255,150,0],[200,30,240]];for(let i=0;i<L.length;i++){if(!L[i])continue;for(let j=0;j<3;j++)d.data[4*i+j]=.6*d.data[4*i+j]+.4*cs[L[i]][j];}ctx.putImageData(d,0,0);}</script>'
    (a.snapshot/'depth-review.html').write_text(page,encoding='utf-8');print(json.dumps(report),flush=True)

if __name__=='__main__':main()
