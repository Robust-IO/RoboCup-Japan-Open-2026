"""Measured thin axial sections shared by candidate generation and scoring."""
import numpy as np

def sections(points):
    p=np.asarray(points,float);center=np.median(p,axis=0)
    _,v=np.linalg.eigh(np.cov((p-center).T));axis=v[:,-1]
    u=(p-center)@axis;lo,hi=np.quantile(u,[.02,.98]);length=max(hi-lo,.01)
    half=max(.004,min(.008,length*.04));radial=(p-center)-u[:,None]*axis
    rows=[]
    for a in np.linspace(lo+.04*length,hi-.04*length,25):
        mask=abs(u-a)<half;q=radial[mask]
        if len(q)<8:continue
        w=float(2*np.quantile(np.linalg.norm(q-np.median(q,axis=0),axis=1),.9))
        rows.append((a,w,np.median(p[mask],axis=0)))
    if not rows:return center,axis,half,[]
    body=max(w for _,w,_ in rows)
    neck=[row for row in rows if row[1]<=body*.8 and body-row[1]>=.008]
    # Keep all observed neck sections narrower than the body.
    # A single narrowest slice must not veto another usable neck contact.
    return center,axis,half,neck
