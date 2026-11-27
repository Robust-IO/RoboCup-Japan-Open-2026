"""Bound work to the original pixel predicate, retaining full-image coordinates."""
import numpy as np


def pixel_window(shape, left, top, right, bottom):
    height,width=shape
    if not np.isfinite([left,top,right,bottom]).all():raise ValueError('invalid_pixel_window')
    x0=max(0,min(width,int(np.ceil(left))));x1=max(x0,min(width,int(np.ceil(right))))
    y0=max(0,min(height,int(np.ceil(top))));y1=max(y0,min(height,int(np.ceil(bottom))))
    v,u=np.indices((y1-y0,x1-x0))
    return (slice(y0,y1),slice(x0,x1)),u+x0,v+y0
