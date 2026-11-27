"""Explicit depth decoding and pinhole optical coordinates, not robot coordinates."""
import math
import struct

def depth_at(data, width, height, step, encoding, bigendian, u, v):
    if encoding not in ('16UC1','32FC1'):
        raise ValueError('Unsupported depth encoding')
    size=2 if encoding=='16UC1' else 4
    if width<=0 or height<=0 or step<width*size or len(data)!=step*height:
        raise ValueError('Invalid depth buffer dimensions/stride')
    if not (0<=u<width and 0<=v<height):
        raise ValueError('Pixel outside depth image')
    raw=struct.unpack_from(('>' if bigendian else '<')+('H' if size==2 else 'f'),data,v*step+u*size)[0]
    metres=raw*.001 if encoding=='16UC1' else raw
    return metres if math.isfinite(metres) and metres>0 else None

def optical_point(u,v,z,k):
    if len(k)!=9 or not all(math.isfinite(x) for x in [u,v,z,*k]) or z<=0 or k[0]<=0 or k[4]<=0:
        raise ValueError('Invalid pinhole inputs')
    if abs(k[1])+abs(k[3])+abs(k[6])+abs(k[7])>1e-9 or abs(k[8]-1)>1e-9:
        raise ValueError('Unsupported camera matrix')
    return ((u-k[2])*z/k[0],(v-k[5])*z/k[4],z)
