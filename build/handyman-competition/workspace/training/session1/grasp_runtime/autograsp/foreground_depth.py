"""Separate a detected foreground from distant background without identity rejection."""
import numpy as np

def foreground_mask(depths,bin_m=.02):
 d=np.asarray(depths,float);valid=np.isfinite(d)&(d>.05)
 if valid.sum()<30:return valid
 lo=float(d[valid].min());bins=np.floor((d[valid]-lo)/bin_m).astype(int)
 counts=np.bincount(bins);dense=counts>=max(8,int(counts.max()*.04))
 # Fill one missing histogram bin, but not a large foreground/background gap.
 if len(dense)>2:dense[1:-1]|=dense[:-2]&dense[2:]
 ix=np.flatnonzero(dense)
 if not len(ix):return valid
 groups=np.split(ix,np.where(np.diff(ix)>1)[0]+1)
 masses=[int(counts[g].sum()) for g in groups]
 substantial=[g for g,m in zip(groups,masses) if m>=max(30,max(masses)*.2)]
 if len(substantial)<2:return valid
 front=substantial[0];limit=lo+(front[-1]+1)*bin_m+.01
 return valid&(d<=limit)
