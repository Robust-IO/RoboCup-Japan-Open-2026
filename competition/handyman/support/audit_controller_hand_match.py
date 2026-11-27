import cv2,numpy as np,json
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');old=cv2.imread(str(b/'controller-overhead-handle-clearance-20261107-173547/captures/lift-proof-2/hand.png'));new=cv2.imread(str(b/'controller-dining-carry-20261107-173906/captures/dining-table/hand.png'));mask=np.zeros(old.shape[:2],np.uint8);mask[80:410,420:640]=255;s=cv2.SIFT_create(nfeatures=1200);ka,da=s.detectAndCompute(cv2.cvtColor(old,cv2.COLOR_BGR2GRAY),mask);kb,db=s.detectAndCompute(cv2.cvtColor(new,cv2.COLOR_BGR2GRAY),None)
for ratio in (.72,.8,.85):
 pairs=[m for m,n in cv2.BFMatcher().knnMatch(da,db,k=2) if m.distance<ratio*n.distance];aa=np.float32([ka[m.queryIdx].pt for m in pairs]);bb=np.float32([kb[m.trainIdx].pt for m in pairs]);H,i=cv2.findHomography(aa,bb,cv2.RANSAC,3.);print(json.dumps(dict(ratio=ratio,matches=len(pairs),inliers=int(i.sum()) if i is not None else 0,H=H.tolist() if H is not None else None)))
