"""RGB evidence of a visible beak fixed relative to the hand across a real lift."""
import json
from pathlib import Path
import cv2,numpy as np
base=Path('/mnt/d/handyman-training-runs');prior=base/'penguin-native-transfer-20261025-10'
reference=prior/'grasp/captures/014-near-rejected';rgb=cv2.imread(str(reference/'hand.png'))
# The calibrated hand view and existing head RGBD place the face in this broad
# image region. Detect its colored beak; robot fingers carry no yellow motif.
hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV);mask=cv2.inRange(hsv,np.array([15,90,50]),np.array([45,255,255]))
mask[:90]=0;mask[390:]=0;mask[:,:60]=0;mask[:,350:]=0
n,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
candidates=[(st,centers[i]) for i,st in enumerate(stats) if i and 100<=st[4]<=6000]
assert len(candidates)==1,candidates
st,center=candidates[0];x,y,w,h=map(int,st[:4]);pad=7
x1,y1,x2,y2=max(0,x-pad),max(0,y-pad),min(640,x+w+pad),min(480,y+h+pad)
template=rgb[y1:y2,x1:x2];rows=[]
captures=[prior/'grasp/captures/013-near',reference]+sorted((base/'penguin-current-head-lift-test-20261025-01/captures').iterdir())
for folder in captures:
 frame=cv2.imread(str(folder/'hand.png'));search=frame[max(0,y1-32):min(480,y2+32),max(0,x1-32):min(640,x2+32)]
 match=cv2.matchTemplate(search,template,cv2.TM_CCOEFF_NORMED);_,score,_,where=cv2.minMaxLoc(match)
 found=[max(0,x1-32)+where[0],max(0,y1-32)+where[1]]
 patch=frame[found[1]:found[1]+template.shape[0],found[0]:found[0]+template.shape[1]]
 yellow=cv2.inRange(cv2.cvtColor(patch,cv2.COLOR_BGR2HSV),np.array([15,90,50]),np.array([45,255,255]))
 rows.append(dict(capture=str(folder),correlation=score,pixel_shift=[found[0]-x1,found[1]-y1],
                  current_yellow_pixels=int(np.count_nonzero(yellow)),reference_yellow_pixels=int(st[4])))
result=dict(robot_commands_sent=False,reference=str(reference),template_box=[x1,y1,x2,y2],rows=rows,
    interpretation='RGB appearance/relative-hand tracking only; does not create new 3D depth positions')
with (base/'penguin-hand-face-lift-follow-audit-20261025.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result))
