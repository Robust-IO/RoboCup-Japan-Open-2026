"""Recenter clipped detections; confidence is a view hint, not identity acceptance."""
import math

def recenter_head(rows,shape,head,k):
    height,width=shape[:2];pan,tilt=head
    for d in sorted(rows,key=lambda d:d['confidence'],reverse=True):
        if d['confidence']<.2:continue
        x1,y1,x2,y2=d['box']
        if min(x1,y1)>3 and x2<width-3 and y2<height-3:continue
        dx=math.atan2((x1+x2)/2-width/2,k[0]);dy=math.atan2((y1+y2)/2-height/2,k[4])
        goal=(max(-.6,min(.6,pan-max(-.25,min(.25,dx)))),max(-.8,min(0.,tilt-max(-.25,min(.25,dy)))))
        if max(abs(goal[0]-pan),abs(goal[1]-tilt))>.025:return goal
    return None
