"""Room ownership of a localized visible surface, never a grasp authorization."""
import math
from rgbd_quality import transform_point


def inside(x,y,polygon):
    result=False
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:
            result=not result
    return result


def boundary_distance(x,y,polygon):
    distances=[]
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        dx,dy=b[0]-a[0],b[1]-a[1];length=dx*dx+dy*dy
        t=max(0.,min(1.,((x-a[0])*dx+(y-a[1])*dy)/length)) if length else 0.
        distances.append(math.hypot(x-a[0]-t*dx,y-a[1]-t*dy))
    return min(distances)


class RoomGate:
    def __init__(self,rooms,target_room,margin=.15):
        if target_room not in rooms or not math.isfinite(margin) or margin<=0:
            raise ValueError('invalid_room_gate')
        self.rooms={};self.target=target_room;self.margin=margin
        for name,vertices in rooms.items():
            p=[tuple(float(v) for v in point) for point in vertices]
            if len(p)<3 or any(len(a)!=2 or not all(math.isfinite(v) for v in a) for a in p):
                raise ValueError('invalid_room_polygon')
            if abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(p,p[1:]+p[:1])))<1e-9:
                raise ValueError('degenerate_room_polygon')
            self.rooms[name]=p

    def classify(self,point,translation,quaternion):
        mapped=transform_point(point,translation,quaternion)
        x,y=mapped[:2]
        owners=[name for name,p in self.rooms.items() if inside(x,y,p)]
        near=any(boundary_distance(x,y,p)<=self.margin for p in self.rooms.values())
        state=('boundary_uncertain' if near or len(owners)!=1 else
               'inside' if owners==[self.target] else 'outside_target_room')
        return dict(state=state,target_room=self.target,rooms=owners,
                    position_map_m=list(mapped),frame_id='map',boundary_margin_m=self.margin)
