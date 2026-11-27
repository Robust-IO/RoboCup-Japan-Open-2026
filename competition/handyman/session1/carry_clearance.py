"""Static-map carry envelope and translation-only escape planning. No ROS commands."""
import math
import numpy as np
from scipy.spatial import ConvexHull
from scipy.ndimage import distance_transform_edt

class CarryClearance:
    def __init__(self,selector,vertices_base,margin=.03):
        self.selector=selector
        a=np.arange(32)*2*np.pi/32
        base=np.c_[np.cos(a),np.sin(a)]*(selector.radius+margin)
        v=np.asarray(vertices_base,float)
        if v.ndim!=2 or v.shape[1]!=3 or not len(v) or not np.isfinite(v).all():raise ValueError('invalid_carry_envelope')
        # Include the margin on every measured hand/object point, not just the base.
        expanded=(v[:,None,:2]+np.c_[np.cos(a[::4]),np.sin(a[::4])][None,:,:]*margin).reshape(-1,2)
        points=np.r_[base,expanded];self.polygon=points[ConvexHull(points).vertices]
        self.radius=float(np.linalg.norm(self.polygon,axis=1).max())
        self.clearance=distance_transform_edt(np.pad(selector.free,1))[1:-1,1:-1]*selector.res
        # Sample polygon interior as well as edges; thin furniture must not be skipped.
        step=selector.res*.4
        xx,yy=np.meshgrid(np.arange(points[:,0].min(),points[:,0].max()+step,step),np.arange(points[:,1].min(),points[:,1].max()+step,step))
        q=np.c_[xx.ravel(),yy.ravel()];h=ConvexHull(self.polygon)
        self.samples=np.r_[q[(q@h.equations[:,:2].T+h.equations[:,2]<=1e-8).all(1)],self.polygon]
    def turn_clear(self,pose):
        c=self.selector.cell(pose)
        return self.selector.valid(c) and self.clearance[c]>self.radius+self.selector.res*.71
    def turn_path(self,pose,delta):
        """Evaluate the full signed sweep, including both endpoints.

        Positive yaw is left/CCW. The angular step bounds corner travel to
        0.4 map cells; the polygon already includes the physical safety margin.
        """
        if not all(math.isfinite(v) for v in (*pose,delta)):
            raise ValueError('invalid_turn_pose')
        step=min(.05,self.selector.res*.4/max(self.radius,.01))
        margins=[];blocked=False
        origin=np.asarray(self.selector.origin)
        a=origin[2];rot=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
        for yaw in np.linspace(pose[2],pose[2]+delta,max(2,math.ceil(abs(delta)/step)+1)):
            p=[pose[0],pose[1],float(yaw)]
            if self.occupied(p):blocked=True;break
            c,s=math.cos(yaw),math.sin(yaw)
            points=self.polygon@np.array([[c,s],[-s,c]])+np.asarray(pose[:2])-origin[:2]
            cells=np.floor((points@rot)[:,::-1]/self.selector.res).astype(int)
            # occupied() has already rejected out-of-map polygon vertices.
            margins.append(float(self.clearance[cells[:,0],cells[:,1]].min()))
        return dict(delta_rad=float(delta),direction='left' if delta>0 else 'right',
                    clear=not blocked,min_clearance_m=min(margins) if margins else 0.,
                    mean_clearance_m=float(np.mean(margins)) if margins else 0.)
    def choose_turn(self,pose,target_yaw):
        if not all(math.isfinite(v) for v in (*pose,target_yaw)):
            raise ValueError('invalid_turn_pose')
        short=math.atan2(math.sin(target_yaw-pose[2]),math.cos(target_yaw-pose[2]))
        if abs(short)<1e-6:return dict(delta_rad=0.,direction='none',clear=True,alternatives=[])
        cell=self.selector.cell(pose)
        if self.selector.valid(cell) and self.clearance[cell]>self.radius+.15+self.selector.res:
            return dict(delta_rad=short,direction='left' if short>0 else 'right',clear=True,alternatives=[],basis='full_circle_has_spare_clearance')
        left=short if short>0 else short+2*math.pi
        alternatives=[self.turn_path(pose,left),self.turn_path(pose,left-2*math.pi)]
        valid=[x for x in alternatives if x['clear']]
        if not valid:raise ValueError('no_clear_carry_turn_arc')
        # One-cell differences are map quantization, not a reason for a full lap.
        # Once both sweeps have 15cm spare outside the inflated envelope,
        # extra empty room is not useful enough to justify a near-full turn.
        best=max(valid,key=lambda x:(round(min(.15,x['min_clearance_m'])/self.selector.res),
                  round(min(.15,x['mean_clearance_m'])/self.selector.res),-abs(x['delta_rad'])))
        return dict(best,alternatives=alternatives)
    def occupied(self,pose):
        c,s=math.cos(pose[2]),math.sin(pose[2]);r=np.array([[c,-s],[s,c]])
        points=self.samples@r.T+pose[:2]-self.selector.origin[:2]
        a=self.selector.origin[2];rot=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
        cells=np.floor((points@rot)[:,::-1]/self.selector.res).astype(int)
        inside=(cells[:,0]>=0)&(cells[:,1]>=0)&(cells[:,0]<self.selector.free.shape[0])&(cells[:,1]<self.selector.free.shape[1])
        blocked=~inside;blocked[inside]=~self.selector.free[cells[inside,0],cells[inside,1]]
        return set(map(tuple,cells[blocked]))
    def escape(self,pose,*,excluded=(),target_yaw=None):
        def endpoint_clear(q):
            if target_yaw is None:return self.turn_clear(q)
            try:return self.choose_turn(q,target_yaw)['clear']
            except ValueError as exc:
                if str(exc)!='no_clear_carry_turn_arc':raise
                return False
        if endpoint_clear(pose) and not excluded:return dict(delta=[0.,0.,0.],pose=list(pose),distance_m=0.)
        initial=self.occupied(pose);c,s=math.cos(pose[2]),math.sin(pose[2]);r=np.array([[c,-s],[s,c]])
        # Retreat preferred; sideways holonomic translation is allowed without yaw.
        for distance in np.arange(.10,1.21,.05):
            for angle in (math.pi,3*math.pi/4,-3*math.pi/4,math.pi/2,-math.pi/2,math.pi/4,-math.pi/4,0.):
                local=distance*np.array([math.cos(angle),math.sin(angle)]);world=r@local
                goal=[pose[0]+world[0],pose[1]+world[1],pose[2]]
                if any(np.linalg.norm(np.asarray(goal[:2])-np.asarray(p[:2]))<.15 for p in excluded):continue
                if not endpoint_clear(goal):continue
                previous=initial;valid=True
                for t in np.linspace(0,1,max(2,math.ceil(distance/(self.selector.res*.4)))):
                    occupied=self.occupied([pose[0]+t*world[0],pose[1]+t*world[1],pose[2]])
                    # Allow leaving existing map overlap, never entering new occupied cells.
                    if not occupied.issubset(previous):valid=False;break
                    previous=occupied
                if valid:return dict(delta=[float(local[0]),float(local[1]),0.],pose=goal,distance_m=float(distance))
        raise ValueError('no_translation_only_carry_escape_on_static_map')

class ProgressWatch:
    """Windowed translational progress; oscillating yaw does not reset the timer."""
    def __init__(self,seconds=12.,distance=.06):self.seconds=seconds;self.distance=distance;self.anchor=None;self.since=None
    def stalled(self,pose,now,goal):
        if self.anchor is None or math.dist(pose[:2],self.anchor[:2])>=self.distance:
            self.anchor=list(pose);self.since=now
        # At final XY, heading convergence is real progress (not oscillation).
        error=abs(math.atan2(math.sin(goal[2]-pose[2]),math.cos(goal[2]-pose[2])))
        if math.dist(pose[:2],goal[:2])<.08:
            if not hasattr(self,'best') or error<self.best-.08:self.since=now;self.best=error
        return now-self.since>=self.seconds
