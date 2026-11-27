from pathlib import Path
import sys,math,json
from runtime_resources import RuntimeResources
owner=RuntimeResources(Path(sys.argv[1]),Path('/__HANDYMAN_PACKAGE__/runtime/autograsp-v1/replay-inputs/hand.json'),Path(sys.argv[2]))
try:
 b=owner.open();rt=b['runtime'];m=b['motion_backend'];rt.sample(profile='state');s=rt.state()
 if s['joints'][5]<.6:raise ValueError('empty_hand_required')
 m.stage='withdraw';m.adjust('away-from-chair',s['poses']['map'],[-.35,0.,0.],s['joints'][:5],s['joints'][5])
 rt.sample(profile='state');s=rt.state();p=s['poses']['map'];yaw=math.atan2(-10.4-p[1],-3.9-p[0]);delta=math.atan2(math.sin(yaw-p[2]),math.cos(yaw-p[2]))
 m.adjust('face-runtime-bin',p,[0.,0.,delta],s['joints'][:5],s['joints'][5])
 for i,tilt in enumerate([-.5,-.7]):
  rt.face_source((0.,tilt));item=b['grasp_backend']._sample();path=rt.save(item,'bin-view-'+str(i));print(str(path),flush=True)
finally:owner.close()
