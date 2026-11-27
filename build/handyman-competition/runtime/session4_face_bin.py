from pathlib import Path
import sys,math,json
from runtime_resources import RuntimeResources
owner=RuntimeResources(Path(sys.argv[1]),Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/autograsp-v1/replay-inputs/hand.json'),Path(sys.argv[2]))
try:
 b=owner.open();rt=b['runtime'];m=b['motion_backend'];rt.sample(profile='state');s=rt.state()
 if s['joints'][5]<.6:raise ValueError('empty_hand_required')
 m.stage='observe_target'
 for index in range(6):
  rt.sample(profile='state');s=rt.state();p=s['poses']['map'];yaw=math.atan2(-10.4-p[1],-3.9-p[0]);delta=math.atan2(math.sin(yaw-p[2]),math.cos(yaw-p[2]))
  if abs(delta)<.03:break
  step=max(-.30,min(.30,delta))
  m.adjust('face-bin-'+str(index),p,[0.,0.,step],s['joints'][:5],s['joints'][5])
 else:raise ValueError('facing_not_converged')
 for i,tilt in enumerate([-.5,-.7]):
  rt.face_source((0.,tilt));item=b['grasp_backend']._sample();path=rt.save(item,'bin-view-'+str(i));print(str(path),flush=True)
finally:owner.close()
