from pathlib import Path
import sys,json
from runtime_resources import RuntimeResources
owner=RuntimeResources(Path(sys.argv[1]),Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/autograsp-v1/replay-inputs/hand.json'),Path(sys.argv[2]))
try:
 b=owner.open();rt=b['runtime']
 for i,head in enumerate([(0.,-.5),(.6,-.5),(.6,-.7)]):
  rt.face_source(head);item=b['grasp_backend']._sample();path=rt.save(item,'bin-current-'+str(i));print(str(path),flush=True)
finally:owner.close()
