import json,subprocess,os,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');f=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');root=b/('tumbler-kitchen-observe-'+time.strftime('%Y%m%d-%H%M%S'));root.mkdir()
base='source /opt/ros/humble/setup.bash; source /__HANDYMAN_PACKAGE__/project/install/setup.bash; export PYTHONPATH=/__HANDYMAN_PACKAGE__/runtime:/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1:$PYTHONPATH; export ROS_LOG_DIR='+str(root/'roslogs')+'; exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml --run -- '
def run(args,name):
 import shlex
 with (root/(name+'.log')).open('w') as log:r=subprocess.run(['bash','-lc',base+shlex.join(args)],stdout=log,stderr=subprocess.STDOUT)
 if r.returncode:raise RuntimeError(name+':'+(root/(name+'.log')).read_text()[-2000:])
receipt=dict(output=str(root),source='white_side_table#kitchen',destination='white_side_table#living_room',object='tumbler',status='preflight');(b/'tumbler-task-latest.json').write_text(json.dumps(receipt,indent=2))
run(['/usr/bin/python3',str(f/'inspect_transfer_ready.py'),'--output',str(root/'preflight.json'),'--efficient-transfer','--joint-feedback-max-age-s','.6'],'preflight')
r=json.loads((root/'preflight.json').read_text());j=r['joints'];assert j['hand_motor_joint']>.6 and abs(j['arm_lift_joint'])<.02 and abs(j['arm_flex_joint'])<.02
run(['/usr/bin/python3',str(f/'navigate_carry_trial.py'),'--run','--goal','9.205','1.482962','-0.012512','--holding-opening',str(j['hand_motor_joint']),'--verify-current-precision','--motion-profile','efficiency_v1','--semantic-map','/__HANDYMAN_PACKAGE__/maps/export/LayoutA/semantic_map.json','--joint-feedback-max-age-s','.85','--recover-feedback-interruption','--output',str(root/'navigation.json')],'navigation')
run(['/usr/bin/python3',str(b/'probe_stop_observe_act_20261101.py'),'--output',str(root/'capture')],'capture')
receipt.update(status='source_observation_captured');(b/'tumbler-task-latest.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
