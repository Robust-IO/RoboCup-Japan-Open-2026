"""Launch read-only subscriptions using staged code. Never starts control nodes."""
import json,subprocess,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');validation=json.loads((b/'gate-revision-validation-latest.json').read_text())
if validation['status']!='offline_validated_not_deployed':raise ValueError('offline_validation_required')
stage=Path(validation['root']);output=stage/('read-only-live-'+time.strftime('%Y%m%d-%H%M%S'))
cmd='source /opt/ros/humble/setup.bash; source /__HANDYMAN_PACKAGE__/project/install/setup.bash; export PYTHONPATH='+str(stage/'runtime')+':'+str(stage/'frozen')+':/__HANDYMAN_PACKAGE__/runtime:/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1:$PYTHONPATH; export ROS_LOG_DIR='+str(stage/'read-only-probe-roslogs')+'; exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 '+str(stage/'tests/probe_stationary_capture.py')+' '+str(output)
log=output.with_name(output.name+'-console.log')
try:
    with log.open('x') as stream:r=subprocess.run(['bash','-lc',cmd],stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,timeout=25)
    report=dict(status='read_only_probe_finished',returncode=r.returncode,output=str(output),log=str(log),robot_commands_sent=False)
except subprocess.TimeoutExpired:
    report=dict(status='read_only_probe_waiting_for_unity_no_control',output=str(output),log=str(log),robot_commands_sent=False)
(stage/'read-only-probe-receipt.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
