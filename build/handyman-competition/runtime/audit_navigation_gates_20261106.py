import os,sys,json,time,subprocess,hashlib
from pathlib import Path
BASE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
if os.environ.get('ROS_DOMAIN_ID')!='71':
    cmd='source /opt/ros/humble/setup.bash; export ROS_LOG_DIR=/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/navigation-audit-roslogs; exec /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 '+str(Path(__file__).resolve())
    raise SystemExit(subprocess.run(['bash','-lc',cmd]).returncode)
import rclpy
from rcl_interfaces.srv import GetParameters,ListParameters
rclpy.init();node=rclpy.create_node('read_only_navigation_gate_audit')
def call(kind,topic,req):
    c=node.create_client(kind,topic)
    try:
        if not c.wait_for_service(timeout_sec=1):raise RuntimeError('unavailable')
        f=c.call_async(req);end=time.monotonic()+4
        while not f.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        if not f.done():raise RuntimeError('timeout')
        return f.result()
    finally:node.destroy_client(c)
def value(v):
    return {1:lambda:v.bool_value,2:lambda:v.integer_value,3:lambda:v.double_value,4:lambda:v.string_value,6:lambda:list(v.bool_array_value),7:lambda:list(v.integer_array_value),8:lambda:list(v.double_array_value),9:lambda:list(v.string_array_value)}.get(v.type,lambda:None)()
report={'robot_commands_sent':False,'parameters_changed':False,'parameters':{},'processes':[],'hashes':{}}
for target in ('controller_server','planner_server','bt_navigator','local_costmap/local_costmap','global_costmap/global_costmap','behavior_server'):
    try:
        req=ListParameters.Request();req.depth=10
        names=call(ListParameters,'/'+target+'/list_parameters',req).result.names
        req=GetParameters.Request();req.names=names
        vals=call(GetParameters,'/'+target+'/get_parameters',req).values
        report['parameters'][target]=dict(zip(names,map(value,vals)))
    except Exception as e:report['parameters'][target]={'error':str(e)}
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:
        args=[s.decode(errors='replace') for s in (p/'cmdline').read_bytes().split(b'\0') if s]
    except (OSError,PermissionError):continue
    if args and (Path(args[0]).name in ('controller_server','bt_navigator','planner_server','behavior_server') or any('table_localization' in a or 'idle_navigation' in a for a in args)):
        report['processes'].append({'pid':p.name,'args':args})
frozen=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
for name in ('navigate_carry_trial.py','navigation_feedback_gate.py','navigation_feedback_recovery.py','navigation_entry.py','motion_speed.py','config/motion_speed_profiles.json','apple_transfer_auto_backend.py'):
    p=frozen/name
    if p.exists():report['hashes'][name]=hashlib.sha256(p.read_bytes()).hexdigest()
out=BASE/'navigation-gates-readonly-audit-20261106.json';out.write_text(json.dumps(report,ensure_ascii=False,indent=2))
node.destroy_node();rclpy.shutdown()
print(json.dumps(report,ensure_ascii=False))
