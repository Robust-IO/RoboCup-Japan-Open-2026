"""Back up live anchor and hand off its single publisher, without motion."""
import json,os,signal,subprocess,sys,time,yaml
from pathlib import Path
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener
from table_localization_service import td,write

b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');receipt=b/'table-localizer-service.json'
def running(pid):
    try:return Path('/proc',str(pid),'stat').read_text().split()[2]!='Z'
    except FileNotFoundError:return False
if receipt.exists():
    old=json.loads(receipt.read_text())
    if running(old['pid']):print(json.dumps(dict(status='already_running',**old)));raise SystemExit(0)
rclpy.init();n=rclpy.create_node('install_single_table_localizer');buf=Buffer();listener=TransformListener(buf,n)
end=time.monotonic()+8
while time.monotonic()<end:
    rclpy.spin_once(n,timeout_sec=.02)
    if buf.can_transform('map','odom',Time()) and buf.can_transform('odom','base_footprint',Time()):break
anchor=td(buf.lookup_transform('map','odom',Time()));owners=[]
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:parts=[p.decode() for p in (proc/'cmdline').read_bytes().split(b'\0') if p]
    except (FileNotFoundError,PermissionError,UnicodeError):continue
    if parts and Path(parts[0]).name=='handyman_initial_pose_localizer':owners.append(dict(pid=int(proc.name),args=parts))
if len(owners)!=1:raise ValueError('unique_known_old_map_owner_required')
from canned_grasp_runtime import TOPICS
if any(n.count_publishers(t) for t in TOPICS):raise ValueError('robot_must_be_idle_for_map_owner_handoff')
root=b/'backups'/('table-localizer-handoff-'+time.strftime('%Y%m%d-%H%M%S'));root.mkdir();write(root/'old-anchor.json',anchor);write(root/'old-owner.json',owners[0])
out=b/('table-localizer-runtime-'+time.strftime('%Y%m%d-%H%M%S'));out.mkdir()
cfg=json.loads(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text());env=yaml.safe_load(Path(cfg['environment']).read_text());uri=env['map']
if uri.startswith('package://'):
    from ament_index_python.packages import get_package_share_directory
    package,relative=uri[len('package://'):].split('/',1);mapyaml=Path(get_package_share_directory(package))/relative
else:mapyaml=(Path(cfg['environment']).parent/uri).resolve()
if not mapyaml.is_file():raise ValueError('configured_navigation_map_missing')
command=[sys.executable,str(b/'table_localization_service.py'),'--initial-anchor',str(root/'old-anchor.json'),'--activate-file',str(out/'activate'),'--map',str(mapyaml),'--output',str(out)]
with (out/'console.log').open('x') as stream:child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
end=time.monotonic()+10
while time.monotonic()<end:
    if child.poll() is not None:raise ValueError('new_localizer_failed_before_handoff')
    if (out/'service-status.json').exists():break
    rclpy.spin_once(n,timeout_sec=.02)
else:raise ValueError('new_localizer_not_ready')
os.kill(owners[0]['pid'],signal.SIGINT);end=time.monotonic()+8
while time.monotonic()<end and (running(owners[0]['pid']) or 'initial_pose_localizer' in n.get_node_names()):rclpy.spin_once(n,timeout_sec=.05)
if running(owners[0]['pid']):raise ValueError('old_map_owner_not_stopped_no_activation')
(out/'activate').write_text('old_owner_stopped\n');end=time.monotonic()+5
while time.monotonic()<end:
    rclpy.spin_once(n,timeout_sec=.03)
    status=json.loads((out/'service-status.json').read_text())
    if status['status']=='active':break
else:raise ValueError('new_map_owner_not_active')
report=dict(status='single_map_owner_installed',pid=child.pid,output=str(out),backup=str(root),old_owner=owners[0],initial_anchor=anchor,command=command,robot_commands_sent=False)
write(receipt,report);n.destroy_node();rclpy.shutdown();print(json.dumps(report))
