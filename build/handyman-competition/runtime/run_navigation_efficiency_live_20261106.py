"""Empty-arm living/bedroom round trip. Does not grasp, release, or send protocol."""
import os,subprocess,sys,time,json,math
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');frozen=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
if os.environ.get('ROS_DOMAIN_ID')!='71':
    stamp=time.strftime('%Y%m%d-%H%M%S');out=base/('navigation-efficiency-live-'+stamp)
    command='source /opt/ros/humble/setup.bash; source /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/install/setup.bash; export ROS_LOG_DIR='+str(out)+'-roslogs; exec /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 '+str(Path(__file__).resolve())+' '+str(out)
    out.mkdir()
    with (out/'console.log').open('x') as log:child=subprocess.Popen(['bash','-lc',command],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    receipt=dict(pid=child.pid,output=str(out),started_ns=time.time_ns(),robot_commands='navigation_only',grasp_commands_sent=False,release_commands_sent=False)
    (base/'navigation-efficiency-live-latest.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt));raise SystemExit(0)
sys.path.insert(0,str(frozen));from apple_transfer_automation import RobotLock,check_inputs
import yaml
root=Path(sys.argv[1]);report=dict(status='starting',stages=[],grasp_commands_sent=False,release_commands_sent=False,task_finished_sent=False);started=time.monotonic()
def save():report['elapsed_s']=time.monotonic()-started;(root/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
with RobotLock():
    try:
        cfg=json.loads((frozen/'config/apple_transfer_auto_v1.json').read_text());check_inputs(cfg)
        probe=json.loads((base/'navigation-efficiency-live-probe-latest.json').read_text())
        if probe['status']!='connected' or any(probe['command_publishers'].values()):raise ValueError('idle_connected_robot_required')
        expected=[0.,0.,0.,-math.pi/2,0.,.65]
        if any(abs(a-b)>.02 for a,b in zip(probe['joints'],expected)):raise ValueError('empty_tucked_arm_required_for_this_test')
        origin=probe['pose'];report['origin']=origin
        env=yaml.safe_load(Path(cfg['environment']).read_text())
        for label,source,destination,goal in [('outbound','living_room','bedroom',cfg['source_staging_goal']),('return','bedroom','living_room',origin)]:
            route=next(r for r in env['routes'] if r['from']==source and r['to']==destination)
            command=[sys.executable,str(frozen/'navigate_carry_trial.py'),'--run','--motion-profile','efficiency_v1','--verify-current-precision','--holding-opening','.65','--semantic-map',cfg['semantic_map'],'--goal',*map(str,goal),'--output',str(root/(label+'.json'))]
            for w in route['waypoints']:command+=['--waypoint',*[str(w[k]) for k in ('x','y','yaw')]]
            report['active_stage']=label;save();begin=time.monotonic()
            with (root/(label+'.log')).open('x') as log:result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
            detail=json.loads((root/(label+'.json')).read_text());report['stages'].append(dict(stage=label,elapsed_s=time.monotonic()-begin,status=detail['status'],error=detail.get('error'),feedback_recoveries=detail['feedback_recoveries'],nav2_retries=len(detail['nav2_retries']),passed_points=len(detail['reached'])));save()
            if result.returncode or detail['status']!='arrived':raise RuntimeError('navigation_stage_failed:'+label+':'+str(detail.get('error')))
        report.update(status='round_trip_completed',final_pose=detail['final_pose']);save()
    except BaseException as e:report.update(status='failed',error=str(e) or type(e).__name__);save();raise
