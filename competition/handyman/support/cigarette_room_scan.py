"""Room-specified camera search for this task; never a room-absence proof.

Uses existing room observation points, Nav2 Spin collision checking and resident
whole-frame detection. Search results propose a surface for closer inspection;
they do not themselves authorize a grasp.
"""
from competition_paths import is_run_path
import argparse,json,math,os,signal,time,uuid
from pathlib import Path
import numpy as np
import cv2
from apple_transfer_automation import RobotLock,write_json
from apple_transfer_auto_backend import LiveBackend
from pepper_runtime import PepperRuntime
from apple_worker_client import Worker
from held_can_appearance import pixel_points
from apple_transfer_auto_plan import map_point
from scripts.search_room_gate import inside,boundary_distance
from task_outcome_policy import pose_matches

def candidate(sample,detection,environment):
    d=sample['depth'];h,w=d.shape;x1,y1,x2,y2=detection['box'];rgb=sample['rgb'];p=pixel_points(d,sample['snapshot'])
    u,v=np.meshgrid(np.arange(w),np.arange(h));core=(u>x1+.12*(x2-x1))&(u<x2-.12*(x2-x1))&(v>y1+.12*(y2-y1))&(v<y2-.12*(y2-y1))
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV);dh=abs(hsv[:,:,0].astype(float)-99);blue=(np.minimum(dh,180-dh)<=8)&(hsv[:,:,1]>45)&(hsv[:,:,2]>30)
    mask=core&blue&np.isfinite(d)&(d>0)&(d<4.)
    if mask.sum()<20:raise ValueError('insufficient_current_cigarette_surface_depth')
    z=d[mask]
    if np.quantile(z,.95)-np.quantile(z,.05)>.08:raise ValueError('cigarette_surface_depth_not_coherent')
    point=np.median(p[mask],axis=0);mapped=map_point(sample['snapshot'],point)
    owners=[name for name,r in environment['rooms'].items() if inside(*mapped[:2],r['region']) and boundary_distance(*mapped[:2],r['region'])>.15]
    if owners!=['bedroom']:raise ValueError('target_surface_not_uniquely_inside_bedroom')
    return dict(class_name='cigarette',confidence=detection['confidence'],surface_base_m=point.tolist(),surface_map_m=mapped.tolist(),
        actual_depth_pixels=int(mask.sum()),depth_spread_m=float(np.quantile(z,.95)-np.quantile(z,.05)),
        stamp_ns=sample['snapshot']['streams']['depth']['stamp_ns'],room='bedroom',position_semantics='visible_surface_not_grasp_pose')

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--run',action='store_true');parser.add_argument('--output',type=Path,required=True);a=parser.parse_args()
    if not a.run or os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('explicit_domain71_required')
    if not is_run_path(a.output.resolve()):raise ValueError('D_output_required')
    cfg=json.loads(Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text());cfg.update(joint_feedback_max_age_s=.6,navigation_joint_feedback_max_age_s=.85,navigation_feedback_recovery=True)
    root=a.output;runtime=backend=worker=None;manager=None;configured=False;active=None;opening=0.
    report=dict(status='starting',target='cigarette',room='bedroom',robot_grip_commands_sent=False,does_not_exist_authorized=False,views=[],candidates=[])
    def stop(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,stop)
    with RobotLock():
        root.mkdir(exist_ok=False);start=time.time_ns()
        with (root/'journal.jsonl').open('x') as log:
            def record(e):log.write(json.dumps(dict(time_ns=time.time_ns(),**e))+'\n');log.flush();write_json(root/'result.json',report)
            def stage(s):report['stage']=s;write_json(root/'result.json',report)
            try:
                import rclpy
                from rclpy.action import ActionClient
                from nav2_msgs.action import Spin
                from nav2_msgs.srv import ManageLifecycleNodes
                rclpy.init();epoch='cigarette-room-scan-'+uuid.uuid4().hex;runtime=PepperRuntime(root,epoch,record);backend=LiveBackend(cfg);backend.bind(root,epoch,record)
                runtime.sample(processing_reserve_ns=0);state=runtime.state();runtime.ownership();report['initial_state']=state;opening=state['joints'][5]
                if not pose_matches(state['joints'],[0,0,0,-math.pi/2,0,opening]) or opening not in (0.,):raise ValueError('initial_empty_tucked_search_pose_required')
                if backend.room_at(state['poses']['map'])!='bedroom':raise ValueError('bedroom_entry_required')
                worker=Worker(cfg['model_python'],cfg['registry'],root/'model.json',imgsz=640)
                manager=runtime.node.create_client(ManageLifecycleNodes,'/lifecycle_manager_navigation/manage_nodes')
                def wait(f,seconds):
                    end=time.monotonic()+seconds
                    while not f.done() and time.monotonic()<end:runtime.pump(.02)
                    if not f.done():raise ValueError('search_action_or_service_timeout')
                    return f.result()
                def lifecycle(command):
                    if not manager.wait_for_service(timeout_sec=5):raise ValueError('navigation_manager_unavailable')
                    req=ManageLifecycleNodes.Request();req.command=command
                    if not wait(manager.call_async(req),25).success:raise ValueError('search_navigation_lifecycle_failed')
                found=False
                points=backend.environment['rooms']['bedroom']['search_points'][:2]
                for point_index,point in enumerate(points):
                    if point_index:
                        stage('navigate_to_bedroom_backup');backend.stage='navigate_source'
                        backend.navigate('bedroom','bedroom',[point[k] for k in ('x','y','yaw')],opening,'backup-navigation')
                    runtime.face_source((0.,-.35));lifecycle(0);configured=True
                    spin=ActionClient(runtime.node,Spin,'/spin')
                    if not spin.wait_for_server(timeout_sec=5):raise ValueError('existing_nav2_spin_unavailable')
                    try:
                        for direction in range(8):
                            stage(f'observe_point_{point_index}_direction_{direction}');rows=[];captures=[]
                            for frame in range(3):
                                sample=runtime.sample(processing_reserve_ns=0);path=runtime.save(sample,f'p{point_index}-d{direction}-f{frame}');inference=worker.infer(path,pump=runtime.pump);write_json(path/'detections.json',inference);captures.append(str(path))
                                for detection in inference.get('detections',[]):
                                    if detection['name']!='cigarette' or detection['confidence']<.2:continue
                                    try:
                                        item=candidate(sample,detection,backend.environment);item.update(capture=str(path),point_index=point_index,direction=direction);rows.append(item);report['candidates'].append(item)
                                    except ValueError as e:record(dict(event='candidate_rejected',capture=str(path),reason=str(e)))
                            view=dict(point_index=point_index,direction=direction,captures=captures,target_candidates=rows);report['views'].append(view);write_json(root/'result.json',report)
                            if len(rows)==3 and min(x['confidence'] for x in rows)>=.5 and len({x['stamp_ns'] for x in rows})==3:
                                xyz=np.asarray([x['surface_map_m'] for x in rows]);spread=float(np.linalg.norm(xyz-xyz.mean(0),axis=1).max())
                                if spread<.03:
                                    report.update(status='room_target_surface_found_for_near_confirmation',found_candidate=dict(rows[-1],surface_map_m=xyz.mean(0).tolist(),center_spread_m=spread),last_robot_state=runtime.state(),grasp_authorized=False);found=True;break
                            if direction==7:break
                            stage('turn_in_place_with_existing_nav2_spin');before=runtime.state();goal=Spin.Goal();goal.target_yaw=math.pi/4;goal.time_allowance.sec=15
                            active=wait(spin.send_goal_async(goal),5)
                            if not active.accepted:raise ValueError('search_spin_goal_rejected')
                            result=wait(active.get_result_async(),20);active=None
                            if result.status!=4:raise ValueError('search_spin_did_not_complete')
                            after=runtime.state()
                            if math.dist(before['poses']['map'][:2],after['poses']['map'][:2])>.05:raise ValueError('search_spin_translation_drift')
                            runtime.pump(.3)
                    finally:
                        if active is not None:
                            try:wait(active.cancel_goal_async(),5)
                            except Exception:pass
                            active=None
                        lifecycle(3);configured=False;spin.destroy()
                    if found:break
                if not found:report.update(status='search_views_completed_without_confirmed_target',last_robot_state=runtime.state(),room_absence_verified=False)
            except (Exception,KeyboardInterrupt) as e:
                report.update(status='stopped_requires_actual_state_review',reason=str(e) or 'interrupted')
                if runtime:
                    try:report['last_robot_state']=runtime.state()
                    except Exception:pass
            finally:
                if configured:
                    try:lifecycle(3)
                    except Exception as e:report['navigation_cleanup_error']=str(e)
                if worker:worker.close()
                if backend:backend.close()
                if runtime:runtime.shutdown();rclpy.shutdown()
                report['elapsed_s']=(time.time_ns()-start)/1e9;write_json(root/'result.json',report);print(json.dumps(report))
if __name__=='__main__':main()
