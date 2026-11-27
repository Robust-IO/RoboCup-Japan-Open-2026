"""Reuse the existing ring geometry, navigation, model worker and RGBD runtime."""
import json,math
import numpy as np
from perception_bridge import inside
class Session3Search:
 def __init__(self,bundle):
  self.b=bundle;self.rt=bundle['runtime'];self.motion=bundle['motion_backend'];self.backend=bundle['grasp_backend'];self.output=bundle['output'];self.cfg=bundle['config'];self.room=self.cfg.get('task',{}).get('source_room','lobby')
  self.options=dict(self.cfg['session_profile']['search']);self.options.update(head_tilts=[0.,-math.pi/6],aspect_ratio_threshold=1.35)
  from session3_search_policy import aspect_ratio
  ratio,_=aspect_ratio(bundle['environment']['rooms'][self.room]['region'])
  self.points=self.cfg['search_plan']['points'][:1] if ratio<=1.35 else self.cfg['search_plan']['points'];self.serial=0
  if ratio>1.35 and len(self.points)<2:raise ValueError('regenerate_search_plan_for_1_35_threshold')
 def check(self,cancel):
  if cancel.is_set():raise RuntimeError('search_cancelled')
 def navigate(self,point,cancel,label):
  self.check(cancel);self.rt.sample(profile='state');state=self.rt.state()
  source=self.motion.room_at(state['poses']['map'])
  result=self.motion.navigate(source,self.room,[point[k] for k in ('x','y','yaw')],state['joints'][5],label)
  self.check(cancel)
  if result.get('verified') is not True:raise ValueError('search_navigation_not_arrived')
  self.rt.sample(profile='state')
  if self.motion.room_at(self.rt.state()['poses']['map'])!=self.room:raise ValueError('search_room_not_reached')
 def navigate_room(self,effect,cancel):
  self.check(cancel);self.rt.sample(profile='state')
  from room_entry import entered,finish_on_room_entry
  if not entered(self.rt.state()['poses']['map'],{'polygon':self.b['environment']['rooms'][self.room]['region']}):
   with finish_on_room_entry(self.motion,self.output,self.room,self.b['environment']['rooms'][self.room]['region']):
    self.navigate(self.points[0],cancel,'session3-room-entry-'+str(self.serial))
  return dict(status='ok',arrived=True,room=getattr(self,'room','lobby'))
 def capture(self,target,cancel,label):
  self.check(cancel);item=self.backend._sample();s=item['snapshot'];stamp=s['streams']['depth']['stamp_ns']
  path=self.rt.save(item,label);raw=self.b['worker'].infer(path,pump=lambda:self.rt.pump(0));self.check(cancel)
  return self.evaluate(item,path,raw,target)
 def evaluate(self,item,path,raw,target):
  s=item['snapshot'];stamp=s['streams']['depth']['stamp_ns']
  points,_,indices=self.backend._cloud(item)
  usable=len(points)>=100 and len(points)>=.1*item['depth'].size
  from apple_transfer_auto_plan import map_point
  origin=np.asarray(map_point(s,np.zeros(3)));basis=np.stack([np.asarray(map_point(s,v))-origin for v in np.eye(3)],axis=1)
  mapped=points@basis.T+origin;v,u=np.unravel_index(indices,item['depth'].shape)
  polygon=self.b['environment']['rooms'][self.room]['region'];matches=[];candidates=[];unresolved=False;outside=[]
  for d in raw.get('detections',[]):
   if d['name']!=target or d['confidence']<self.options['candidate_confidence']:continue
   x1,y1,x2,y2=d['box'];a=mapped[(u>=x1)&(u<x2)&(v>=y1)&(v<y2)]
   if len(a)<30:unresolved=True;continue
   # Use the central image region to reduce background at box boundaries.
   dx=(x2-x1)*.2;dy=(y2-y1)*.2
   core=mapped[(u>=x1+dx)&(u<x2-dx)&(v>=y1+dy)&(v<y2-dy)]
   if len(core)<30:unresolved=True;continue
   center=np.median(core,axis=0)
   if not inside(*center[:2],polygon):outside.append(d);continue
   candidates.append(dict(position_map_m=center.tolist(),confidence=d['confidence'],box=d['box']))
   if d['confidence']>=self.options['confirmed_confidence']:
    matches.append(dict(position_map_m=center.tolist(),confidence=d['confidence'],box=d['box']))
   else:unresolved=True
  row=dict(target=target,room=getattr(self,'room','lobby'),stamp_ns=stamp,usable=usable,matches=matches,candidates=candidates,unresolved=unresolved,excluded_room_candidates=outside,evidence_ref=str(path))
  (path/'search-observation.json').write_text(json.dumps(row));(path/'detections.json').write_text(json.dumps(raw))
  return row
 def search(self,effect,cancel):
  return self.continuous_search(effect,cancel)
 def stopped_search(self,effect,cancel):
  from active_search_policy import ring_views
  target=effect['task']['target'];self.serial+=1;rows=[];valid=0;unresolved=0;last_stamp=0
  required=len(self.points)*8*len(self.options['head_tilts'])
  report=self.output/f'search-{self.serial}-{target}.json'
  def finish(**kw):
   result=dict(status='ok',target=target,room=getattr(self,'room','lobby'),valid_views=valid,required_views=required,unresolved_candidates=unresolved,evidence_ref=str(report),**kw)
   report.write_text(json.dumps(dict(result=result,observations=rows,plan=self.cfg['search_plan']),indent=2));return result
  for pi,point in enumerate(self.points):
   for vi,view in enumerate(ring_views(point['x'],point['y'],point['yaw'])):
    self.navigate(view,cancel,f'session3-search-{self.serial}-{pi}-{vi}')
    for ti,tilt in enumerate(self.options['head_tilts']):
     self.check(cancel);self.rt.face_source((0.,tilt));frames=[]
     # One local retry for incomplete or tentative observations, no repeated room tour.
     for attempt in range(2):
      frames=[]
      for fi in range(self.options['frames_per_view']):
       row=self.capture(target,cancel,f's3-{self.serial}-{pi}-{vi}-{ti}-{attempt}-{fi}')
       row['usable']=row['usable'] and row['stamp_ns']>last_stamp;last_stamp=max(last_stamp,row['stamp_ns']);rows.append(row);frames.append(row)
       self.rt.pump(.2);self.check(cancel)
      if all(r['usable'] for r in frames):
       for a in frames[0]['matches']:
        for z in frames[-1]['matches']:
         if np.linalg.norm(np.asarray(a['position_map_m'])-z['position_map_m'])<=.08:
          return finish(confirmed=True,position_map_m=z['position_map_m'],search_exhausted=False)
       if not any(r['unresolved'] or r['matches'] for r in frames):break
     if all(r['usable'] for r in frames):valid+=1
     if any(r['unresolved'] or r['matches'] for r in frames):unresolved+=1
  return finish(confirmed=False,search_exhausted=valid==required and unresolved==0,
                absence_basis='completed_configured_main_backup_views',absolute_visibility_proven=False)

 def near_candidate(self,candidate,target,cancel,label):
  from search_near_view import choose_near_view
  self.rt.sample(profile='state');pose=self.rt.state()['poses']['map']
  selector=getattr(self.motion,'candidate_selector',None)
  if selector is None:selector=self.b.get('candidate_selector')
  if selector is None:
   from candidate_selection import load_selector
   selector=load_selector(self.cfg,self.b['semantic'])
  point=choose_near_view(pose,candidate['position_map_m'],self.b['environment']['rooms'][self.room]['region'],selector)
  if point is None:return dict(confirmed=False,resolved=False,reason='no_reachable_near_view')
  self.navigate(point,cancel,label+'-near')
  self.rt.sample(profile='state');pose=self.rt.state()['poses']['map']
  xyz=candidate['position_map_m'];distance=math.dist(pose[:2],xyz[:2])
  # Center the candidate using camera-height estimate, then confirm in fresh RGBD.
  tilt=max(-.8,min(0.,math.atan2(xyz[2]-1.0,max(distance,.1))))
  self.rt.face_source((0.,tilt))
  frames=[self.capture(target,cancel,label+'-'+str(k)) for k in range(2)]
  if not all(r['usable'] for r in frames) or frames[1]['stamp_ns']<=frames[0]['stamp_ns']:return dict(confirmed=False,resolved=False,reason='near_view_invalid')
  for a in frames[0]['matches']:
   for z in frames[1]['matches']:
    if np.linalg.norm(np.asarray(a['position_map_m'])-z['position_map_m'])<=.08:return dict(z,confirmed=True,resolved=True)
  return dict(confirmed=False,resolved=not any(r['unresolved'] or r['matches'] for r in frames),reason='near_view_not_confirmed')

 def continuous_search(self,effect,cancel):
  from continuous_scan import ContinuousSweep,wrap
  target=effect['task']['target'];self.serial+=1;rows=[];valid=0;unresolved=0
  count=int(self.options.get('continuous_bins',12));required=len(self.points)*count*2
  report=self.output/f'search-{self.serial}-{target}.json'
  def finish(**kw):
   result=dict(status='ok',target=target,room=getattr(self,'room','lobby'),valid_views=valid,required_views=required,unresolved_candidates=unresolved,evidence_ref=str(report),**kw)
   report.write_text(json.dumps(dict(result=result,observations=rows,plan=self.cfg['search_plan'],mode='continuous_two_tilts'),indent=2));return result
  for pi,point in enumerate(self.points):
   self.navigate(point,cancel,f'continuous-point-{self.serial}-{pi}')
   for ti,tilt in enumerate((0.,-math.pi/6)):
    self.check(cancel);self.rt.face_source((0.,tilt))
    by_bin={};start=None;remaining=2*math.pi-.03;attempted=[];segment=0
    while remaining>.03:
     segment+=1
     start,moving,travel=ContinuousSweep(self).run(target,cancel,f's3-{self.serial}-{pi}-{ti}-{segment}',scan_start=start,remaining_angle=remaining,skip_bins=by_bin.keys())
     remaining=max(0.,remaining-travel);rows.extend(moving)
     self.rt.sample(profile='state');resume=list(self.rt.state()['poses']['map'])
     for row in moving:
      by_bin[row['angle_bin']]=row
      for candidate in row.get('candidates',[]):
       xyz=np.asarray(candidate['position_map_m'])
       if any(np.linalg.norm(xyz-old)<.15 for old in attempted):continue
       attempted.append(xyz)
       confirmed=self.near_candidate(candidate,target,cancel,f'near-{self.serial}-{pi}-{ti}-{len(attempted)}')
       if confirmed['confirmed']:return finish(confirmed=True,position_map_m=confirmed['position_map_m'],search_exhausted=False)
       # A rejected near-view candidate does not invalidate already covered bins.
       row['matches']=[];row['unresolved']=not confirmed['resolved']
       self.navigate(dict(x=resume[0],y=resume[1],yaw=resume[2]),cancel,f'resume-scan-{self.serial}-{pi}-{ti}-{len(attempted)}')
       self.rt.face_source((0.,tilt))
     if not moving and travel<.01:raise ValueError('continuous_search_no_new_observation')
    # Repair only missing or invalid angular bins, never repeat a complete circle.
    for idx in range(count):
     if idx in by_bin and by_bin[idx]['usable'] and not by_bin[idx]['unresolved']:continue
     self.navigate(dict(x=point['x'],y=point['y'],yaw=wrap(start+idx*2*math.pi/count)),cancel,f'scan-repair-{self.serial}-{pi}-{ti}-{idx}')
     self.rt.face_source((0.,tilt))
     frame=self.capture(target,cancel,f'repair-{self.serial}-{pi}-{ti}-{idx}');rows.append(frame)
     for candidate in frame.get('candidates',[]):
      confirmed=self.near_candidate(candidate,target,cancel,f'repair-near-{self.serial}-{pi}-{ti}-{idx}')
      if confirmed['confirmed']:return finish(confirmed=True,position_map_m=confirmed['position_map_m'],search_exhausted=False)
      frame['matches']=[];frame['unresolved']=not confirmed['resolved']
     by_bin[idx]=frame
    valid+=sum(bool(r['usable']) for r in by_bin.values())
    unresolved+=sum(bool(r['unresolved'] or r['matches']) for r in by_bin.values())
  return finish(confirmed=False,search_exhausted=valid==required and unresolved==0,
    absence_basis='main_then_backup_level_and_minus30_completed',absolute_visibility_proven=False)
