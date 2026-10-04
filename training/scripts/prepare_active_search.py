"""Prepare an isolated ring configuration. Offline only, no motion authorization.

The package snapshot keeps existing request/worker point equality checks intact.
Never replace competition YAML to test generated points.
"""
import argparse
import hashlib
import json
from pathlib import Path
import uuid
import yaml

from build_active_search_points import build
from search_request_consumer import prepare, beneath

TARGETS = ('apple', 'canned_juice', 'rabbit_doll', 'pink_cup', 'white_cup', 'filled_ketchup')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def snapshot(repo, output, layout, room, target, report, supplement_points=0):
    repo=repo.resolve();output=output.resolve()
    if target not in TARGETS:
        raise ValueError('target_not_supported_by_current_model')
    source=repo/'src/handyman_rebuild_ros2'
    if output.is_relative_to(source) or source.is_relative_to(output):
        raise ValueError('output_overlaps_source_package')
    layout_row=next((r for r in report['reports'] if r['layout']==layout),None)
    if layout_row is None:
        raise ValueError('unknown_layout')
    room_row=next((r for r in layout_row['rooms'] if r['room']==room),None)
    if room_row is None or room_row['status']!='proposed' or len(room_row['views'])!=8:
        raise ValueError('room_requires_review_no_ring')
    catalog_path=source/'config/environments.yaml'
    catalog=yaml.safe_load(catalog_path.read_bytes())
    env_path=beneath(source/'config',catalog['environment_files'][layout])
    env_bytes=env_path.read_bytes();environment=yaml.safe_load(env_bytes)
    from search_supplement_plan import supplement_views
    extra_views=supplement_views(room_row,{n:r['region'] for n,r in environment['rooms'].items()},room,supplement_points)
    if digest(env_bytes)!=layout_row['environment_sha256']:
        raise ValueError('environment_changed_since_planning')
    prefix='package://handyman_rebuild_ros2/'
    if not environment['map'].startswith(prefix):
        raise ValueError('invalid_map_reference')
    map_path=beneath(source,environment['map'][len(prefix):])
    meta=map_path.read_bytes();image_name=yaml.safe_load(meta)['image']
    image_path=beneath(source,str((map_path.parent/image_name).relative_to(source)))
    if digest(meta+b'\0'+image_path.read_bytes())!=layout_row['map_bundle_sha256']:
        raise ValueError('map_changed_since_planning')
    param_path=repo/'src/handyman_ros2/param/nav2_params.yaml'
    if digest(param_path.read_bytes())!=report['footprint_config_sha256']:
        raise ValueError('footprint_changed_since_planning')
    # Copy only config/maps; record every input so changes during copying abort.
    inputs={p:p.read_bytes() for folder in ('config','maps')
            for p in (source/folder).rglob('*') if p.is_file()}
    for p in inputs:
        if not p.resolve().is_relative_to(source):
            raise ValueError('source_resource_outside_package')
    if (inputs.get(env_path)!=env_bytes or inputs.get(map_path)!=meta
            or digest(meta+b'\0'+inputs.get(image_path,b''))!=layout_row['map_bundle_sha256']):
        raise ValueError('source_changed_during_capture')
    output.mkdir(parents=True,exist_ok=False)
    share=output/'package'
    for p,data in inputs.items():
        dest=share/p.relative_to(source);dest.parent.mkdir(parents=True,exist_ok=True)
        dest.write_bytes(data)
    points=[dict(id=f'{layout}/{room}/{i}',pose={k:v[k] for k in ('x','y','yaw')})
            for i,v in enumerate(room_row['views']+extra_views)]
    environment['rooms'][room]['search_points']=[p['pose'] for p in points]
    (share/env_path.relative_to(source)).write_text(yaml.safe_dump(environment))
    task=uuid.uuid4().hex
    request=dict(schema='handyman-search-request-v1',task_id=task,environment=layout,layout=layout,
                 room=room,target=target,map_uri=environment['map'],frame_id='map',points=points,
                 actionable=False,requires_map_verification=True)
    prepared=prepare(request,share)
    if any(p.read_bytes()!=data for p,data in inputs.items()) or digest(param_path.read_bytes())!=report['footprint_config_sha256']:
        raise ValueError('source_changed_during_copy')
    files={str(p.relative_to(output)):digest(p.read_bytes()) for p in share.rglob('*') if p.is_file()}
    manifest=dict(schema='handyman-active-search-bundle-v1',task_id=task,layout=layout,room=room,target=target,
                  request=request,package_share=str(share),files=files,
                  map_bundle_sha256=prepared['map_bundle_sha256'],
                  source_environment_sha256=layout_row['environment_sha256'],
                  footprint_config_sha256=report['footprint_config_sha256'],
                  source_repo=str(repo),main=points[0]['pose'],
                  supplement=dict(requested_points=supplement_points,actual_points=len(extra_views)//8,
                                  strategy='hidden_region_interior_2d',coverage_verified=False,
                                  proposed_backups=room_row.get('backups',[])),
                  actionable=False,live_path_verified=False,does_not_exist_authorized=False,
                  blockers=['Current TF/map identity and live planner preflight required.',
                            'Live ring execution remains disabled; this is not a motion permit.'])
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False))
    return manifest


def validate_bundle(path):
    path=path.resolve();m=json.loads(path.read_text());root=path.parent
    if m.get('schema')!='handyman-active-search-bundle-v1' or m.get('actionable') is not False:
        raise ValueError('invalid_bundle_schema_or_authority')
    for name,expected in m['files'].items():
        p=beneath(root,name)
        if digest(p.read_bytes())!=expected:
            raise ValueError('bundle_file_changed')
    share=(root/'package').resolve()
    if Path(m['package_share']).resolve()!=share:
        raise ValueError('package_share_mismatch')
    for k in ('task_id','layout','room','target'):
        if m[k]!=m['request'][k]:
            raise ValueError('request_identity_mismatch')
    prepared=prepare(m['request'],share)
    if prepared['map_bundle_sha256']!=m['map_bundle_sha256'] or m['main']!=m['request']['points'][0]['pose']:
        raise ValueError('bundle_map_or_main_mismatch')
    return m


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repo',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--layout',required=True);p.add_argument('--room',required=True)
    p.add_argument('--target',choices=TARGETS,required=True)
    p.add_argument('--supplement-points',type=int,choices=(0,1,2),default=0)
    a=p.parse_args();m=snapshot(a.repo,a.output,a.layout,a.room,a.target,build(a.repo),a.supplement_points)
    validate_bundle(a.output/'manifest.json')
    print(json.dumps(dict(manifest=str(a.output/'manifest.json'),task_id=m['task_id'],main=m['main'],
                          views=len(m['request']['points']),motion_started=False,live_path_verified=False)))


if __name__=='__main__':main()
