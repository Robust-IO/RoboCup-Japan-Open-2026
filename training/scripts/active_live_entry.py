"""Explicit live ring gate; online map gate remains authoritative in runtime."""
import json
import subprocess
import sys
from pathlib import Path

PREFIX='/handyman/manual_single_search'


def validate_live_ring(args, env):
    if not (args.allow_live and args.multi_point and args.active_bundle
            and args.point_index==0 and args.head_tilt is not None
            and args.session_budget_seconds is not None and args.view_seconds is not None):
        raise ValueError('live_ring_requires_bound_plan_head_and_session_budget')
    if env.get('ROS_DOMAIN_ID')!='71' or env.get('ROS_LOCALHOST_ONLY')!='1':
        raise ValueError('live_ring_requires_domain71_localhost')
    expected=dict(action_name='/navigate_to_pose',map_topic='/map',
                  vision_topic='/handyman/vision/diagnostics',
                  request_topic=PREFIX+'/request',status_topic=PREFIX+'/status',
                  result_topic=PREFIX+'/result',private_prefix=PREFIX+'/runtime')
    if any(getattr(args,k)!=v for k,v in expected.items()):
        raise ValueError('live_ring_requires_private_topics_and_standard_navigation')


def check_plan_result(report,bundle):
    if (report.get('success') is not True or report.get('action_status')!=4
            or report.get('task_id')!=bundle['task_id']
            or report.get('map_bundle_sha256')!=bundle['map_bundle_sha256']
            or report.get('target')!=bundle['main']
            or report.get('start_source')!='current_robot_tf'
            or report.get('robot_control_started') is not False):
        raise ValueError('live_ring_current_path_preflight_failed')


def plan_current_path(binding,output):
    binding.check()
    subprocess.run([sys.executable,str(Path(__file__).with_name('probe_search_path.py')),
                    '--active-bundle',str(binding.path),'--output',str(output)],
                   check=True,timeout=25,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    report=json.loads(output.read_text())
    check_plan_result(report,binding.bundle)
    binding.check()
    return report
