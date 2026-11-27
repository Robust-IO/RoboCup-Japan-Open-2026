"""Object-parameterized OFFLINE coordinator. Never imports a ROS motion adapter.

    The r19 apple entry remains the sole currently verified live entry. Synthetic
    success here is not evidence of moving a cube, nor permission to do so.
"""
import argparse
import json
from pathlib import Path
import re
import time

from apple_transfer_automation import STAGES,write_json
from manipulation_profiles import transfer_config,validate_transfer_config


def plan(name):
    config=transfer_config(name)
    return dict(status='offline_template_plan',config=config,stages=list(STAGES),
                adapter=config['object_profile']['geometry'],
                legacy_apple_entry_preserved=True,navigation_parameters_changed=False,
                robot_commands_sent=False,live_motion_supported=False,
                remaining=[name+'_'+stage for stage in ('current_rgbd_validation','near_tracking','contact_and_lift_adapter',
                           'held_surface_tracking','release_and_placement_validation')])


class OfflineBackend:
    mode='offline_test'
    def bind(self,output,task_id,config,record):
        self.output,self.task_id,self.config,self.record=output,task_id,config,record
    def perform(self,stage):
        path=self.output/(stage+'.json')
        proof=dict(task_id=self.task_id,class_name=self.config['task']['object'],stage=stage,
                   synthetic=True,verified=True,physical_action_executed=False)
        write_json(path,proof)
        return dict(verified=True,evidence=str(path))
    def close(self):pass


def run_offline(config,task_id,output,backend):
    validate_transfer_config(config)
    if getattr(backend,'mode',None)!='offline_test':raise ValueError('generic_live_adapter_not_enabled')
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,100}',task_id):raise ValueError('invalid_task_id')
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    report=dict(status='running_offline',task_id=task_id,class_name=config['task']['object'],
                confirmed_stages=[],physical_transfer_verified=False,robot_commands_sent=False,
                task_finished_sent=False,does_not_exist_sent=False,automatic_retry=False,automatic_resume=False)
    with (output/'journal.jsonl').open('x') as stream:
        def record(value):
            stream.write(json.dumps(dict(time_ns=time.time_ns(),**value),allow_nan=False)+'\n');stream.flush()
        try:
            backend.bind(output,task_id,config,record)
            write_json(output/'config.snapshot.json',config)
            for stage in STAGES:
                report['stage']=stage;record(dict(event='intent',stage=stage))
                response=backend.perform(stage)
                if not isinstance(response,dict) or response.get('verified') is not True:
                    raise ValueError('stage_not_verified:'+stage)
                evidence=Path(response['evidence']).resolve()
                if not evidence.is_relative_to(output.resolve()):raise ValueError('evidence_outside_current_run')
                proof=json.loads(evidence.read_text())
                if (proof.get('task_id')!=task_id or proof.get('class_name')!=config['task']['object']
                        or proof.get('stage')!=stage or proof.get('synthetic') is not True
                        or proof.get('verified') is not True or proof.get('physical_action_executed') is not False):
                    raise ValueError('offline_evidence_context_mismatch')
                report['confirmed_stages'].append(stage);record(dict(event='confirmed',stage=stage,evidence=str(evidence)))
            report['status']='offline_flow_verified_not_physical_transfer'
        except (Exception,KeyboardInterrupt) as exc:
            report.update(status='stopped_offline',reason=str(exc) or 'interrupted')
            record(dict(event='stopped',stage=report.get('stage'),reason=report['reason']))
        finally:
            try:backend.close()
            except Exception as exc:report.update(status='stopped_offline',cleanup_error=str(exc))
            write_json(output/'result.json',report)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--object',required=True,choices=('apple','rubiks_cube','canned_juice','ground_pepper'))
    parser.add_argument('--offline-smoke-test',action='store_true')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.offline_smoke_test:
        if args.output is None:parser.error('offline smoke test requires a NEW output directory')
        result=run_offline(transfer_config(args.object),'offline-'+args.object,args.output,OfflineBackend())
    else:result=plan(args.object)
    print(json.dumps(result,ensure_ascii=False))
    if result['status']=='stopped_offline':raise SystemExit(1)


if __name__=='__main__':main()
