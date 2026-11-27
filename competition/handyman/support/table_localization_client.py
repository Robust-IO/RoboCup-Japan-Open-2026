"""Stopped observation -> single-owner laser correction -> new observation."""
import json,time
from std_srvs.srv import Trigger
from table_localization import config

def localize(runtime,root,label):
    if not config()['enabled']:return dict(status='disabled_by_configuration')
    stopped=runtime.sample(processing_reserve_ns=0);runtime.fresh_measurement(stopped['snapshot']);runtime.ownership()
    client=runtime.node.create_client(Trigger,'/handyman/table_localization/check')
    try:
        if not client.wait_for_service(timeout_sec=5):raise ValueError('automatic_table_localization_service_unavailable')
        future=client.call_async(Trigger.Request());deadline=time.monotonic()+90
        while not future.done():
            if time.monotonic()>deadline:raise ValueError('table_localization_service_no_result')
            runtime.pump(.02)
        response=future.result();proof=json.loads(response.message)
        (root/(label+'-localization.json')).write_text(json.dumps(proof,indent=2))
        if not response.success:raise ValueError('table_localization_failed:'+proof.get('reason',proof.get('status','unknown')))
        # Localization changes map coordinates, not the physical pose. The
        # pre-service odom/joint proof must still hold before taking a new frame.
        runtime.fresh_measurement(stopped['snapshot']);runtime.invalidate_observations()
        runtime._table_localization_verified=True;runtime._live_table_geometry={}
        fresh=runtime.sample(processing_reserve_ns=0)
        runtime.record(dict(event='table_localization_verified',label=label,status=proof['status'],
            correction_applied=proof.get('correction_applied',False),new_depth_stamp_ns=fresh['snapshot']['streams']['depth']['stamp_ns']))
        return proof
    finally:runtime.node.destroy_client(client)
