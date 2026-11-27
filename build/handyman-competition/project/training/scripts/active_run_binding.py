"""Bind one runtime request to an immutable offline bundle; no motion permit."""
import hashlib
import json
from pathlib import Path
from prepare_active_search import validate_bundle


class ActiveRunBinding:
    def __init__(self, manifest, share):
        self.path=Path(manifest).resolve()
        self.fingerprint=hashlib.sha256(self.path.read_bytes()).hexdigest()
        self.bundle=validate_bundle(self.path)
        if Path(share).resolve()!=Path(self.bundle['package_share']).resolve():
            raise ValueError('active_bundle_package_mismatch')
        self.used=False
        self.check()

    def check(self):
        if hashlib.sha256(self.path.read_bytes()).hexdigest()!=self.fingerprint:
            raise ValueError('active_manifest_changed')
        current=validate_bundle(self.path)
        params=Path(current['source_repo'])/'src/handyman_ros2/param/nav2_params.yaml'
        if hashlib.sha256(params.read_bytes()).hexdigest()!=current['footprint_config_sha256']:
            raise ValueError('active_footprint_changed')

    def accept(self, request, journal=None):
        if self.used:raise ValueError('active_bundle_request_already_used')
        self.check()
        expected=self.bundle['request']
        # Only the fresh transport timestamp may differ from the saved request.
        body={k:v for k,v in request.items() if k!='stamp_ns'}
        if body!=expected:raise ValueError('active_bundle_request_mismatch')
        if journal is not None:journal.claim_active_request(self.bundle['task_id'],self.fingerprint)
        self.used=True

    def result_matches(self,row):
        if not isinstance(row,dict):return False
        context=row.get('observer_context',{})
        return (isinstance(context,dict) and row.get('task_id')==self.bundle['task_id']
                and row.get('target')==self.bundle['target']
                and context.get('task_id')==self.bundle['task_id']
                and context.get('map_sha256')==self.bundle['map_bundle_sha256']
                and context.get('point_id') in {p['id'] for p in self.bundle['request']['points']}
                and row.get('terminal') is True and row.get('cleanup_verified') is True
                and row.get('actionable') is False and row.get('does_not_exist_authorized') is False)
