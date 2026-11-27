"""Use shared tuck/navigation/grasp with an explicitly supplied container adapter."""
import json
from container_transfer import drop_and_check,ContainerPolicy

class ContainerBindings:
    def __init__(self,stages,shared_transfer,adapter_factory):
        self.stages=stages;self.shared=shared_transfer;self.adapter_factory=adapter_factory
        self.result=None;self.started=False
    def tuck(self,effect,cancel):return self.shared.tuck(effect,cancel)
    def place(self,effect,cancel):
        if self.started:raise ValueError('container_drop_already_started')
        self.started=True;self.stages.check(cancel)
        adapter=self.adapter_factory(self.stages,self.stages.context)
        self.result=drop_and_check(adapter,self.stages.target,self.stages.destination_id,cancel=cancel,policy=ContainerPolicy(persistent_recovery=True,finish_on_release=True))
        (self.stages.output/'container-result.json').write_text(json.dumps(self.result,indent=2))
        return self.result
    def verify_placement(self,effect,cancel):
        self.stages.check(cancel)
        if not self.result or self.result.get('status')!='ok':
            return dict(status='incomplete',reason='container_verification_unavailable')
        # These observations were acquired after the release in this run.
        # This stage publishes no success itself; normal protocol lifecycle does.
        return self.result
    def handlers(self):return dict(tuck=self.tuck,place=self.place,verify_placement=self.verify_placement)
