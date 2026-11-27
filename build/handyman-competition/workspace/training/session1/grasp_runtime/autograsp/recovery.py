"""Bounded recovery decisions; uncertain holding never authorizes opening."""
from dataclasses import dataclass,field
from enum import Enum

class Failure(str,Enum):
 MISALIGNED='misaligned'
 EMPTY_CLOSE='empty_close'
 APPROACH_COLLISION='approach_collision'
 OCCLUDED='occluded'
 NO_CANDIDATE='no_candidate'
 SLIPPED='slipped'
 HOLD_UNCERTAIN='hold_uncertain'
 SENSOR_UNAVAILABLE='sensor_unavailable'

@dataclass
class Recovery:
 maximum_corrections:int=5
 corrections:int=0
 excluded:set=field(default_factory=set)
 history:list=field(default_factory=list)
 alignment_retries:dict=field(default_factory=dict)
 def decide(self,failure,candidate_id=None,*,holding='empty',fresh_observation=False):
  failure=Failure(failure)
  if holding not in ('empty','held','uncertain'):raise ValueError('invalid_holding_state')
  if self.corrections>=self.maximum_corrections:
   result=dict(action='stop_with_evidence',reason='correction_budget_exhausted',open_hand=False)
  elif holding in ('held','uncertain'):
   self.corrections+=1
   result=dict(action='inspect_hold_without_opening',reason=failure.value,open_hand=False)
  else:
   self.corrections+=1
   if failure==Failure.MISALIGNED and self.alignment_retries.get(candidate_id,0)<2:
    self.alignment_retries[candidate_id]=self.alignment_retries.get(candidate_id,0)+1
    result=dict(action='reobserve_then_correct_pose',open_hand=False)
   elif failure==Failure.SENSOR_UNAVAILABLE:
    result=dict(action='recover_observation_only',open_hand=False)
   else:
    if candidate_id and failure not in (Failure.NO_CANDIDATE,Failure.OCCLUDED):self.excluded.add(candidate_id)
    result=dict(action='withdraw_empty_reobserve_replan' if failure not in (Failure.OCCLUDED,Failure.NO_CANDIDATE) else 'change_observation_replan',
      open_hand=failure in (Failure.EMPTY_CLOSE,Failure.SLIPPED),retry_same_failed_candidate=False)
  result.update(corrections=self.corrections,maximum_corrections=self.maximum_corrections,requires_fresh_observation=True,
      candidate_id=candidate_id,failure=failure.value,excluded_candidates=sorted(self.excluded))
  self.history.append(result.copy());return result
