#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
namespace startup_clock {
struct Sample { double received_s; double ahead_ms; int64_t stamp_ns; std::vector<double> joints; };
struct Decision { bool usable; bool correction; int64_t correction_ms; std::string reason; };
inline Decision assess(const std::vector<Sample>& s,double now) {
  if(s.size()<20 || s.back().received_s-s.front().received_s<2.)return {false,false,0,"collecting_fresh_joint_samples"};
  if(now-s.back().received_s>.5)return {false,false,0,"joint_feedback_not_arriving"};
  std::vector<double> errors;
  for(std::size_t i=0;i<s.size();++i){
    if(!std::isfinite(s[i].ahead_ms) || s[i].stamp_ns<=0 || (i && s[i].stamp_ns<=s[i-1].stamp_ns))return {false,false,0,"joint_timestamp_not_advancing"};
    if(s[i].joints.empty() || s[i].joints.size()!=s[0].joints.size())return {false,false,0,"invalid_joint_layout"};
    for(std::size_t j=0;j<s[i].joints.size();++j)
      if(!std::isfinite(s[i].joints[j]) || std::abs(s[i].joints[j]-s[0].joints[j])>.005)return {false,false,0,"robot_not_stationary"};
    errors.push_back(s[i].ahead_ms);
  }
  std::sort(errors.begin(),errors.end());
  double lo=errors[errors.size()/10],hi=errors[errors.size()*9/10],median=errors[errors.size()/2];
  if(hi-lo>50. || errors.back()-errors.front()>100.)return {false,false,0,"unstable_clock_or_transport_delay"};
  if(median>=-100. && hi<=0.)return {true,false,0,"clock_ready"};
  // A packet in the future proves a clock mismatch; a late packet alone does
  // NOT prove one. Never erase genuine network delay by making old data fresh.
  if(median<=0.)return {false,false,0,"late_data_not_evidence_of_clock_offset"};
  if(median>2000.)return {false,false,0,"clock_offset_above_automatic_range"};
  return {true,true,static_cast<int64_t>(std::ceil(errors.back()+15.)),"correct_future_source_clock"};
}
}
