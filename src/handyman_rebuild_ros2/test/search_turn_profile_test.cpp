#include <cassert>
#include <cmath>
#include "handyman_rebuild_ros2/search_turn_profile.hpp"
using namespace handyman_rebuild_ros2;
int main() {
  assert(std::abs(turnError(-3.0,3.0)-.28318530718)<1e-8);
  assert(turnSpeed(.02,.1,.05)==0);
  assert(turnSpeed(.8,0,.05)<=.040001);
  assert(turnSpeed(.8,.45,.05)<=.45);
  for (double bad : {0.,-.1,.51}) {
    bool caught=false;try {turnSpeed(.8,.1,bad);} catch (...) {caught=true;}assert(caught);
  }
  double left=M_PI/4,speed=0,time=0;
  while (left>.035 && time<10) {
    double next=turnSpeed(left,speed,.05);
    assert(next>=0 && next<=.45 && next<=speed+.040001);
    left-=next*.05;speed=next;time+=.05;
  }
  assert(left>=0 && left<=.035 && time<4.0);
}
