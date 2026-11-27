#include "startup_clock_policy.hpp"
#include <cassert>
using namespace startup_clock;
int main(){std::vector<Sample> s;for(int i=0;i<30;++i)s.push_back({i*.1,160.+i%3,1000000000LL+i*100000000LL,{0.,0.}});
auto d=assess(s,3.);assert(d.usable && d.correction && d.correction_ms==177);
for(auto& x:s)x.ahead_ms=-20.;d=assess(s,3.);assert(d.usable&&!d.correction);
for(auto& x:s)x.ahead_ms=-800.;assert(!assess(s,3.).usable);
for(auto& x:s)x.ahead_ms=160.;s.back().joints[0]=.01;assert(!assess(s,3.).usable);
s.back().joints[0]=0.;s.back().stamp_ns=s.front().stamp_ns;assert(!assess(s,3.).usable);
s.back().stamp_ns=4000000000LL;assert(!assess(s,4.).usable);
s.clear();for(int i=0;i<30;++i)s.push_back({i*.1,3.+38.*i/29.,1000000000LL+i*100000000LL,{0.,0.}});
d=assess(s,3.);assert(d.usable&&d.correction&&d.correction_ms==56);
for(auto& x:s)x.ahead_ms-=d.correction_ms;assert(assess(s,3.).usable&&!assess(s,3.).correction);
for(int i=0;i<30;++i)s[i].ahead_ms=i*5.;assert(!assess(s,3.).usable);
for(auto& x:s)x.ahead_ms=20.;s.back().ahead_ms=150.;assert(!assess(s,3.).usable);
}
