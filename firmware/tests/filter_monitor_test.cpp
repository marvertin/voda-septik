#include "filter_monitor.h"
#include <cassert>
#include <cmath>
#include <cstdio>
static int64_t run(FilterMonitor &m, int64_t t, float flow, float dp, int steps=41, bool noisy=false) {
    for(int i=0;i<steps;++i,t+=250000) {
        m.flow(flow,t);
        m.pressure(3+(noisy && i%10==0 ? 2 : dp),3,t);
    }
    return t;
}
int main() {
    FilterMonitor m;
    assert(!m.result().valid);
    auto t=run(m,1000000,24.99f,.4f);
    assert(!m.result().valid);
    t=run(m,t,25,.4f);
    assert(m.result().valid && m.result().measurement_id==1);
    assert(std::abs(m.result().pressure_diff_bar-.4f)<1e-5);
    t=run(m,t,35,.4f,80);
    assert(m.result().measurement_id>=2);
    auto id=m.result().measurement_id;
    t=run(m,t,0,0,80);
    assert(m.result().measurement_id==id && m.result().pressure_diff_bar>.39);
    t=run(m,t,35,0,41);
    assert(m.result().measurement_id==id+1 && m.result().pressure_diff_bar==0);
    FilterMonitor negative;
    run(negative,1000000,35,-.02f);
    assert(negative.result().pressure_diff_bar<0);
    FilterMonitor spikes;
    run(spikes,1000000,35,.4f,41,true);
    // 5 spikes exceed the four trimmed samples: reject instead of publishing a false result.
    assert(!spikes.result().valid);
    FilterMonitor trimmed;
    for (int i=0;i<41;++i) {
        auto ts=1000000+i*250000LL; trimmed.flow(35,ts);
        trimmed.pressure(3.4f + (i==15 ? 3.0f : 0.0f),3,ts);
    }
    assert(trimmed.result().valid && std::abs(trimmed.result().pressure_diff_bar-.4f)<1e-5);
    assert(trimmed.result().sample_count==33);
    FilterMonitor common_transient;
    for (int i=0;i<41;++i) {
        auto ts=1000000+i*250000LL; common_transient.flow(35,ts);
        common_transient.pressure(3.4f+i*.03f,3+i*.03f,ts);
    }
    assert(!common_transient.result().valid);
    FilterMonitor stale_flow;
    stale_flow.flow(35,1000000);
    for(int i=0;i<41;++i) stale_flow.pressure(3.4,3,1000000+i*250000LL);
    assert(!stale_flow.result().valid);
    FilterMonitor gap;
    t=run(gap,1000000,35,.4f,20);
    gap.tick(t+2000000);
    run(gap,t+2000000,35,.4f,20);
    assert(!gap.result().valid);
    FilterMonitor invalid;
    t=run(invalid,1000000,35,.4f,20);
    invalid.flow(35,t); invalid.pressure(NAN,3,t);
    run(invalid,t+250000,35,.4f,20);
    assert(!invalid.result().valid);
    FilterMonitor drift;
    for(int i=0;i<41;++i) { auto ts=1000000+i*250000LL; drift.flow(i%2?26:50,ts); drift.pressure(3.4,3,ts); }
    assert(!drift.result().valid);
    FilterMonitor restart;
    assert(!restart.result().valid && restart.result().measurement_id==0);
    FilterConfig c; c.offset_bar=.1;
    FilterMonitor offset(c); run(offset,1000000,35,.4);
    assert(std::abs(offset.result().pressure_diff_bar-.3)<1e-5);
    puts("filter monitor: threshold, continuous windows, hold, cleaning, signed dp, spikes, stale/invalid data, instability, restart, offset OK");
}
