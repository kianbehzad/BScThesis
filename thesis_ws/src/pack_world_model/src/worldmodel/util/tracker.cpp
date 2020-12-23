#include "pack_world_model/worldmodel/util/tracker.h"

double FramePeriod = 0.016;
double LatencyDelay = 0.04;

Tracker::Tracker(int state_n, int obs_n, double _stepsize)
    : Kalman(state_n, obs_n, _stepsize) {
    setFramePeriod(1.0 / 65.0);
    setLatencyDelay(0.08);
}



void setFramePeriod(double framePeriod) {
    FramePeriod = framePeriod;
}

double getFramePeriod() {
    return FramePeriod;
}

void setLatencyDelay(double latencyDelay) {
    LatencyDelay = latencyDelay;
}

double getLatencyDelay() {
    return LatencyDelay;
}
