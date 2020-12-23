#ifndef TRACKER_H
#define TRACKER_H


#include <pack_world_model/worldmodel/util/kalman.h>
#include <pack_util/geom/geom.h>

using namespace rcsc;

struct vraw {
    Vector2D pos; //in meters
    double angle; //in degrees
    double conf; //0-1
    double timestamp; //in seconds
};


class Tracker : public Kalman {
public:
    Tracker(int state_n, int obs_n, double _stepsize);
    virtual void reset() = 0;
    virtual void observe(vraw obs, double timestamp) = 0;
    virtual void observeNew(vraw obs, double _acc_x_sign, double _acc_y_sign) = 0;
    virtual Vector2D position(double time) = 0;
    virtual Vector2D velocity(double time) = 0;
    virtual Vector2D acceleration(double time) = 0;
    virtual double direction(double time) = 0;
    virtual double angular_velocity(double time) = 0;
};

void setFramePeriod(double framePeriod);
double getFramePeriod();
void setLatencyDelay(double latencyDelay);
double getLatencyDelay();

#endif // TRACKER_H
