#ifndef ROBOTTRACKER_H
#define ROBOTTRACKER_H

#include <pack_world_model/worldmodel/util/tracker.h>

class RobotTracker : public Tracker {
private:
    double latency;

    bool reset_on_obs;

    struct rcommand {
        double timestamp;
        Vector2D v;
        double w;
    };

    std::deque<rcommand> cs; // Velocity commands

    rcommand get_command(double time);

    bool our_robot;

    double stuck;

    double boundTo(double x, double low, double high);

protected:
    virtual Matrix& f(const Matrix &x, Matrix &I);// noiseless dynamics
    virtual Matrix& f2(const Matrix &x, Matrix &I , Matrix &U);// noiseless dynamics
    virtual Matrix& h(const Matrix &x); // noiseless observation

    virtual Matrix& Q(const Matrix &x); // Covariance of propagation noise
    virtual Matrix& R(const Matrix &x); // Covariance of observation noise

    virtual Matrix& A(const Matrix &x); // Jacobian of f w.r.t. x
    virtual Matrix& W(const Matrix &x); // Jacobian of f w.r.t. noise
    virtual Matrix& H(const Matrix &x); // Jacobian of h w.r.t. x
    virtual Matrix& V(const Matrix &x); // Jacobian of h w.r.t. noise

public:
    void command(double timestamp, Vector2D v, double w);

    RobotTracker(double _latency, bool _our_robot);
    virtual ~RobotTracker() {}

    virtual void reset() {
        reset_on_obs = true;
    }
    void reset(double timestamp, float *state);
    virtual void observe(vraw obs, double timestamp);
    virtual void observeNew(vraw obs, double _acc_x_sign, double _acc_y_sign);
    virtual Vector2D position(double time);
    virtual Vector2D velocity(double time);
    virtual Vector2D acceleration(double time);
    virtual double direction(double time);
    virtual double angular_velocity(double time);
};

#endif // ROBOTTRACKER_H
