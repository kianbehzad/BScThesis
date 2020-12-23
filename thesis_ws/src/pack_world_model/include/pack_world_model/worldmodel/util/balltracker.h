#ifndef BALLTRACKER_H
#define BALLTRACKER_H

#include "pack_world_model/worldmodel/util/tracker.h"

const double Gravity       = 9.8;


enum OccludeFlag {
    Visible = 1,
    Occluded = 2
};

class BallTracker : public Tracker {
private:
    double kalman_gain_acceleration;
    double input_gain_acceleration;
    double acc_x_sign;
    double acc_y_sign;
protected:
    bool _reset;
    OccludeFlag occluded;
    char occluding_team, occluding_robot;
    Vector2D occluding_offset;
public:
    BallTracker(double _kalman_gain_acceleration);
    virtual ~BallTracker();
    double velocity_variance(const Matrix &x);
    bool check_occlusion();
    void tick_occlusion(double dt);
    Vector2D occluded_position(double time);
    Vector2D occluded_velocity(double time);
    virtual void observe(vraw obs, double timestamp);
    virtual void observeNew(vraw obs, double _acc_x_sign, double _acc_y_sign);
    virtual void reset();
    void reset(double timestamp, float state[4], float variances[16],
               OccludeFlag _occluded,
               char _occluding_team, char _occluding_robot,
               Vector2D _occluding_offset);
    virtual Vector2D position(double time);
    virtual Vector2D velocity(double time);
    virtual Vector2D acceleration(double time);

    virtual double direction(double time) {}
    virtual double angular_velocity(double time) {}
    Matrix covariances(double time);
    bool collision(double time, int &team, int &robot);
    Matrix& f(const Matrix &x, Matrix &I);
    virtual Matrix& f2(const Matrix &x, Matrix &I , Matrix &U) {}

    virtual Matrix& h(const Matrix &x);
    virtual Matrix& Q(const Matrix &x);
    virtual Matrix& R(const Matrix &x);
    virtual Matrix& A(const Matrix &x);
    virtual Matrix& W(const Matrix &x);
    virtual Matrix& H(const Matrix &x);
    virtual Matrix& V(const Matrix &x);
};

double BallFriction();

#endif // BALLTRACKER_H
