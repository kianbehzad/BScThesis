#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <QDebug>
#include "pack_world_model/worldmodel/util/balltracker.h"

using namespace std;


const double Ball_Position_Variance = 0.005;//0.005
const double Ball_Velocity_Variance = 0.05;//0.05
const double Ball_Acceleration_Variance = 0.5;//0.5
//const double Ball_Velocity_Variance_No_Robot = 0.5*0.5;
//const double Ball_Velocity_Variance_Near_Robot = 0.5*0.5;

//
// State = ( x, y, v_x, v_y )
// Observation = ( x, y )
//
// Noise_Propagate = ( v_x, v_y )
// Noise_Observe = ( x, y )
//



BallTracker::BallTracker(double _kalman_gain_acceleration) : Tracker(6, 2, getFramePeriod()) {
    _reset = true;
    occluded = Visible;
    prediction_lookahead = getLatencyDelay();
    kalman_gain_acceleration = _kalman_gain_acceleration;
    input_gain_acceleration = 1.0 - kalman_gain_acceleration;
}

BallTracker::~BallTracker()
    = default;

double BallTracker::velocity_variance(const Matrix &x) {
    return Ball_Velocity_Variance;
//
//  Vector2D ball = Vector2D(x.e(0, 0), x.e(1, 0));
//  double dist = 5000.0;
//
//  for(int i = 0; i < NUM_TEAMS; i++) {
//    for(int j = 0; j < MAX_TEAM_ROBOTS; j++) {
//      if (!tracker->Exists(i, j)) continue;
//
//      double d = (tracker->robots[i][j].position(0.0) - ball).length();
//      if (d < dist) dist = d;
//    }
//  }
//
//  double r = bound((dist - ROBOT_DEF_WIDTH_H) / ROBOT_DEF_WIDTH_H, 0, 1);
//  return r * DVAR(BALL_VELOCITY_VARIANCE_NO_ROBOT) +
//    (1 - r) * DVAR(BALL_VELOCITY_VARIANCE_NEAR_ROBOT);
}

bool BallTracker::check_occlusion() {
    return false;
    /*  if (!tracker) return false;

      if (occluded != Visible) return true;

      Vector2D camera(0,0);
      Vector2D ball = position(0.0) - camera;

      double occluding_pct = 0.5;

      for(int i = 0; i < NUM_TEAMS; i++) {
        for(int j = 0; j < MAX_TEAM_ROBOTS; j++) {
          if (!tracker->Exists(i, j)) continue;

          double radius = tracker->Radius(i, j);
          double height = tracker->Height(i, j);

          Vector2D p = tracker->robots[i][j].position(0.0) - camera;
          double from = offset_to_line(Vector2D(0, 0), ball, p);
          double along = offset_along_line(Vector2D(0, 0), ball, p);
          double ball_along = ball.length();

          if (fabs(from) > radius) continue;
          if (ball_along < along) continue;

          along += sqrt(radius * radius - from * from);

          double x = (along * height) / (CAMERA_HEIGHT - height);
          double pct = (x - (ball_along - along) + BALL_RADIUS) /
            (2.0 * BALL_RADIUS);

          pct = bound(pct, 0, 1);

          if (pct > occluding_pct) {
            occluded = MaybeOccluded;
            occluding_team = i;
            occluding_robot = j;
            occluding_offset = (ball - p).rotate(-(p - camera).angle());

            occluding_pct = pct;
          }
        }
      }

      return (occluded != Visible);*/
}

void BallTracker::tick_occlusion(double dt) {
    /*  Vector2D camera(0, 0);

      Vector2D p = tracker->robots[occluding_team][occluding_robot]
        .position(0.0);
      Vector2D v = tracker->robots[occluding_team][occluding_robot]
        .velocity(0.0);
      Vector2D b = occluding_offset.rotate((p - camera).angle());

      double bdelta = MAX(v.dot(b.norm()) * dt, 0.0);
      double radius;

      switch(tracker->Type(occluding_team, occluding_robot)) {
      case ROBOT_TYPE_DIFF:
      case ROBOT_TYPE_OMNI:
        radius = DVAR(BALL_TEAMMATE_COLLISION_RADIUS); break;
      default:
        radius = DVAR(BALL_OPPONENT_COLLISION_RADIUS); break;
      }

      if (b.length() - bdelta < radius) b = b.norm(radius);
      else b = b.norm(b.length() - bdelta);

      occluding_offset = b.rotate(-(p - camera).angle());

      // Update the x and P queue.
      Matrix x(4,1), P(4);
      Vector2D xp = occluded_position(dt);
      Vector2D xv = occluded_velocity(dt);

      x.e(0,0) = xp.x;
      x.e(1,0) = xp.y;
      x.e(2,0) = xv.x;
      x.e(3,0) = xv.y;

      P.e(0,0) *= DVAR(BALL_POSITION_VARIANCE);
      P.e(1,1) *= DVAR(BALL_POSITION_VARIANCE);
      P.e(2,2) *= 250000.0; // 500m/s
      P.e(3,3) *= 250000.0; // 500m/s

      xs.clear(); xs.push_back(x);
      Ps.clear(); Ps.push_back(P);
      time += dt;
      */

}

Vector2D BallTracker::occluded_position(double time) {

    Vector2D camera(0, 0);
    Vector2D b;

//  b = tracker->robots[occluding_team][occluding_robot].position(time);
//  b += occluding_offset.rotate((b - camera).angle());

    return b;
}

Vector2D BallTracker::occluded_velocity(double time) {
//  if (!tracker) return Vector2D(0.0, 0.0);
//  return tracker->robots[occluding_team][occluding_robot].velocity(time);
}

void BallTracker::observeNew(vraw obs, double _acc_x_sign, double _acc_y_sign) {
    obs.timestamp = time + stepsize;
    observe(obs, time + stepsize);
    acc_x_sign = _acc_x_sign;
    acc_y_sign = _acc_y_sign;
}

void BallTracker::observe(vraw obs, double timestamp) {
    // mhb: Need this?
    if (isnan(xs[0].e(0, 0))) {
        _reset = true;
    }

    if (_reset && obs.timestamp >= timestamp) {
        Matrix x(6, 1), P(6);

        x.e(0, 0) = obs.pos.x;
        x.e(1, 0) = obs.pos.y;
        x.e(2, 0) = 0.0;
        x.e(3, 0) = 0.0;
        x.e(4, 0) = 0.0;
        x.e(5, 0) = 0.0;

        P.e(0, 0) *= Ball_Position_Variance;
        P.e(1, 1) *= Ball_Position_Variance;
        P.e(2, 2) *= Ball_Velocity_Variance;
        P.e(3, 3) *= Ball_Velocity_Variance;
        P.e(4, 4) *= Ball_Acceleration_Variance;
        P.e(5, 5) *= Ball_Acceleration_Variance;

        initial(obs.timestamp, x, P);

        occluded = Visible;

        _reset = false;

    } else {

//    if (_reset && occluded != Occluded) return;

        // If this is a new observation.
        if (timestamp > time) {

            // Tick to current time.
            if (occluded == Occluded) {
                tick_occlusion(timestamp - time);
            } else {
                tick(timestamp - time);
            }

            // Make Observation Matrix
            Matrix o(2, 1);
            o.e(0, 0) = obs.pos.x;
            o.e(1, 0) = obs.pos.y;


            // Make observation
//      if (obs.timestamp == timestamp)
            {
                update(o);

//        occluded = Visible;
//        occluded_last_obs_time = obs.timestamp;

            }
//      else {
//
//        if (occluded == Visible)
//          check_occlusion();
//
//        if (occluded == MaybeOccluded &&
//            timestamp - occluded_last_obs_time > DVAR(BALL_OCCLUDE_TIME)) {
//
//          occluded = Occluded;
//          _reset = true;
//        }
//      }

            if (error_time_elapsed() > 10.0) {
//        fprintf(stderr, "Kalman Error (pos, vpos): ");
//        fprintf(stderr, "%f ",
//                hypot(error_mean().e(0, 0), error_mean().e(1, 0)));
//        fprintf(stderr, "%f\n",
//                hypot(error_mean().e(2, 0), error_mean().e(3, 0)));
                error_reset();
            }

        }
    }

}

void BallTracker::reset() {
    _reset = true;
}

void BallTracker::reset(double timestamp, float state[6], float variances[16],
                        OccludeFlag _occluded,
                        char _occluding_team, char _occluding_robot,
                        Vector2D _occluding_offset) {
    Matrix x(6, 1, state), P(6, 6, variances);

    initial(timestamp, x, P);

    occluded = _occluded;
    occluding_team = _occluding_team;
    occluding_robot = _occluding_robot;
    occluding_offset = Vector2D(_occluding_offset.x, _occluding_offset.y);

    _reset = false;
}

Vector2D BallTracker::position(double time) {
    if (occluded == Occluded) {
        return occluded_position(time);
    }

//  qDebug()<<"Ball Prediction";
    Matrix x = predict(time);
    return Vector2D(x.e(0, 0), x.e(1, 0));
}

Vector2D BallTracker::velocity(double time) {
    if (occluded == Occluded) {
        return occluded_velocity(time);
    }

//  qDebug()<<"Ball Prediction";
    Matrix x = predict(time);
    return Vector2D{x.e(2, 0), x.e(3, 0)};

}

Vector2D BallTracker::acceleration(double time) {

//  qDebug()<<"Ball Prediction";
    Matrix x = predict(time);
    return Vector2D(x.e(4, 0), x.e(5, 0));
}

Matrix BallTracker::covariances(double time) {
    return predict_cov(time);
}

bool BallTracker::collision(double time, int &team, int &robot) {
    Matrix I = predict_info(time);

    if (I.nrows() <= 1) {
        return false;
    }

    team = (int) rint(I.e(0, 0));
    robot = (int) rint(I.e(1, 0));

    return true;
}

#ifndef MIN
#define MIN(a,b) ((a<b) ? a : b)
#endif

Matrix& BallTracker::f(const Matrix &x, Matrix &I) {
    I = Matrix();

    static Matrix f;

    f = x; // Copy Matrix
    double &_x = f.e(0, 0), &_y = f.e(1, 0), &_vx = f.e(2, 0), &_vy = f.e(3, 0), &_ax = f.e(4, 0), &_ay = f.e(5, 0);
    double _v = sqrt(_vx * _vx + _vy * _vy);
    //mhmmd
    double _a =  MIN(BallFriction() * Gravity, _v / stepsize);
//  double _a = BallFriction()*Gravity;
//  double _a = 0;
//  double _ax = (_v == 0.0) ? 0.0 : -_a * _vx / _v;
//  double _ay = (_v == 0.0) ? 0.0 : -_a * _vy / _v;

    _ax = kalman_gain_acceleration * _ax + ((_v < 0.01) ? 0.0 : -(input_gain_acceleration * _a * _vx / _v));
    _ay = kalman_gain_acceleration * _ay + ((_v < 0.01) ? 0.0 : -(input_gain_acceleration * _a * _vy / _v));

    double walls = 0.0;
//
//  if (IVAR(BALL_WALLS_SLOPED)) {
//    if (fabs(_x) > FIELD_LENGTH_H && fabs(_y) > GOAL_WIDTH_H) {
//      _ax += copysign(M_SQRT1_2 * GRAVITY * 5.0 / 7.0, -_x);
//      walls = true;
//    }
//
//    if (fabs(_y) > FIELD_WIDTH_H) {
//      _ay = copysign(M_SQRT1_2 * GRAVITY * 5.0 / 7.0, -_y);
//      walls = true;
//    }
//  }
//
//  if (IVAR(BALL_WALLS_OOB)) {
//    if ((fabs(_x) > FIELD_LENGTH_H + WALL_WIDTH &&
//         fabs(_y) > GOAL_WIDTH_H) ||
//        (fabs(_y) > FIELD_WIDTH_H + WALL_WIDTH)) {
//      _vx = 0.0;
//      _vy = 0.0;
//      _ax = 0.0;
//      _ay = 0.0;
//
//      walls = true;
//    }
//  }

    // Update Position
    _x += _vx * stepsize + 0.5 * _ax * stepsize * stepsize;
    _y += _vy * stepsize + 0.5 * _ay * stepsize * stepsize;

    // If there's a collision... then set ball's velocity to the colliding
    //  object's velocity.
    Vector2D cv, cp;
    int team = 0 , robot = 0;

    _vx += _ax * stepsize;
    _vy += _ay * stepsize;

    return f;
}

Matrix& BallTracker::h(const Matrix &x) {
    static Matrix h(2, 1);
    h.e(0, 0) = x.e(0, 0);
    h.e(1, 0) = x.e(1, 0);
    return h;
}

Matrix& BallTracker::Q(const Matrix &x) {
    static Matrix Q(4);

    // Base noise covariances on distance to nearest robot.
    Q.e(0, 0) = Q.e(1, 1) = velocity_variance(x);
    Q.e(2, 2) = Q.e(3, 3) = Ball_Acceleration_Variance;

    return Q;
}

Matrix& BallTracker::R(const Matrix &x) {
    static Matrix R;

    if (!R.nrows()) {
        R.identity(2);
        R.scale(Ball_Position_Variance);
    }

    return R;
}

Matrix& BallTracker::A(const Matrix &x) {
    static Matrix A;

    // This is not quite right since this doesn't account for friction
    // But the Jacobian with friction is pretty messy.

    if (!A.nrows()) {
        A.identity(6);
        A.e(0, 2) = stepsize;
        A.e(0, 4) = stepsize * stepsize * 0.5;
        A.e(1, 3) = stepsize;
        A.e(1, 5) = stepsize * stepsize * 0.5;
        A.e(2, 4) = stepsize;
        A.e(3, 5) = stepsize;
        A.e(4, 4) = kalman_gain_acceleration;
        A.e(5, 5) = kalman_gain_acceleration;
    }

    return A;
}

Matrix& BallTracker::W(const Matrix &x) {
    static Matrix W((char *const)(
                        "[0, 0, 0, 0;"
                        " 0, 0, 0, 0;"
                        " 1, 0, 0, 0;"
                        " 0, 1, 0, 0;"
                        " 0, 0, 1, 0;"
                        " 0, 0, 0, 1]"));
    return W;
}

Matrix& BallTracker::H(const Matrix &x) {
    static Matrix H((char *const)(
                        "[ 1, 0, 0, 0, 0, 0; "
                        "  0, 1, 0, 0, 0, 0 ] "));
    return H;
}

Matrix& BallTracker::V(const Matrix &x) {
    static Matrix V((char *const)(
                        "[ 1, 0; "
                        "  0, 1 ]"));

    return V;
}


double BallFriction() {
//    return 0.013;// 0.19;
    /*  if (wm->getIsSimulMode())
        return 0.07;*/
    return 0.19;// 0.175;// 0.19;

}
