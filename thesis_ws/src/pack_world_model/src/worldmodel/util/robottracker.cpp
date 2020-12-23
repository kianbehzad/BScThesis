/* LICENSE: */



#include <cstdio>

#include <pack_world_model/worldmodel/util/robottracker.h>
#include <pack_world_model/worldmodel/wm/robot.h>
#include <fstream>


const bool Robot_Fast_Predict = false;

const double Robot_Position_Variance = 0.005; //0.0005
const double Robot_Angle_Variance = 200.0; //2.0
const double Robot_Velocity_Next_Step_Covariance = 1.2; //1.2
const double Robot_Acceleration_Next_Step_Covariance = 1.0;//1.0
const double Robot_Velocity_Variance = 0.01; //0.01
const double Robot_Angular_Velocity_Variance = 9.0;//710
const double Robot_Acceleration_Variance = 1; //10

const double Robot_Stuck_Decay = 1.0;
const double Robot_Stuck_Threshold = 0.8;

const double kalman_gain_position = 0.5;
const double input_gain_position = 1 - kalman_gain_position;

const double kalman_gain_velocity = 0.5;
const double input_gain_velocity = 1 - kalman_gain_velocity;

const double kalman_threshold = 0.01;
const double kalman_threshold_2 = 0.0005;

double boundto(double x, double a, double b) {
    if (x > b) {
        return b;
    }
    if (x < a) {
        return a;
    }
    return x;
}



//
// State = ( x, y, theta, v_x, v_y, v_theta, a_x, a_y)
// Observation = ( x, y, theta )
//
// Noise_Propagate = ( v_x, v_y, v_theta, a_x, a_y )
// Noise_Update = ( x, y, theta )
//

RobotTracker::RobotTracker(double _latency, bool _our_robot)
    : Tracker(8, 3, getFramePeriod()) {
    latency = _latency;
    reset_on_obs = true;
    our_robot = false;//_our_robot;
    prediction_lookahead = getLatencyDelay();
}

RobotTracker::rcommand RobotTracker::get_command(double time) {
    if (cs.empty() || cs[0].timestamp > time) {
        return (rcommand) {
            0.0, Vector2D(0.0, 0.0), 0.0
        };
    }

    uint i;

    for (i = 1; i < cs.size(); i++) {
        if (cs[i].timestamp > time) {
            break;
        }
    }

    return cs[i - 1];
}

void RobotTracker::command(double timestamp, Vector2D v, double w) {
    rcommand c = { timestamp + latency - (getFramePeriod() / 2.0), v, w };

    while (cs.size() > 1 && cs[0].timestamp < time - stepsize) {
        cs.pop_front();
    }

    while (!cs.empty() && cs.back().timestamp == c.timestamp) {
        cs.pop_back();
    }

    cs.push_back(c);
}

void RobotTracker::observeNew(vraw obs, double _acc_x_sign, double _acc_y_sign) {
    obs.timestamp = time + stepsize;
    observe(obs, time + stepsize);
//  std::ofstream outFile( "optimrawdata" , std::ios::app );
//  outFile <<obs.pos.x<<"\n";
//  outFile.flush();
}

void RobotTracker::observe(vraw obs, double timestamp) {
    if (reset_on_obs) {
        if (obs.conf <= 0.0) {
            return;
        }

        static Matrix x(8, 1), P(8);

        x.e(0, 0) = obs.pos.x;
        x.e(1, 0) = obs.pos.y;
        x.e(2, 0) = obs.angle;
        x.e(3, 0) = 0.0;
        x.e(4, 0) = 0.0;
        x.e(5, 0) = 0.0;
        x.e(6, 0) = 0.0;
        x.e(7, 0) = 0.0;


        P.e(0, 0) = Robot_Position_Variance;
        P.e(1, 1) = Robot_Position_Variance;
        P.e(2, 2) = Robot_Angle_Variance;
        P.e(3, 3) = 0.0; // 0m/s
        P.e(4, 4) = 0.0;
        P.e(5, 5) = 0.0;
        P.e(6, 6) = 0.0;
        P.e(7, 7) = 0.0;

        initial(obs.timestamp, x, P);

        reset_on_obs = false;
    }

    else {
        // If this is a new observation.
        if (timestamp > time) {
            // Tick to current time. estimate eq time update
            tick(timestamp - time);

            // Make observation
            if (obs.timestamp == timestamp) {
                Matrix o(3, 1);
                o.e(0, 0) = obs.pos.x;
                o.e(1, 0) = obs.pos.y;
                o.e(2, 0) = obs.angle;

                update(o);

                if (our_robot) {
                    Vector2D kalman_pos(xs.back().e(0, 0), xs.back().e(1, 0));
                    Vector2D vision_pos(obs.pos.x, obs.pos.y);

                    if ((kalman_pos - vision_pos).innerProduct(Vector2D(xs.back().e(3, 0) , xs.back().e(4, 0)).normalize()) >
                            kalman_threshold) {
                        stuck += 0.1;
                    }

                    else if (Vector2D(xs.back().e(3, 0) , xs.back().e(4, 0)).length() > 0.01 &&
                             (kalman_pos - vision_pos).innerProduct(Vector2D(xs.back().e(3, 0) , xs.back().e(4, 0)).normalize()) <
                             kalman_threshold_2) {
                        stuck = 0;
                    }

                    stuck = boundTo(stuck, 0.0 , 1.0);
                }
            }

            if (error_time_elapsed() > 10.0) {
                //reset_on_obs = true;
                //        fprintf(stderr, "Kalman Error (pos, theta, vpos, vtheta): ");
                //        fprintf(stderr, "%f ",
                //                hypot(error_mean().e(0, 0), error_mean().e(1, 0)));
                //        fprintf(stderr, "%f ", error_mean().e(2, 0));
                //        fprintf(stderr, "%f ",
                //  #include <tracker.h>              hypot(error_mean().e(3, 0), error_mean().e(4, 0)));
                //        fprintf(stderr, "%f\n", error_mean().e(5, 0));
                error_reset();
            }
        }
    }
}

void RobotTracker::reset(double timestamp, float state[8]) {
    Matrix x(8, 1, state), P(8);

    P.e(0, 0) = Robot_Position_Variance;
    P.e(1, 1) = Robot_Position_Variance;
    P.e(2, 2) = Robot_Angle_Variance;
    P.e(3, 3) = 0.0; // 0m/s
    P.e(4, 4) = 0.0;
    P.e(5, 5) = 0.0;
    P.e(6, 6) = 0.0;
    P.e(7, 7) = 0.0;


    initial(timestamp, x, P);

    reset_on_obs = false;
}

Vector2D RobotTracker::position(double time) {
    Matrix x = predict(time);
    return Vector2D{x.e(0, 0), x.e(1, 0)};
}

Vector2D RobotTracker::velocity(double time) {
    if (our_robot && stuck > Robot_Stuck_Threshold) {
//    reset_on_obs = true;
        return Vector2D{0.0, 0.0};
    }

    Matrix x;

    x = predict(time);

    double vx = x.e(3, 0);
    double vy = x.e(4, 0);

    return Vector2D{vx, vy};
}

Vector2D RobotTracker::acceleration(double time) {
    Matrix x;

    x = predict(time);

    double ax = x.e(6, 0);
    double ay = x.e(7, 0);

    return Vector2D{ax, ay};
}


double RobotTracker::direction(double time) {
    Matrix x;

    x = predict(time);

    return x.e(2, 0);
}

double RobotTracker::angular_velocity(double time) {
    Matrix x;

    x = predict(time);

    return x.e(5, 0);
}

Matrix& RobotTracker::f(const Matrix &x, Matrix &I) {
    if (our_robot) {
        rcommand c = get_command(stepped_time);

        Matrix Us(3, 1);
        Us.e(0, 0) = c.v.x;
        Us.e(1, 0) = c.v.y;
        Us.e(2, 0) = c.w;

        return f2(x , I , Us);
    }

    static Matrix f;
    f = x;

    double
    &_x = f.e(0, 0),
     &_y = f.e(1, 0),
      &_theta = f.e(2, 0),
       &_vx = f.e(3, 0),
        &_vy = f.e(4, 0),
         &_vtheta = f.e(5, 0),
          &_ax = f.e(6, 0),
           &_ay = f.e(7, 0),


            stuck = 0.0;

//  _ax *= Robot_Acceleration_Next_Step_Covariance;
//  _ay *= Robot_Acceleration_Next_Step_Covariance;

//  _vtheta *= Robot_Velocity_Next_Step_Covariance;
//  _vx *= Robot_Velocity_Next_Step_Covariance;
//  _vy *= Robot_Velocity_Next_Step_Covariance;

    _vx += (1.0 - stuck) * stepsize * _ax;
    _vy += (1.0 - stuck) * stepsize * _ay;

    _theta += /*( 1.0 - stuckk ) * */stepsize * _vtheta;
    _x += (1.0 - stuck) * stepsize * _vx + 0.5 * (1.0 - stuck) * stepsize * stepsize * _ax ;
    _y += (1.0 - stuck) * stepsize * _vy + 0.5 * (1.0 - stuck) * stepsize * stepsize * _ay ;

    return f;
}

Matrix& RobotTracker::f2(const Matrix &x, Matrix &I , Matrix &U) {
    static Matrix f;
    f = x;

    double
    &_x = f.e(0, 0),
     &_y = f.e(1, 0),
      &_theta = f.e(2, 0),
       &_vx = f.e(3, 0),
        &_vy = f.e(4, 0),
         &_vtheta = f.e(5, 0),
          &_ax = f.e(6, 0),
           &_ay = f.e(7, 0),

            stuck = 0;
    stuck = boundto(stuck , 0 , 1) * Robot_Stuck_Decay;

//  _ax *= Robot_Acceleration_Next_Step_Covariance;
//  _ay *= Robot_Acceleration_Next_Step_Covariance;

//  _vtheta *= Robot_Velocity_Next_Step_Covariance;
//  _vx *= Robot_Velocity_Next_Step_Covariance;
//  _vy *= Robot_Velocity_Next_Step_Covariance;

    _vtheta = (input_gain_velocity * (U.e(2, 0)) + kalman_gain_velocity * _vtheta);
    _vx = (1.0 - stuck) * stepsize * _ax + (1.0 - stuck) * (input_gain_velocity * (U.e(0, 0)) + kalman_gain_velocity * _vx);
    _vy = (1.0 - stuck) * stepsize * _ay + (1.0 - stuck) * (input_gain_velocity * (U.e(1, 0)) + kalman_gain_velocity * _vy);

//  _vx +=  ( 1.0 - stuck ) * stepsize * _ax;
//  _vy +=  ( 1.0 - stuck ) * stepsize * _ay;

    _theta += /*( 1.0 - stuckk ) * */stepsize * (kalman_gain_position * _vtheta + input_gain_position * U.e(2, 0));
    _x += (1.0 - stuck) * stepsize * (kalman_gain_position * _vx + input_gain_position * U.e(0, 0)) + 0.5 * (1.0 - stuck) * stepsize * stepsize * _ax ;
    _y += (1.0 - stuck) * stepsize * (kalman_gain_position * _vy + input_gain_position * U.e(1, 0)) + 0.5 * (1.0 - stuck) * stepsize * stepsize * _ay ;

//  std::ofstream outFile( "asghar" , std::ios::app );
//  outFile <<__FILE__<<" ---> vx --->"<<_vx<<"\n";
//  outFile.flush();

    return f;
}

Matrix& RobotTracker::h(const Matrix &x) {
    static Matrix h(3, 1);

    h.e(0, 0) = x.e(0, 0);
    h.e(1, 0) = x.e(1, 0);
    h.e(2, 0) = x.e(2, 0);

    return h;
}

Matrix& RobotTracker::Q(const Matrix &x) {
    //angle is in degrees, position is in meters
    static Matrix Q;

    if (Q.nrows() == 0) {
        Q.identity(5);
        Q.e(0, 0) = Robot_Velocity_Variance;
        Q.e(1, 1) = Robot_Velocity_Variance;
        Q.e(2, 2) = Robot_Angular_Velocity_Variance;
        Q.e(3, 3) = Robot_Acceleration_Variance;
        Q.e(4, 4) = Robot_Acceleration_Variance;

    }

    return Q;
}

Matrix& RobotTracker::R(const Matrix &x) {
    static Matrix R;

    if (R.nrows() == 0) {
        R.identity(3);
        R.e(0, 0) = Robot_Position_Variance;
        R.e(1, 1) = Robot_Position_Variance;
        R.e(2, 2) = Robot_Angle_Variance;
    }

    return R;
}

Matrix& RobotTracker::A(const Matrix &x) {
    static Matrix A(8);

    if (!our_robot) {
        stuck = 0.0;
    }

    A.e(0, 3) = (1.0 - stuck) * stepsize;
    A.e(0, 6) = stepsize * stepsize * (1.0 - stuck) / 2.0;
    A.e(1, 4) = (1.0 - stuck) * stepsize;
    A.e(1, 7) =  stepsize * stepsize * (1.0 - stuck) / 2.0;
    A.e(2, 5) = /*(1.0 - stuckk) * */stepsize;
    A.e(3, 6) = (1.0 - stuck) * stepsize;
    A.e(4, 7) = (1.0 - stuck) * stepsize;
    A.e(3, 3) = Robot_Velocity_Next_Step_Covariance * (1.0 - stuck);
    A.e(4, 4) = Robot_Velocity_Next_Step_Covariance * (1.0 - stuck);
    A.e(5, 5) = Robot_Velocity_Next_Step_Covariance;
    A.e(6, 6) = Robot_Acceleration_Next_Step_Covariance;
    A.e(7, 7) = Robot_Acceleration_Next_Step_Covariance;

    if (our_robot) {
        A.e(0, 3) *= kalman_gain_position;
        A.e(1, 4) *= kalman_gain_position;
        A.e(2, 5) *= kalman_gain_position;
        A.e(3, 3) *= kalman_gain_velocity;
        A.e(4, 4) *= kalman_gain_velocity;
        A.e(5, 5) *= kalman_gain_velocity;
    }

    return A;
}

Matrix& RobotTracker::W(const Matrix &x) {
    static Matrix W((char *const)(
                        "[ 0, 0, 0, 0, 0 ; "
                        "  0, 0, 0, 0, 0 ; "
                        "  0, 0, 0, 0, 0 ; "
                        "  1, 0, 0, 0, 0 ; "
                        "  0, 1, 0, 0, 0 ; "
                        "  0, 0, 1, 0, 0 ; "
                        "  0, 0, 0, 1, 0 ; "
                        "  0, 0, 0, 0, 1 ]"));
    return W;
}

Matrix& RobotTracker::H(const Matrix &x) {
    static Matrix H((char *const)(
                        "[ 1, 0, 0, 0, 0, 0, 0, 0; "
                        "  0, 1, 0, 0, 0, 0, 0, 0; "
                        "  0, 0, 1, 0, 0, 0, 0, 0 ]"));
    return H;
}

Matrix& RobotTracker::V(const Matrix &x) {
    static Matrix V((char *const)(
                        "[ 1, 0, 0; "
                        "  0, 1, 0; "
                        "  0, 0, 1 ]"));
    return V;
}

double RobotTracker::boundTo(double x, double low, double high) {
    if (x < low) {
        x = low;
    }
    if (x > high) {
        x = high;
    }
    return (x);
}



