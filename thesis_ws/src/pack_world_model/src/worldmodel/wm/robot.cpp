//
// Created by parsian-ai on 10/13/17.
//

#include <pack_world_model/worldmodel/wm/robot.h>

#include <QFile>
#include <QDebug>
#include <fstream>


const double Robot::robot_radius_old         = knowledge::robot_radius_old;
//const double Robot::robot_radius_new       = knowledge::robot_radius_new;
const double Robot::robot_radius_new         = knowledge::robot_radius_new;
const double Robot::center_from_kicker_old   = knowledge::center_from_kicker_old;
//const double Robot::center_from_kicker_new = knowledge::center_from_kicker_new;
const double Robot::center_from_kicker_new   = knowledge::center_from_kicker_new;
const double Robot::kicker_width_old         = knowledge::kicker_width_old;
//const double Robot::kicker_width_new       = knowledge::kicker_width_new;
const double Robot::kicker_width_new         = knowledge::kicker_width_new;
const double Robot::wheel_rad_old            = knowledge::wheel_rad_old;
//const double Robot::wheel_rad_new          = knowledge::wheel_rad_new;
const double Robot::wheel_rad_new            = knowledge::wheel_rad_new;

void Robot::setNewRobot(bool _new) {
    newRobot = _new;
}

bool Robot::isNewRobot() {
    return newRobot;
}

double Robot::robotRadius() {
    return (newRobot) ? robot_radius_new : robot_radius_old;
}

double Robot::kickerWidth() {
    return (newRobot) ? kicker_width_new : kicker_width_old;
}

double Robot::centerFromKicker() {
    return (newRobot) ? center_from_kicker_new : center_from_kicker_old;
}

double Robot::wheelRadius() {
    return (newRobot) ? wheel_rad_new : wheel_rad_old;
}


Robot::Robot(int _id, bool isOurTeam, bool noKalman) : MovingObject(true) {
    ////////////////////////////////new kalman
    kalman::Vector x;
    x(0) = -1 * observation->pos.y;
    x(1) = observation->pos.x;
    x(2) = observation->dir.th().radian();
    x(3) = 0.0;
    x(4) = 0.0;
    x(5) = 0.0;
    donKalman = new kalman(x);
    kalmanTime.start();
    // we can only observe the position
    donKalman->H(0, 0) = 1.0;
    donKalman->H(1, 1) = 1.0;
    donKalman->H(2, 2) = 1.0;

    vForwardCmd = 0.5;
    vNormalCmd = 0;
    vAngCmd = 0;
    test = 0;
    /////////////////////////////////////////////

    /*////////////////////*/


    lastFrameKalmanReset = 0;
    newRobot = false;
    repl = false;
    lastFrameAngle = turns = 0.0;
    id = _id;
    inOurTeam = isOurTeam;
    vel.assign(0, 0);
    angularVel = 0;
    lastInsight = 0;
    inSight = 0.0;
    elementNotInSight = true;
    if (!noKalman) {
        tracker = new RobotTracker(getLatencyDelay(), inOurTeam);
        tracker->reset();
    } else {
        tracker = nullptr;
    }
    blindness = 100;
    ANGULAR_DIRECTION = 0.0;
    ADD_TO_ANGULAR_DIRECTION = 0.0;

    markedByDefense = false;
    markedByMark = false;
}
void Robot::newPredict(qint64 time, bool updateFuture, bool permanentUpdate, bool cameraSwitched, bool applyCommand, kalmParam _param) {
    kalman *kalman = donKalman;
    const qint64 lastTime = (updateFuture) ? kalmanFutureLastTime : kalmanLastTime;
    double timeDiff = (max(kalmanTime.elapsed() , 1)) * 0.001; //(time - lastTime);
    //timeDiff = 0.016;
    Q_ASSERT(timeDiff >= 0);
    const float phi = kalman->baseState()(2) - (_PI / 2);
    const float v_s = kalman->baseState()(3);
    const float v_f = kalman->baseState()(4);
    const float omega = kalman->baseState()(5);
    // Process state transition: update position with the current speed
    kalman->F(0, 3) = std::cos(phi) * timeDiff;
    kalman->F(0, 4) = -1 * std::sin(phi) * timeDiff;
    kalman->F(1, 3) = std::sin(phi) * timeDiff;
    kalman->F(1, 4) = 1 * std::cos(phi) * timeDiff;
    kalman->F(2, 5) = timeDiff;

    kalman->F(3, 3) = 1;
    kalman->F(4, 4) = 1;
    kalman->F(5, 5) = 1;

    kalman->u = kalman::Vector::Zero();
    ////////////strange if
    test++;
    float v_x = std::cos(phi) * v_s - std::sin(phi) * v_f;
    float v_y = std::sin(phi) * v_s + std::cos(phi) * v_f;
//    ROS_INFO_STREAM("Vf" << phi * _RAD2DEG << " VN" << vNormalCmd);
    // radio commands are intended to be applied over 10ms
    float cmd_interval = (float)(timeDiff);
    //TODO: add radio command
    float cmd_omega = _param.vw;
    float cmd_v_y = _param.vy;
    float cmd_v_x = _param.vx;
    float a_w = (cmd_omega - omega) / cmd_interval;

    float accel_x = (cmd_v_x - v_x) / cmd_interval;
    float accel_y = (cmd_v_y - v_y) / cmd_interval;
    float accel_s = (std::cos(-phi) * accel_x - std::sin(-phi) * accel_y) * timeDiff;
    float accel_f = (std::sin(-phi) * accel_x + std::cos(-phi) * accel_y) * timeDiff;



     if(fabs(accel_f) > 5)
        accel_f = 5 * sign(accel_f);
    if(fabs(accel_s )> 5)
        accel_s = 5 * sign(accel_s);

    //debug(QString("acc : %1").arg(accel_f),D_MHMMD);


    kalman->u(0) = 0;
    kalman->u(1) = 0;
    kalman->u(2) = 0;
    //TODO : must know is it simulator or not
    if (/*(!knowledge->isSimulMode) &&*/ applyCommand) {
        kalman->u(3) = 0;//accel_s ;
        kalman->u(4) = 0;//  accel_f;
        kalman->u(5) = 0;//a_w * timeDiff;
    }

    // update covariance jacobian
    kalman->B = kalman->F;
    kalman->B(0, 2) = -(v_s * std::sin(phi) + v_f * std::cos(phi)) * timeDiff;
    kalman->B(1, 2) = (v_s * std::cos(phi) - v_f * std::sin(phi)) * timeDiff;


    // Process noise: stddev for acceleration
    // guessed from the accelerations that are possible on average
    const float sigma_a_x = 4.0f;
    const float sigma_a_y = 4.0f;
    // a bit too low, but that speed is nearly impossible all the time
    const float sigma_a_phi = 10.0f;

    // using no position errors (in opposite to the CMDragons model)
    // seems to yield better results in the simulator
    // d = timediff
    // G = (d^2/2, d^2/2, d^2/2, d, d, d)
    // sigma = (x, y, phi, x, y, phi)  (using x = sigma_a_x, ...)
    // Q = GG^T*(diag(sigma)^2)
    kalman::Vector G;
    G(0) = timeDiff * timeDiff / 2 * sigma_a_x;
    G(1) = timeDiff * timeDiff / 2 * sigma_a_y;
    G(2) = timeDiff * timeDiff / 2 * sigma_a_phi;
    G(3) = timeDiff * sigma_a_x;
    G(4) = timeDiff * sigma_a_y;
    G(5) = timeDiff * sigma_a_phi;

    //    if (cameraSwitched) {
    //        // handle small errors in camera alignment
    //        G(0) += 0.02;
    //        G(1) += 0.02;
    //        G(2) += 0.05;
    //    }

    kalman->Q(0, 0) = G(0) * G(0);
    kalman->Q(0, 3) = G(0) * G(3);
    kalman->Q(3, 0) = G(3) * G(0);
    kalman->Q(3, 3) = G(3) * G(3);

    kalman->Q(1, 1) = G(1) * G(1);
    kalman->Q(1, 4) = G(1) * G(4);
    kalman->Q(4, 1) = G(4) * G(1);
    kalman->Q(4, 4) = G(4) * G(4);

    kalman->Q(2, 2) = G(2) * G(2);
    kalman->Q(2, 5) = G(2) * G(5);
    kalman->Q(5, 2) = G(5) * G(2);
    kalman->Q(5, 5) = G(5) * G(5);

    kalman->predict(permanentUpdate);
    kalmanLastTime = time;


}
void Robot::visionUpdate() {
    donKalman->z(0) = -1 * observation->pos.y;
    donKalman->z(1) = observation->pos.x;
    double appliedDir = observation->dir.th().degree();
    if ((observation->dir.th().degree() - lastDir.th().degree()) > 180) {
        appliedDir += 360;
    }
    if ((observation->dir.th().degree() - lastDir.th().degree()) < -180) {
        appliedDir -= 360;
    }
    donKalman->z(2) = (ANGULAR_DIRECTION + ADD_TO_ANGULAR_DIRECTION) * _DEG2RAD;

    kalman::MatrixMM R = kalman::MatrixMM::Zero();
    R(0, 0) = 0.004;
    R(1, 1) = 0.004;
    R(2, 2) = 0.01;
    donKalman->R = R.cwiseProduct(R);
    donKalman->update();
}



Robot::~Robot() {
    if (donKalman != NULL) {
        delete donKalman;
    }
}

void Robot::resetKalman() {
    if (tracker != nullptr) {
        delete tracker;
        tracker = new RobotTracker(getLatencyDelay() , inOurTeam);
        tracker->reset();
    }
}

void Robot::init() {
    //    motionEstimator = new CMotionEstimator();
    //    motionEstimator->svm->load("model.svm");
}

void Robot::filter(int vanished) {
    if (tracker == NULL) {
        MovingObject::filter(vanished);
        return;
    }

    if (ANGULAR_DIRECTION > 120.0 && observation->dir.th().degree() < -120.0) {
        ADD_TO_ANGULAR_DIRECTION += 360.0;
    }

    if (ANGULAR_DIRECTION < -120.0 && observation->dir.th().degree() > 120.0) {
        ADD_TO_ANGULAR_DIRECTION -= 360.0;
    }

    ANGULAR_DIRECTION = observation->dir.th().degree();
    double kalmanDelayTime = 0.1;

    if (false && inOurTeam) {
        if (vanished <= 0) {


            int nStep = kalmanDelayTime / 0.016;
            while(lastCommands.count() > nStep-1) {
                lastCommands.removeAt(lastCommands.count() - 1);
            }
//            ROS_INFO_STREAM("tedade ina : "<< lastCommands.count());
            kalmParam  temp;
            const float phi = donKalman->baseState()(2) - (_PI / 2);
            temp.vw = vAngCmd * _DEG2RAD;
            temp.vy = vForwardCmd * sin(-phi) - (vNormalCmd * cos(-phi));
            temp.vx = -1 * vForwardCmd * cos(-phi) - (vNormalCmd * sin(-phi));
            lastCommands.append(temp);
            // newPredict(0,false,true,false,(inOurTeam));
            for (int i = 0  ; i < (lastCommands.count()) ; i++) {
                newPredict(nStep, false, true, false, inOurTeam,lastCommands[i]);
            }
            visionUpdate();

            pos = Vector2D(donKalman->state()(1), -1 * donKalman->state()(0));
            dir = Vector2D(cos(donKalman->state()(2)), sin(donKalman->state()(2)));

            double phiii = donKalman->state()(2);
            const float v_s = donKalman->state()(3);
            const float v_f = donKalman->state()(4);
            //debug(QString("phi: %1").arg(phi))
            float vx = -1 * std::cos(phiii) * v_s + std::sin(phiii) * v_f;
            float vy = -1 * std::sin(phiii) * v_s - std::cos(phiii) * v_f;

            vel = Vector2D(vx, vy);
            acc = Vector2D(0, 0);
            angularVel = donKalman->state()(5);
            inSight = 1;

        } else if (vanished <= blindness) {
            //TODO: get from config
            int nStep = kalmanDelayTime / 0.016;
            kalmParam temp;
            temp.vx = 0;
            temp.vy = 0;
            temp.vw = 0;
            newPredict(1, false, true, false, (false),temp);
            visionUpdate();

            pos = Vector2D(donKalman->state()(1), -1 * donKalman->state()(0));
            dir = Vector2D(cos(donKalman->state()(2)), sin(donKalman->state()(2)));
            double phi = donKalman->state()(3);
            const float v_s = donKalman->state()(3);
            const float v_f = donKalman->state()(4);
            float vx = std::cos(phi) * v_s - std::sin(phi) * v_f;
            float vy = std::sin(phi) * v_s + std::cos(phi) * v_f;
            vel = Vector2D(vx, vy);
            acc = Vector2D(0, 0);
            angularVel = donKalman->state()(5);
            inSight = 0.5;

        } else {
            pos.invalidate();
            inSight = 0;
        }
        kalmanTime.restart();
        return;

    }

    ////////////////////mhmmd
    int kalmanVelTune = 8;
    if (vanished <= 0) {
        vraw v;

        v.angle = ANGULAR_DIRECTION + ADD_TO_ANGULAR_DIRECTION;
        v.conf  = observation->confidence;
        v.pos   = observation->pos;
        v.timestamp = observation->time;
        //TODO: add robot command
//        if(inOurTeam)
//        {
//            tracker->command(v.timestamp,Vector2D(wm->our[id]->kalman_velocs.vx,wm->our[id]->kalman_velocs.vy),wm->our[id]->kalman_velocs.vw);
//        }

        tracker->observeNew(v, 0, 0);
        pos = v.pos;
        dir = observation->dir;
        pos = tracker->position(getFramePeriod());
        vel = tracker->velocity(kalmanVelTune * getFramePeriod());
        acc = tracker->acceleration(kalmanVelTune * getFramePeriod());
        dir = Vector2D::unitVector(tracker->direction(getFramePeriod()));
        angularVel = tracker->angular_velocity(kalmanVelTune * getFramePeriod());

        inSight = 1.0;
    } else {
        if (vanished < blindness) {
            /*
            vraw v;
            v.conf  = 1;//observation.confidence;
            v.angle = observation.dir.th().degree();// + angularVel*getFramePeriod();
            //            vel *= 0.5;
            vel *= 0.0;
            v.pos   = observation.pos;// + vel*getFramePeriod();
            v.timestamp = observation.time;
            //            tracker->observe(v, v.timestamp);
            //            tracker->observeNew(v);
            pos = tracker->position(getFramePeriod());
            pos = v.pos;

            vel = tracker->velocity(getFramePeriod());
            //            dir = Vector2D::unitVector(tracker->angular_velocity(getFramePeriod()));
            //            angularVel = tracker->angular_velocity(getFramePeriod());
            //            tracker->reset();
            angularVel *= 0.8;
            dir = Vector2D::unitVector(tracker->direction(getFramePeriod()));
            //            dir = Vector2D::unitVector(v.angle);
            inSight = 0.5;
            */

            vraw v;
            v.conf  = 1;
            v.angle = ANGULAR_DIRECTION + ADD_TO_ANGULAR_DIRECTION;
            v.pos   = observation->pos;
            v.timestamp = observation->time;

            // if(inOurTeam)
            // {
            //     tracker->command(v.timestamp,Vector2D(kalman_velocs.vx,kalman_velocs.vy),kalman_velocs.vw);
            // }

            pos = v.pos;
            dir = observation->dir;
            //     pos = tracker->position((8+vanished)*getFramePeriod());
            vel = tracker->velocity((kalmanVelTune + vanished - 1) * getFramePeriod());
            acc = tracker->acceleration((kalmanVelTune + vanished - 1) * getFramePeriod());
            dir = Vector2D::unitVector(tracker->direction((8 + vanished) * getFramePeriod()));
            angularVel = tracker->angular_velocity((kalmanVelTune + vanished - 1) * getFramePeriod());
            inSight = 0.5;
        }

        else {
            tracker->reset();
            vel *= 0.8;
            acc *= 0.8;
            angularVel *= 1;
            inSight = 0.0;
        }
    }

}

//double Robot::distFunction(Vector2D pos1,Vector2D dir1,Vector2D pos2,Vector2D dir2)
//{
//    return hypot(((pos1-pos2).length() / hypot(field._FIELD_WIDTH, field._FIELD_HEIGHT)),Vector2D::angleBetween(dir1,dir2).degree()/360.0);
//}



bool Robot::isActive() {
    return (this->inSight > 0.0);
}

Vector2D Robot::getKickerPos(double margin) {
    return pos + dir * (centerFromKicker() + margin);
}

Circle2D Robot::getCirle() {
    return Circle2D{pos, robotRadius()};
}

//bool Robot::isBallOwner(double verticaldist, double horizontaldist)
////default horizontal dist is kickerWidth()
//{
//    double d1 = horizontaldist;
//    if (d1 < 0) d1 = kickerWidth();
//    return (this->kickSensor && conf()->Common_KickSensor()) ||
//           ((fabs((ball->pos - pos).innerProduct(dir))<centerFromKicker()+CBall::radius+verticaldist)
//            && (fabs((ball->pos - pos).outerProduct(dir))<d1*0.5));
//}
//
//double Robot::ballComingSpeed()
//{
//    return ball->vel * (pos - ball->pos).norm();
//}

//bool Robot::isBallComingTowardMe(double factor,double vBallMin)
//{
//    Vector2D v1=pos-ball->pos;
//    Vector2D v2=ball->vel.norm();
//    double k = 1.0 + factor*(v1.length()-centerFromKicker())/(hypot(_FIELD_WIDTH,_FIELD_HEIGHT));
//    return (fabs(v1.outerProduct(v2)) < k*kickerWidth()*0.5) && (v1.innerProduct(v2) > 0) && (ball->vel.length()>vBallMin);
//}

//bool Robot::isBallGoingFartherFrom(double factor,double vBallMin)
//{
//    Vector2D v1=pos-ball->pos;
//    Vector2D v2=ball->vel.norm();
//    double k = 1.0 + factor*(v1.length()-robotRadius())/(hypot(_FIELD_WIDTH,_FIELD_HEIGHT));
//    return (fabs(v1.outerProduct(v2)) < k*kickerWidth()*0.5) && (v1.innerProduct(v2) < 0) && (ball->vel.length()>vBallMin);
//}
//
//bool Robot::isBallBack()
//{
//    return (((ball->pos-pos).outerProduct(dir) < 2.0*robotRadius())
//            &&
//            ((ball->pos-(dir*robotRadius())).innerProduct(dir)<0));
//}
//
//bool Robot::isBallAside()
//{
//    //atan(3.5/8.7)  ~ 25 degrees
//    if ((ball->pos - pos).length() < robotRadius() + CBall::radius + 0.03)
//    {
//        if (fabs(Vector2D::angleBetween(dir, ball->pos - pos).degree())>20)
//            return true;
//    }
//    else
//        return false;
//    return false;
//}
//
//bool Robot::collidesBall(Vector2D target)
//{
//Line2D l(pos, target);
//double s = sign((ball->pos - pos) * (target - pos))*sign((ball->pos - target) * (target - pos));
//if (s>0) return false;
//return l.dist(ball->pos) < CBall::radius + robotRadius() + 0.01;
//}
//
//bool Robot::isBallBack(Vector2D goal)
//{
//Vector2D target = -(goal - ball->pos).norm()*(centerFromKicker() + CBall::radius+0.020) + ball->pos;
//Segment2D l1(this->pos+Vector2D::unitVector((this->pos-target).th().degree()+90)*robotRadius(),target+Vector2D::unitVector(this->dir.th().degree()+90)*robotRadius());
//Segment2D l2(this->pos+Vector2D::unitVector((this->pos-target).th().degree()-90)*robotRadius(),target+Vector2D::unitVector(this->dir.th().degree()-90)*robotRadius());
//Segment2D l3(this->pos,target);
//Segment2D l4(this->pos+Vector2D::unitVector((this->pos-target).th().degree()+90)*robotRadius()*0.5,target+Vector2D::unitVector(this->dir.th().degree()+90)*robotRadius()*0.5);
//Segment2D l5(this->pos+Vector2D::unitVector((this->pos-target).th().degree()-90)*robotRadius()*0.5,target+Vector2D::unitVector(this->dir.th().degree()-90)*robotRadius()*0.5);
//Circle2D cc(ball->pos,robotRadius());
//Vector2D s1,s2;
//if (Vector2D::angleOf(goal,this->pos,ball->pos).degree()<30) return false;
//bool f = ((cc.intersection(l1,&s1,&s2)!=0) ||
//          (cc.intersection(l2,&s1,&s2)!=0) ||
//          (cc.intersection(l3,&s1,&s2)!=0) ||
//          (cc.intersection(l4,&s1,&s2)!=0) ||
//          (cc.intersection(l5,&s1,&s2)!=0));
//return f;
//}
//


void Robot::setReplace(Vector2D newPos, float newDirection) {
    replPos = newPos;
    replDir = newDirection;
    repl = true;
}

int Robot::replacementPacket(char* buf) {
    if (repl) {
        buf[0] = 100;
    } else {
        buf[0] = 10;
    }
    float x;
    x = static_cast<float>(replPos.x);
    buf[1] = *((char*)(& (x)));
    buf[2] = *((char*)(& (x)) + 1);
    buf[3] = *((char*)(& (x)) + 2);
    buf[4] = *((char*)(& (x)) + 3);
    x = static_cast<float>(replPos.y);
    buf[5] = *((char*)(& (x)));
    buf[6] = *((char*)(& (x)) + 1);
    buf[7] = *((char*)(& (x)) + 2);
    buf[8] = *((char*)(& (x)) + 3);
    x = replDir;
    buf[9]  = *((char*)(& (x)));
    buf[10] = *((char*)(& (x)) + 1);
    buf[11] = *((char*)(& (x)) + 2);
    buf[12] = *((char*)(& (x)) + 3);
    repl = false;
    return 13;
}


void Robot::recvData(char Data) {
    kickSensor = (((Data & 0x08) >> 4) == 1);
}
