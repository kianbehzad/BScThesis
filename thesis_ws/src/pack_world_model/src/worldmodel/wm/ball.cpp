#include <pack_world_model/worldmodel/wm/ball.h>


const double _ROBOT_RADIUS = 0.09;


const double CBall::radius = 0.0215;

CBall::CBall(bool noKalman) : MovingObject() {
    init();
    if (!noKalman) {
        tracker = new BallTracker(0.0);
//    kalmantracker = new BallTracker(1.0);
        tracker->reset();
//    kalmantracker->reset();
    } else {
        tracker = nullptr;
//    kalmantracker = NULL;
    }

    blindness = 8;
    old = 0;
}

void CBall::resetKalman() {
    if (tracker != nullptr) {
        delete tracker;
        tracker = new BallTracker(0.0);
        tracker->reset();

//    delete kalmantracker;
//    kalmantracker = new BallTracker(1.0);
//    kalmantracker->reset();
    }
}

CBall::~CBall()
    = default;

void CBall::init() {
    repl = false;
    modelC2Sum = 0;
    modelC2Count = 0;
    modelC2Ave = 0;
    modelSampleTime = 0.008;//0.016;
    modelFrameCnt = 0;
    vel.assign(0, 0);
    elementNotInSight = true;
    float Margin = 0.2;
    ballInsistanceCounter = 1;
//    fieldRect.assign(-field._FIELD_WIDTH/2.0 - Margin,field._FIELD_HEIGHT/2.0 + Margin,field._FIELD_WIDTH+2*Margin,field._FIELD_HEIGHT+2*Margin);
}

void CBall::setReplace(Vector2D newPos, Vector2D newVel) {
    repl = true;
    replPos = newPos;
    replVel = newVel;
}

int CBall::replacementPacket(char* buf) {
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
    x = static_cast<float>(replVel.x);
    buf[9]  = *((char*)(& (x)));
    buf[10] = *((char*)(& (x)) + 1);
    buf[11] = *((char*)(& (x)) + 2);
    buf[12] = *((char*)(& (x)) + 3);
    x = static_cast<float>(replVel.y);
    buf[13]  = *((char*)(& (x)));
    buf[14] = *((char*)(& (x)) + 1);
    buf[15] = *((char*)(& (x)) + 2);
    buf[16] = *((char*)(& (x)) + 3);
    repl = false;
    return 17;
}

double CBall::getVel() {
    if (modelObjStopped) {
        return vel.length();
    }
    return fabs(modelC1 / modelSampleTime);
}
Vector2D CBall::getPosInFuture(double _t) {
    return pos + (-0.5 * (getBallAcc()) * _t * _t + vel.length() * _t) * vel.norm();
}

double CBall::modelWhenIsObjAt(double dToObj) {
    if (modelC2 == 0.0) {
        double r = -modelSampleTime * (modelC0 - dToObj) / modelC1;
        return r;
    }


    double delta = modelC1 * modelC1 - 4.0 * (modelC0 - dToObj) * modelC2;
    if (delta > 0) {
        delta = sqrt(delta);
        double r1 = modelSampleTime * (-modelC1 - delta) / (2.0 * modelC2);
        double r2 = modelSampleTime * (-modelC1 + delta) / (2.0 * modelC2);
        if (r1 < r2) {
            if (r1 > 0) {
                return r1;
            }
            return r2;
        }

        if (r2 > 0) {
            return r2;
        }
        return r1;

    } else {
        double tmax = -(modelC1) / (2.0 * modelC2);
        double dmax = modelC2 * tmax * tmax + modelC1 * tmax + modelC0;
        tmax += tmax * fabs(dToObj - dmax) / dmax;
        return modelSampleTime * tmax;
    }

}

Vector2D CBall::getDir() {
    return modelDir;
}

Vector2D CBall::getStopPos() {
    return pos + vel.norm() * vel.r2() / (2.0 * getBallAcc());
//    if (modelC2<0) return modelObjStopPos;
//    return Vector2D();
}

Vector2D CBall::getProjectionOfPointOnBallVeclocityDirection(Vector2D point, bool usepath) {
    if (usepath && modelDir.valid()) {
        Line2D l(pos, modelDir + pos);
        Vector2D proj = l.projection(point);
        return proj;
    }

    Line2D l(pos, vel + pos);
    Vector2D proj = l.projection(point);
    return proj;

}

bool CBall::isKicked() {
    return (vel.length() > 1.0);
}

bool CBall::isPassed() {
    return (vel.length() > 0.5);
}



void CBall::filter(int vanished) {
    auto * ballLast = new CRawObject;
    if (hist.size() > 100) {
        hist.pop_front();
    }
    ballLast->pos = pos;
    ballLast->dir = dir;
    hist.push_back(ballLast);
    if (ballLinearHist.size() > 20) {
        ballLinearHist.pop_front();
    }
    ballLinearHist.push_back(ballLast->pos);
    bool reflected = false;
    for (int k = 0; k < ballLinearHist.count(); k++) {
        if (fabs(Vector2D::angleBetween((ballLinearHist.back() - ballLinearHist[k]), (ballLinearHist[0] - ballLinearHist[k])).degree()) < 120.0
                && (((ballLinearHist.back() - ballLinearHist[k]).length() > 0.07) && ((ballLinearHist[0] - ballLinearHist[k]).length() > 0.07))
           ) {
            reflected = true;
        }
    }
    if (reflected) {
        resetKalman();
//    qDebug()<<"Reflection";
        ballLinearHist.clear();
//        debug("RESET!!!", D_ERROR);
    }

//    if (vel.length()>0.01)
//    {
//        acc = -vel.norm() * Gravity * BallFriction();
//    }

    if (tracker == nullptr) {
        MovingObject::filter(vanished);
        return;
    }
    if (vanished <= 0) {
        vraw v;
        v.angle = 0.0;
        v.conf  = observation->confidence;
        v.pos   = observation->pos;
        v.timestamp = observation->time;

//    kalmantracker->observeNew(v,0,0);
//    acc = kalmantracker->acceleration(8*getFramePeriod());

//    knowledge->plotWidgetCustom[5] = acc.y;
//    knowledge->plotWidgetCustom[6] = sign(acc.y);

//    tracker->observeNew(v,sign(acc.x),sign(acc.y));
        tracker->observeNew(v, 0, 0);
//    pos = tracker->position(1*getFramePeriod());
        vel = tracker->velocity(8 * getFramePeriod());
//    knowledge->plotWidgetCustom[1] = vel.y;
        pos = observation->pos;
//    knowledge->plotWidgetCustom[0] = pos.y;



        acc = tracker->acceleration(1 * getFramePeriod());
        dir = Vector2D(0, 0);
        angularVel = 0;

        inSight = 1.0;
//        qDebug() << "vel = " << vel.length() << " Time Stamp= " << v.timestamp;
        old ++;
    } else {
        if (vanished < blindness) {
            vraw v;
            v.conf  = 1;
            v.angle = observation->dir.th().degree();
            v.pos   = observation->pos;
            v.timestamp = observation->time;

            pos = v.pos;

//      pos = tracker->position((1+vanished)*getFramePeriod());
            vel = tracker->velocity((8 + vanished) * getFramePeriod());
            acc = tracker->acceleration((1 + vanished) * getFramePeriod());
            dir = Vector2D(0, 0);
            angularVel = 0;

            inSight = 0.5;

        } else {
            tracker->reset();
//      kalmantracker->reset();
            vel *= 0.8;
            angularVel *= 0.8;
            inSight = 0.0;
        }
    }

}

double CBall::whenBallReachToPoint(double dist) {
    double v2 = vel.length() * vel.length();
    double a = getBallAcc();
    double _time = 0;
    if ((v2 - 2 * a * dist) < 0) {
        return -1;
    }

    _time = (vel.length() - sqrt(v2 - 2 * a * dist)) / a;
    return _time;

}

Vector2D CBall::whereBallSpeedIs(double speed) {
    if (vel.length() < speed) {
        return pos;
    }
    return pos + vel.norm() * (vel.r2() - speed * speed) / (2.0 * acc.length());
}

Vector2D CBall::ballSpeedAt(double dist) {
    if (dist < 0) return Vector2D{0.0, 0.0};
    double v2 = vel.r2() - 2.0 * acc.length() * dist;
    if (v2 < 0) return Vector2D{0.0, 0.0};
    return vel.norm() * sqrt(v2);
}

double CBall::getBallAcc() {
    // return Gravity*BallFriction();
//    return this->acc.length();
    return 0.1;
}
