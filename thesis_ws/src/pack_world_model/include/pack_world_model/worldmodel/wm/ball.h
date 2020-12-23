//
// Created by parsian-ai on 10/13/17.
//

#ifndef PACK_WORLD_MODEL_BALL_H
#define PACK_WORLD_MODEL_BALL_H

#include <QDebug>
#include <QQueue>
#include <QTime>

#include <pack_world_model/worldmodel/wm/movingobject.h>
#include <pack_world_model/worldmodel/util/balltracker.h>
#include <pack_util/core/base.h>

class CBall : public MovingObject {
private:
//    CField field;
//    Rect2D fieldRect;
    //Vector2D replPos, replVel;
    BallTracker* tracker;
//  BallTracker* kalmantracker;
//  RobotTracker* tracker;
    int old;
    int blindness;
    QList<Vector2D> ballHist;
    QList<Vector2D> ballLinearHist;
public:
    static const double radius;
    CBall(bool noKalman = false);
    ~CBall();
    virtual void filter(int vanished);
    virtual void init();
    virtual void resetKalman();
    void setReplace(Vector2D newPos, Vector2D newVel); //used in simulation
    int replacementPacket(char* buf);
    Vector2D whereBallSpeedIs(double speed);
    Vector2D ballSpeedAt(double dist);
    double getBallAcc();
    double whenBallReachToPoint(double dist);
    Vector2D getPosInFuture(double _t);
    int ballInsistanceCounter;

    double modelWhenIsObjAt(double dToObj);
    double getVel();

    //model result functions
    Vector2D getDir();
    Vector2D getStopPos();
    Vector2D getProjectionOfPointOnBallVeclocityDirection(Vector2D point, bool usepath = false);

    bool isKicked();
    bool isPassed();
    int  trackerLastBestElementID;
    int  trakerInsistCounter;
    bool elementNotInSight;

    Property(Vector2D, ReplPos, replPos);
    Property(Vector2D, ReplVel, replVel);
    Property(bool, Replaced, repl);
};



#endif //PACK_WORLD_MODEL_BALL_H
