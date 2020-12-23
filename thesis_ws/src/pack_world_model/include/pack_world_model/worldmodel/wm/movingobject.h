#ifndef MovingObject_H
#define MovingObject_H

#include <ostream>

#include <QDebug>
#include <QQueue>

#include <pack_util/math/mathtools.h>
#include <pack_world_model/worldmodel/wm/rawobject.h>

class CRawObject;

class MovingObject {
protected:
    int vanishedCounter;
    double lastInsight;
    double delayTime;
    bool kalmanEnabled;
    int lastFrameKalmanReset;
    int frameCounter;
    int stoppedFrames;
    QList<Vector2D> lastSpeeds;
    QList<double> lastAngularSpeeds;
public:
    MovingObject(bool resetToZero = true);
    CRawObject* observation;
    QQueue<CRawObject*> hist;

    //Final Specifications
    Vector2D pos;
    Vector2D dir;
    Vector2D vel;
    Vector2D acc;

    double angularVel;

    double inSight;
    double obstacleRadius;
    int cam_id;
    int lastFrameUpdated;
    bool modelObjStopped;
    Vector2D modelObjStopPos;
    Vector2D modelDir, modelCurDir;
    double modelSampleTime;
    double modelC0, modelC1, modelC2; //c2 * x^2 + c1 * x + c0
    //finding model variables
    int modelFrameCnt;
    double modelC2Sum, modelC2Ave;
    int modelC2Count;
    double modelDirC0, modelDirC1;
    Vector2D ballStopPos;
    QQueue<double> modelTimeBuffer;
    QQueue<CRawObject> modelObjBuffer;


    //Functions

    virtual void filter(int vanished);
    virtual void init();
    virtual void resetKalman();

    void updateDelayTime(double newDelayTime);

    void update(CRawObject raw);
    void update(MovingObject* obj);
    void findModel(double dt);
    void kalmanFilter();
    Vector2D predict(double time);
    Vector2D predictV(double time);
    Vector2D whereIsAtVel(Vector2D V);
    double whenIsAtVel(double L);
};

extern double observeTimeStep;

#endif // MovingObject_H
