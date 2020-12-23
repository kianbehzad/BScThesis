//
// Created by parsian-ai on 10/13/17.
//

#ifndef PACK_WORLD_MODEL_HALFWORLD_H
#define PACK_WORLD_MODEL_HALFWORLD_H

#include <QList>
#include <QString>
#include <QMap>

#include <pack_util/geom/geom.h>
#include <pack_world_model/worldmodel/wm/visionbelief.h>
#include <pack_world_model/worldmodel/wm/robot.h>
#include <pack_world_model/worldmodel/wm/ball.h>


class CHalfWorld {
public:
    Vector2D positioningPoints[_MAX_NUM_PLAYERS];
    int positioningPointsCount;

    double vForwardCmd[12], vNormalCmd[12], vAngCmd[12];
    quint8 game_state;
    quint8 game_mode;

    CVisionBelief belief;
    QList<Robot*> ourTeam[_MAX_NUM_PLAYERS];
    QList<Robot*> oppTeam[_MAX_NUM_PLAYERS];
    QList<CBall*> ball;
    QString ourRole[_MAX_NUM_PLAYERS];
    QString oppRole[_MAX_NUM_PLAYERS];
    CVisionBelief* c;
    int currentFrame;
    int playmakerID;
    bool closing;
    CHalfWorld();
    void track(QList<CRawObject>& p0, QList<CRawObject>& p);
    void update(QList<Robot*>& robot, CVisionBelief* v, QList<CRawObject>& robot0, int id, bool our);
    void update(QList<CBall*>& ball, CVisionBelief* v);
    void update(CVisionBelief* v);
    void update(CHalfWorld* w);
    void vanishOutOfSights();
    void selectBall(Vector2D pos);
    QMap<QString, QString> knowledgeVars;
};


#endif //PACK_WORLD_MODEL_HALFWORLD_H
