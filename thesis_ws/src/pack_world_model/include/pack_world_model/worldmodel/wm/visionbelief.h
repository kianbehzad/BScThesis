//
// Created by parsian-ai on 10/5/17.
//

#ifndef PACK_WORLD_MODEL_VISIONBELIEF_H
#define PACK_WORLD_MODEL_VISIONBELIEF_H

#include <QList>
#include <pack_world_model/worldmodel/wm/movingobject.h>

#include "pack_util/core/knowledge.h"

const int _MAX_NUM_PLAYERS = knowledge::MAX_ROBOT_NUM;

class CVisionBelief {
public:
    QList<CRawObject> ourTeam[_MAX_NUM_PLAYERS];
    QList<CRawObject> oppTeam[_MAX_NUM_PLAYERS];
    QList<CRawObject> ball;
    double visionLatency;
    double timeStep, ltcapture;
    double time;
    int cam_id;

    //count of frames that each object was out of sight
    int outofsight_ourTeam[_MAX_NUM_PLAYERS];
    int outofsight_oppTeam[_MAX_NUM_PLAYERS];
    int outofsight_ball;

    bool updated; //indicates that camera is ever updated or not
    int lastUpdateTime;
    CVisionBelief();
    void reset();
};

#endif //PACK_WORLD_MODEL_VISIONBELIEF_H
