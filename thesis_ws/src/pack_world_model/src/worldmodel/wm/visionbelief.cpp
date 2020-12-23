//
// Created by parsian-ai on 10/5/17.
//

#include <pack_world_model/worldmodel/wm/visionbelief.h>

CVisionBelief::CVisionBelief()
    : visionLatency() , timeStep() , ltcapture() , time() , cam_id(), outofsight_ball() , lastUpdateTime() {
    reset();
    updated = false;
    for (int  i = 0; i < _MAX_NUM_PLAYERS; i++) {
        outofsight_ourTeam[i] = outofsight_oppTeam[i] = 0;
    }
}

void CVisionBelief::reset() {
    timeStep = 0;
    visionLatency = 0;
    for (int i = 0; i < _MAX_NUM_PLAYERS; i++) {
        ourTeam[i].clear();
        outofsight_ourTeam[i] = 0;
        oppTeam[i].clear();
        outofsight_oppTeam[i] = 0;
    }
    ball.clear();
    outofsight_ball = 0;
}