#include <pack_world_model/worldmodel/wm/visionclient.h>

CVisionClient::CVisionClient() {
//    ourColor = _COLOR_BLUE;
    vcTimer = new QTime();
    vcTimer->start();
    lastCamera = -1;
    activeCameras = 1;
    frameCnt = 0;
}

CVisionClient::~CVisionClient() {
    delete vcTimer;
}

void CVisionClient::parse(const pack_msgs::msg::SSLVisionDetection::SharedPtr& packet) {
    lastCamera = -1;
    float ourTeamSide = (extern_isOurSideLeft == true) ? 1.0f : -1.0f;
    v[packet->camera_id].updated = false;


//    qDebug() << "M" << packet->camera_id;
    frameCnt ++;
    int id = packet->camera_id;
    lastCamera = id;
    v[id].cam_id = id;
    for (int i = 0; i < v[id].ball.count(); i++) {
        if (frameCnt - v[id].ball.at(i).frameCount > 2) {
            v[id].ball.removeAt(i);
        }
    }
    for (int t = 0; t < _MAX_NUM_PLAYERS; t++) {
        for (int i = 0; i < v[id].ourTeam[t].count(); i++) {
            if (frameCnt - v[id].ourTeam[t][i].frameCount > 15) {
                v[id].ourTeam[t].removeAt(i);
            }
        }
        for (int i = 0; i < v[id].oppTeam[t].count(); i++) {
            if (frameCnt - v[id].oppTeam[t][i].frameCount > 15) {
                v[id].oppTeam[t].removeAt(i);
            }
        }
    }
    /*
                    v[id].ball.clear();
            for(int i=0;i<_MAX_NUM_PLAYERS;i++){
                v[id].ourTeam[i].clear();
                v[id].oppTeam[i].clear();
                    }*/
    v[id].lastUpdateTime = vcTimer->elapsed();
    double t = packet->t_capture;
    double dt =  t - v[id].ltcapture;
    if (dt < 0.0) {
        dt = 0.05;
    }
    v[id].time = t;
    v[id].timeStep = dt;
    v[id].ltcapture = t;
    v[id].visionLatency = (packet->t_sent - packet->t_capture);

    if (!extern_cameraConfig.camera_0) {
        if (packet->camera_id == 0) return;
    }
    if (!extern_cameraConfig.camera_1) {
        if (packet->camera_id == 1) return;
    }
    if (!extern_cameraConfig.camera_2) {
        if (packet->camera_id == 2) return;
    }
    if (!extern_cameraConfig.camera_3) {
        if (packet->camera_id == 3) return;
    }
    if (!extern_cameraConfig.camera_4) {
        if (packet->camera_id == 4) return;
    }
    if (!extern_cameraConfig.camera_5) {
        if (packet->camera_id == 5) return;
    }
    if (!extern_cameraConfig.camera_6) {
        if (packet->camera_id == 6) return;
    }
    if (!extern_cameraConfig.camera_7) {
        if (packet->camera_id == 7) return;
    }


    v[id].updated = true;
    for (int i = 0; i < std::min(MAX_OBJECT, static_cast<int>(packet->balls.size())); i++) {
        if (packet->balls[i].confidence != -1
                && packet->balls[i].pos.x != 5000
                && packet->balls[i].pos.y != 5000) {
            CRawObject raw = CRawObject(frameCnt, Vector2D(packet->balls[i].pos.x * ourTeamSide, packet->balls[i].pos.y * ourTeamSide), 0, i
                                        , packet->balls[i].confidence, nullptr, id);
            for (int k = 0; k < v[id].ball.count(); k++) {
                if ((v[id].ball[k].pos - raw.pos).length() < 0.5) {
                    v[id].ball.removeAt(k);
                }
            }
            v[id].ball.append(raw);
        }
    }
    if (! packet->balls.empty()) {
        v[id].outofsight_ball = 0;
    }

    bool our_insight[_MAX_NUM_PLAYERS];
    bool opp_insight[_MAX_NUM_PLAYERS];
    for (int i = 0; i < _MAX_NUM_PLAYERS; i++) {
        our_insight[i] = opp_insight[i] = false;
    }

    for (const auto &u : packet->blue) {
        int rob_id = u.robot_id;
        if (v[id].ourTeam[rob_id].count() >= MAX_OBJECT) {
            continue;
        }
        CRawObject raw = CRawObject(frameCnt, Vector2D(u.pos.x * ourTeamSide, u.pos.y * ourTeamSide),
                                    u.orientation * 180.0f / M_PI + (1.0 - ourTeamSide) * 90.0
                                    , rob_id , u.confidence, nullptr, id);
        for (int k = 0; k < v[id].ourTeam[rob_id].count(); k++) {
            if ((v[id].ourTeam[rob_id][k].pos - raw.pos).length() < 0.5) {
                v[id].ourTeam[rob_id].removeAt(k);
            }
        }
        v[id].ourTeam[rob_id].append(raw);
        v[id].outofsight_ourTeam[rob_id] = 0;
        our_insight[rob_id] = true;
    }

    for (const auto &i : packet->yellow) {
        int rob_id = i.robot_id;
        if (v[id].oppTeam[rob_id].count() >= MAX_OBJECT) {
            continue;
        }
        CRawObject raw = CRawObject(frameCnt, Vector2D(i.pos.x * ourTeamSide, i.pos.y * ourTeamSide),
                                    i.orientation * 180.0f / M_PI + (1.0 - ourTeamSide) * 90.0
                                    , rob_id , i.confidence, nullptr, id);
        for (int k = 0; k < v[id].oppTeam[rob_id].count(); k++) {
            if ((v[id].oppTeam[rob_id][k].pos - raw.pos).length() < 0.5) {
                v[id].oppTeam[rob_id].removeAt(k);
            }
        }
        v[id].oppTeam[rob_id].append(raw);
        v[id].outofsight_oppTeam[rob_id] = 0;
        opp_insight[rob_id] = true;
    }


    for (int i = 0; i < _MAX_NUM_PLAYERS; i++) {
        if (!our_insight[i]) {
            v[id].outofsight_ourTeam[i]++;
        }
        if (!opp_insight[i]) {
            v[id].outofsight_oppTeam[i]++;
        }
    }
}

inline float inSightReduce(float v, int n) {
    if (n > 0) {
        return v / ((float) n * n);
    }
    return v;
}

///////////////////////////////////////////

void CVisionClient::merge(int camera_count) {

    res.reset();
    res.time = 0.0;
    res.timeStep = 0;
    res.ltcapture = 0;
    for (int i = 0; i < camera_count; i++) {
        if (! v[i].updated ) continue;
        res.time += v[i].time;
        res.timeStep += v[i].timeStep;
        res.ltcapture += v[i].ltcapture;
        res.visionLatency += v[i].visionLatency;
        //Match between res and v[i]
        for (int j = 0; j < v[i].ball.count(); j++) {
//            if (v[i].ball[j].confidence <= 0) continue;
            int best = -1;
            double dist = 0;
            double minDist = 0;
            for (int k = 0; k < res.ball.count(); k++) {
                dist = (v[i].ball[j].pos - res.ball[k].pos).length();
                if ((dist < minDist) || (best == -1)) {
                    minDist = dist;
                    best = k;
                }
            }
            if ((best == -1) || (minDist > 0.5)) {
                res.ball.append(v[i].ball[j]);
                res.ball.last().mergeCount = 0;
            } else {
                res.ball[best].pos += v[i].ball[j].pos;
                res.ball[best].mergeCount ++;
            }
        }

        for (int t = 0; t < _MAX_NUM_PLAYERS; t++) {
            for (int j = 0; j < v[i].ourTeam[t].count(); j++) {
                //if (v[i].ourTeam[t][j].confidence <= 0) continue;
                int best = -1;
                double dist = 0;
                double minDist = 0;
                for (int k = 0; k < res.ourTeam[t].count(); k++) {
                    dist = (v[i].ourTeam[t][j].pos - res.ourTeam[t][k].pos).length();
                    if ((dist < minDist) || (best == -1)) {
                        minDist = dist;
                        best = k;
                    }
                }
                if ((best == -1) || (minDist > 0.5)) {
                    res.ourTeam[t].append(v[i].ourTeam[t][j]);
                    res.ourTeam[t].last().mergeCount = 0;
                } else {
                    res.ourTeam[t][best].pos += v[i].ourTeam[t][j].pos;
                    res.ourTeam[t][best].mergeCount ++;
                }
            }
        }
        for (int t = 0; t < _MAX_NUM_PLAYERS; t++) {
            for (int j = 0; j < v[i].oppTeam[t].count(); j++) {
                //if (v[i].oppTeam[t][j].confidence <= 0) continue;
                int best = -1;
                double dist = 0;
                double minDist = 0;
                for (int k = 0; k < res.oppTeam[t].count(); k++) {
                    dist = (v[i].oppTeam[t][j].pos - res.oppTeam[t][k].pos).length();
                    if ((dist < minDist) || (best == -1)) {
                        minDist = dist;
                        best = k;
                    }
                }
                if ((best == -1) || (minDist > 0.5)) {
                    res.oppTeam[t].append(v[i].oppTeam[t][j]);
                    res.oppTeam[t].last().mergeCount = 0;
                } else {
                    res.oppTeam[t][best].pos += v[i].oppTeam[t][j].pos;
                    res.oppTeam[t][best].mergeCount ++;
                }
            }
        }
    }
    for (int k = 0; k < res.ball.count(); k++) {
        res.ball[k].pos /= (float)(res.ball[k].mergeCount + 1);
    }
    for (int t = 0; t < _MAX_NUM_PLAYERS; t++) {
        for (int k = 0; k < res.ourTeam[t].count(); k++) {
            res.ourTeam[t][k].pos /= (float)(res.ourTeam[t][k].mergeCount + 1);
        }
        for (int k = 0; k < res.oppTeam[t].count(); k++) {
            res.oppTeam[t][k].pos /= (float)(res.oppTeam[t][k].mergeCount + 1);
        }
    }

    res.time /= camera_count;
    res.timeStep /= camera_count;
    res.ltcapture /= camera_count;
    res.visionLatency /= camera_count;

}
