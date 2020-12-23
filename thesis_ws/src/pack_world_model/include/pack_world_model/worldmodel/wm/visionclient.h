#ifndef VISIONCLIENT_H
#define VISIONCLIENT_H

#include <fstream>
#include <ostream>
#include <vector>

#include <QByteArray>
#include <QDebug>
#include <QTime>
#include <QList>

#include <pack_util/geom/geom.h>
#include <pack_util/math/mathtools.h>
#include "pack_util/core/knowledge.h"

#include "pack_msgs/msg/ssl_vision_detection.hpp"

#include <pack_world_model/worldmodel/wm/visionbelief.h>
#include <pack_world_model/worldmodel/util/config.h>


#define CAMERA_NUM 8
#define OUT_OF_SIGHT_THRESHOLD 40

class CVisionClient {
public:
    QTime *vcTimer;
    CVisionBelief v[CAMERA_NUM];
//    CVisionBelief mv[CAMERA_NUM];
    CVisionBelief res;
    double sampleT;
    int lastCamera;
    int activeCameras;
    int frameCnt;

    std::vector<Vector2D> boundaries[CAMERA_NUM];

    CVisionClient();
    ~CVisionClient();

    void parse(const pack_msgs::msg::SSLVisionDetection::SharedPtr& packet);
    void merge(int camera_count = CAMERA_NUM);
};

#define MAX_OBJECT 5
#define __RECEIVE_ROBOTS_DATA(__COLOR__,__TEAM__)\
for (int i = 0; i < packet->##__COLOR__##.size();i++) { \
    int rob_id = packet->##__COLOR__##[i].robot_id; \
    if (v[id].__TEAM__##Team[rob_id].count() >= MAX_OBJECT) continue; \
    CRawObject raw = CRawObject(frameCnt, Vector2D(packet->##__COLOR__##[i].pos.x / 1000.0f,packet->##__COLOR__##[i].pos.y / 1000.0f), \
                                packet->##__COLOR__##[i].orientation*180.0f/M_PI \
                                ,i ,packet->##__COLOR__##[i].confidence, nullptr, id); \
    for (int k=0;k<v[id].__TEAM__##Team[rob_id].count();k++) \
    { \
            if ((v[id].__TEAM__##Team[rob_id][k].pos - raw.pos).length() < 0.5) \
            { \
                    v[id].__TEAM__##Team[rob_id].removeAt(k); \
            } \
    } \
    v[id].__TEAM__##Team[rob_id].append(raw); \
    v[id].outofsight_##__TEAM__##Team[rob_id] = 0; \
    __TEAM__##_insight[rob_id] = true; \
}


#endif // VISIONCLIENT_H
