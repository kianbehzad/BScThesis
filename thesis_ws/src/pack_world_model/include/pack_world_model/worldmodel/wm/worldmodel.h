//
// Created by parsian-ai on 9/19/17.
//

#ifndef PACK_WORLD_MODEL_WORLDMODEL_H
#define PACK_WORLD_MODEL_WORLDMODEL_H

#include "QObject"
#include "QDebug"

#include "pack_msgs/msg/world_model.hpp"
#include "pack_msgs/msg/ssl_vision_detection.hpp"
#include "pack_msgs/msg/ssl_vision_geometry.hpp"
#include "pack_msgs/msg/robot.hpp"
#include "pack_msgs/msg/ssl_vision_detection_robot.hpp"
#include "pack_msgs/msg/ssl_vision_detection_ball.hpp"

#include <pack_world_model/worldmodel/wm/visionclient.h>
#include <pack_world_model/worldmodel/wm/halfworld.h>
#include <pack_world_model/worldmodel/wm/ball.h>
#include <pack_world_model/worldmodel/wm/robot.h>
#include "pack_world_model/worldmodel/util/config.h"

#include "pack_util/core/knowledge.h"

class WorldModel {
public:
    WorldModel();
    ~WorldModel();

    void updateDetection(const pack_msgs::msg::SSLVisionDetection::SharedPtr&);
    void updateGeom(const pack_msgs::msg::SSLVisionGeometry::SharedPtr&);
    void execute();
    void merge(int frame);
    void init();
    void setMode(bool isSimulation);

    double vForwardCmd[12], vNormalCmd[12], vAngCmd[12];

    pack_msgs::msg::WorldModel::SharedPtr getParsianWorldModel();
    Robot* them[_MAX_NUM_PLAYERS];
    Robot* us[_MAX_NUM_PLAYERS];

private:
    pack_msgs::msg::Robot rosRobots[_MAX_NUM_PLAYERS * 2];
    pack_msgs::msg::Robot rosBall;
    CVisionClient *vc;

    CHalfWorld w;
    CHalfWorld mergedHalfWorld;

    CBall* ball;
    bool simulationMode;
    void run();
    void update(CHalfWorld*);
    void testFunc(const pack_msgs::msg::SSLVisionDetection::SharedPtr & packet);
    void printRobotInfo(const pack_msgs::msg::SSLVisionDetectionRobot &robot);

    void toParsianMessage(const Robot* _robot, int id);
    void toParsianMessage(const CBall* _ball);


    pack_msgs::msg::SSLVisionDetection::SharedPtr detection;

    double visionFPS;
    double visionLatency;
    double visionTimestep;
    double visionProcessTime;

    int packs;


};


#endif //PACK_WORLD_MODEL_WORLDMODEL_H
