#include "pack_world_model/worldmodel/wm/worldmodel.h"




WorldModel::WorldModel() {
//    ROS_INFO("const");
    vc = new CVisionClient();
//    vc->ourSide = _SIDE_LEFT;
    simulationMode = true;
    visionFPS = 61.0;
    ball = new CBall(false);
    for (int i = 0; i < _MAX_NUM_PLAYERS; i++) {
        us[i] = new Robot(i, true);
        them[i] = new Robot(i, false);
        vForwardCmd[i] = 0;
        vNormalCmd [i] = 0;
        vAngCmd    [i] = 0;
    }

    packs = 0;

}

WorldModel::~WorldModel() {
    delete vc;
    vc = nullptr;
}

void WorldModel::updateDetection(const pack_msgs::msg::SSLVisionDetection::SharedPtr& _detection) {
    detection = _detection;
}

void WorldModel::execute() {
    run();
}

void WorldModel::toParsianMessage(const Robot* _robot, int id) {
    rosRobots[id].camera_id = static_cast<unsigned char>(_robot->cam_id);
    rosRobots[id].id = static_cast<unsigned char>(_robot->id);
    rosRobots[id].pos = _robot->pos.toParsianMessage();
    rosRobots[id].acc = _robot->acc.toParsianMessage();
    rosRobots[id].vel = _robot->vel.toParsianMessage();
    rosRobots[id].angular_vel = _robot->angularVel;
    rosRobots[id].dir = _robot->dir.toParsianMessage();
    rosRobots[id].in_sight = _robot->inSight;
    rosRobots[id].obstacle_radius = _robot->obstacleRadius;
}

void WorldModel::toParsianMessage(const CBall* _ball) {
    rosBall.camera_id = static_cast<unsigned char>(_ball->cam_id);
    rosBall.pos = _ball->pos.toParsianMessage();
    rosBall.acc = _ball->acc.toParsianMessage();
    rosBall.vel = _ball->vel.toParsianMessage();
    rosBall.angular_vel = _ball->angularVel;
    rosBall.dir = _ball->dir.toParsianMessage();
    rosBall.in_sight = _ball->inSight;
    rosBall.obstacle_radius = _ball->obstacleRadius;

}

pack_msgs::msg::WorldModel::SharedPtr WorldModel::getParsianWorldModel() {
//    if (this->ball == nullptr) return rosWM;

    pack_msgs::msg::WorldModel::SharedPtr rosWM{new pack_msgs::msg::WorldModel};
    rosWM->our.reserve(_MAX_NUM_PLAYERS);
    rosWM->opp.reserve(_MAX_NUM_PLAYERS);

    if (ball->inSight > 0) {
        toParsianMessage(ball);
        rosWM->ball = rosBall;
    }


    for (int i = 0; i < _MAX_NUM_PLAYERS; ++ i) {
        if (us[i]->isActive()) {
            toParsianMessage(us[i], i);
            if (!extern_isOurColorYellow) rosWM->our.push_back(rosRobots[i]);
            else rosWM->opp.push_back(rosRobots[i]);

        }
        if (them[i]->isActive()) {
            toParsianMessage(them[i], i + 12);
            rosWM->opp.push_back(rosRobots[i + 12]);
            if (!extern_isOurColorYellow) rosWM->opp.push_back(rosRobots[i + 12]);
            else rosWM->our.push_back(rosRobots[i + 12]);
        }
    }

    return rosWM;
}

void WorldModel::printRobotInfo(const pack_msgs::msg::SSLVisionDetectionRobot &robot) {
    printf("CONF=%4.2f ", robot.confidence);
    printf("ID=%3d ", robot.robot_id);
    printf(" HEIGHT=%6.2f POS=<%9.2f,%9.2f> ", robot.height, robot.pos.x, robot.pos.y);
    printf("ANGLE=%6.3f ", robot.orientation);

    printf("RAW=<%8.2f,%8.2f>\n", robot.pixel_pos.x, robot.pixel_pos.y);
}

void WorldModel::testFunc(const pack_msgs::msg::SSLVisionDetection::SharedPtr &detection) {
    printf("-----Received Wrapper Packet---------------------------------------------\n");
    //see if the packet contains a robot detection frame:

    //Display the contents of the robot detection results:
    double t_now = 0 /*= GetTimeSec()*/;

    printf("-[Detection Data]-------\n");
    //Frame info:
    printf("Camera ID=%d FRAME=%d T_CAPTURE=%.4f\n", detection->camera_id, detection->frame_number, detection->t_capture);

    printf("SSL-Vision Processing Latency                   %7.3fms\n", (detection->t_sent - detection->t_capture) * 1000.0);
    printf("Network Latency (assuming synched system clock) %7.3fms\n", (t_now - detection->t_sent) * 1000.0);
    printf("Total Latency   (assuming synched system clock) %7.3fms\n", (t_now - detection->t_capture) * 1000.0);
    auto balls_n = static_cast<int>(detection->balls.size());
    auto robots_blue_n = static_cast<int>(detection->blue.size());
    auto robots_yellow_n = static_cast<int>(detection->yellow.size());

    //Ball info:
    for (int i = 0; i < balls_n; i++) {
        pack_msgs::msg::SSLVisionDetectionBall ball = detection->balls[i];
        printf("-Ball (%2d/%2d): CONF=%4.2f POS=<%9.2f,%9.2f> ", i + 1, balls_n, ball.confidence, ball.pos.x, ball.pos.y);
        printf("RAW=<%8.2f,%8.2f>\n", ball.pixel_pos.x, ball.pixel_pos.y);
    }

    //Blue robot info:
    for (int i = 0; i < robots_blue_n; i++) {
        pack_msgs::msg::SSLVisionDetectionRobot robot = detection->blue[i];
        printf("-Robot(US) (%2d/%2d): ", i + 1, robots_blue_n);
        printRobotInfo(robot);
    }

    //Yellow robot info:
    for (int i = 0; i < robots_yellow_n; i++) {
        pack_msgs::msg::SSLVisionDetectionRobot robot = detection->yellow[i];
        printf("-Robot(THEM) (%2d/%2d): ", i + 1, robots_yellow_n);
        printRobotInfo(robot);
    }
}

void WorldModel::merge(int frame) {
    packs = 0;
    for (int i = 0 ; i < _MAX_NUM_PLAYERS ; i++) {
        mergedHalfWorld.vForwardCmd[i] = vForwardCmd[i];
        mergedHalfWorld.vNormalCmd[i] = vNormalCmd[i];
        mergedHalfWorld.vAngCmd[i] = vAngCmd[i];
    }

    if (vc->lastCamera < CAMERA_NUM && vc->lastCamera >= 0) {
        vc->merge(extern_cameraConfig.cam_num);
        mergedHalfWorld.currentFrame = frame;
        mergedHalfWorld.update(&(vc->res));
        mergedHalfWorld.vanishOutOfSights();
    }

    // UPDATE WM
    this->update(&mergedHalfWorld);


}


// This Function Run in a Loop
void WorldModel::run() {
    if (vc == nullptr) {
        return;
    }
    vc->parse(detection);

}

void WorldModel::update(CHalfWorld* w0) {
    w.update(w0);


    if (w.ball.count() > 0) {
        ball->update(w.ball[0]);
    } else {
        ball->inSight = 0.0;
    }
    for (int i = 0; i < _MAX_NUM_PLAYERS; i++) {
        if (!w.ourTeam[i].isEmpty()) {
            us[i]->update(w.ourTeam[i][0]);
        } else {
            us[i]->inSight = 0.0;
        }
        if (!w.oppTeam[i].isEmpty()) {
            them[i]->update(w.oppTeam[i][0]);
        } else {
            them[i]->inSight = 0.0;
        }
    }

}

void WorldModel::setMode(bool isSimulation) {
    simulationMode = isSimulation;
}

