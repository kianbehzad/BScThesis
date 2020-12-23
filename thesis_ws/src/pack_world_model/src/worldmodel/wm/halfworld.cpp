#include <pack_world_model/worldmodel/wm/halfworld.h>


CHalfWorld::CHalfWorld() {
    c = new CVisionBelief();
    currentFrame = 0;
    playmakerID = -1;
}

void CHalfWorld::vanishOutOfSights() {
    if (ball.count() > 0) {
        if (ball[0]->inSight <= 0) {
            ball[0]->inSight = 0.5;
            if (ball.count() > 1) {
                for (int i = 1; i < ball.count(); i++) {
                    if (ball[i]->inSight > 0.0) {
                        ball.swap(0, i);
                        break;
                    }
                }
            }
        }
    }
    for (int i = 1; i < ball.count(); i++) {
        if (ball[i]->inSight <= 0.0) {
            delete ball[i];
            ball.removeAt(i);
            i--;
        }
    }
}

void CHalfWorld::update(QList<CBall *>& ball, CVisionBelief* v) {

    if (ball.count() == 0) {
        for (int i = 0; i < v->ball.count(); i++) {
            ball.append(new CBall(false));
            ball.last()->update(v->ball[i]);
        }
    } else {
        QList<bool> flag;
        for (int i = 0; i < ball.count(); i++) {
            flag.append(false);
        }
        for (int i = 0; i < v->ball.count(); i++) {
            double min_d = 1e5;
            int k = -1;
            for (int j = 0; j < ball.count(); j++) {
                if (flag[j]) {
                    continue;
                }
                double d = (ball[j]->pos + ball[j]->vel * getFramePeriod() - v->ball[i].pos).length();
                if (d < min_d) {
                    min_d = d;
                    k = j;
                }
            }
            if (k != -1) {
                if (min_d > ball[k]->vel.length() * getFramePeriod() * 2.0 + 1.0) {
                    k = -1;
                }
            }
            if (k != -1) {
                flag[k] = true;
                ball[k]->update(v->ball[i]);
            } else {
                flag.append(true);
                ball.append(new CBall(false));
                ball.last()->update(v->ball[i]);
            }
        }
        for (int i = 0; i < flag.count(); i++) {
            if (!flag[i]) {
                ball[i]->update(CRawObject(0, ball[i]->pos, 0.0, -1, 0.0, nullptr, v->cam_id, v->time));
            }
        }
    }
    /////WatchDog
    if (ball.count() > 0) {
        double minVel = 0.6;
        if (ball[0]->vel.length() < minVel)
            for (int k = 1; k < ball.count(); k++) {
                if (ball[k]->vel.length() >= minVel) {
                    ball.swap(k, 0);
                    break;
                }
            }
    }
}

void CHalfWorld::update(CVisionBelief *v) {

    belief = *v;
    QList<CRawObject> p0;
    update(ball, v);

    for (int j = 0; j < _MAX_NUM_PLAYERS; j++) {
        if (ourTeam[j].count() == 0) {
            ourTeam[j].append(new Robot(j, true, false));
        } else {
            if (v->ourTeam[j].count() > 0) {
                ourTeam[j][0]->vForwardCmd = vForwardCmd[j];
                ourTeam[j][0]->vNormalCmd  = vNormalCmd[j];
                ourTeam[j][0]->vAngCmd     = vAngCmd[j];
                ourTeam[j][0]->update(v->ourTeam[j][0]);
            } else if (ourTeam[j][0]->inSight > 0) {
                ourTeam[j][0]->vForwardCmd = vForwardCmd[j];
                ourTeam[j][0]->vNormalCmd  = vNormalCmd[j];
                ourTeam[j][0]->vAngCmd     = vAngCmd[j];
                ourTeam[j][0]->update(CRawObject(0, ourTeam[j][0]->pos, ourTeam[j][0]->dir.th().degree(), -1, 0.0,
                                                 nullptr, v->cam_id, v->time));
            }
        }
    }
    for (int j = 0; j < _MAX_NUM_PLAYERS; j++) {
        if (oppTeam[j].count() == 0) {
            oppTeam[j].append(new Robot(j, false, false));
        } else {
            if (v->oppTeam[j].count() > 0) {
                oppTeam[j][0]->update(v->oppTeam[j][0]);
            } else if (oppTeam[j][0]->inSight > 0) {
                oppTeam[j][0]->update(CRawObject(0, oppTeam[j][0]->pos, oppTeam[j][0]->dir.th().degree(), -1, 0.0,
                                                 nullptr, v->cam_id, v->time));
            }
        }
    }
}

void CHalfWorld::update(CHalfWorld *w) {
    for (int k = 0; k < ball.count(); k++) {
        delete ball[k];
    }
    ball.clear();
    for (int k = 0; k < w->ball.count(); k++) {
        ball.append(new CBall(true));
        ball.back()->update(w->ball[k]);
    }
    for (int j = 0; j < _MAX_NUM_PLAYERS; j++) {
        for (int k = 0; k < ourTeam[j].count(); k++) {
            delete ourTeam[j][k];
        }
        ourTeam[j].clear();
        for (int k = 0; k < w->ourTeam[j].count(); k++) {
            ourTeam[j].append(new Robot(j, true, true));
            ourTeam[j].back()->update(w->ourTeam[j][k]);
        }
    }
    for (int j = 0; j < _MAX_NUM_PLAYERS; j++) {
        for (int k = 0; k < oppTeam[j].count(); k++) {
            delete oppTeam[j][k];
        }
        oppTeam[j].clear();
        for (int k = 0; k < w->oppTeam[j].count(); k++) {
            oppTeam[j].append(new Robot(j, false, true));
            oppTeam[j].back()->update(w->oppTeam[j][k]);
        }
    }
}

//
//void CHalfWorld::selectBall(Vector2D pos)
//{
//    double minDist = 0.0;
//    int bestBall = -1;
//    for (int i=0;i<ball.count();i++)
//    {
//        double d = (ball[i]->pos-pos).length();
//        if (d<minDist || bestBall==-1)
//        {
//            bestBall = i;
//            minDist = d;
//        }
//    }
//    if (bestBall != -1 && ball.count()>0)
//    {
//        ball.swap(0, bestBall);
//    }
//}
//
