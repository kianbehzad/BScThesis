//
// Created by parsian-ai on 10/5/17.
//

#include <pack_world_model/worldmodel/wm/rawobject.h>


CRawObject::CRawObject(int frameCnt, Vector2D _pos, double orientation, int _ID, double _confidence, MovingObject *_ref, int _cam_id, double t)
    : pos() , dir() , updated() , mergeCount() {
    frameCount = frameCnt;
    pos = _pos;
    ID = _ID;
    dir = Vector2D::unitVector(orientation);
    confidence = _confidence;
    ref = _ref;
    cam_id = _cam_id;
    time = t;
    merged = false;
}

CRawObject::CRawObject() : pos() , dir() , cam_id() , ref() , updated() {
    frameCount = 0;
    pos.invalidate();
    ID = -1;
    confidence = 0;
    time = 0.0;
    ref = nullptr;
    merged = false;
    mergeCount = 0;
}