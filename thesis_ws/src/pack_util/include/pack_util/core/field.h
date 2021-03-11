#ifndef PARSIAN_UTIL_FIELD_H
#define PARSIAN_UTIL_FIELD_H

#include <QString>
#include <QList>
#include <QStringList>

#include <pack_util/geom/geom.h>



class CField {
public:

    /* Field */
    double _FIELD_WIDTH;
    double _FIELD_HEIGHT;
    double _FIELD_MARGIN_HEIGHT;
    double _FIELD_MARGIN_WIDTH;
    double _FIELD_PENALTY_POINT;
    double _GOAL_WIDTH;
    double _GOAL_DEPTH;
    double _PENALTY_DEPTH;
    double _PENALTY_WIDTH;
    double _CENTER_CIRCLE_RAD;
    double _MAX_DIST;

    double _BOUNDARY_WIDTH;
    double _LINE_THICKNESS;


    enum class Region {
        OurCornerTop = 1 ,
        OurCornerBottom = 2 ,
        OppCornerTop = 3 ,
        OppCornerBottom = 4 ,
        MidFieldTop = 5 ,
        MidFieldBottom = 6 ,
        FrontGoalOpp = 7 ,
        FrontGoalOur = 8 ,
        Center = 9 ,
        OurPenalty = 10 ,
        OppPenalty = 11 ,
        OurField = 12 ,
        OppField = 13 ,
        OppCornerTopRight = 14 ,
        OppCornerTopLeft = 15 ,
        OppCornerBottomRight = 16 ,
        OppCornerBottomLeft = 17 ,
        OurMidFieldTop = 18 ,
        OurMidFieldBottom = 19 ,
        OppMidFieldTop = 20 ,
        OppMidFieldBottom = 21 ,
        OppFieldBottom = 22 ,
        OppFieldTop = 23 ,
        OurFieldTop = 24 ,
        OurFieldBottom = 25 ,
        OurMidFieldTopMost = 26 ,
        OurMidFieldBottomMost = 27 ,
        Field1stQuarter = 28 ,
        Field2ndQuarter = 29 ,
        Field3rdQuarter = 30 ,
        Field4thQuarter = 31 ,
        OurMidFieldTopWing = 32 ,
        OurMidFieldBottomWing = 33 ,
        // For mani
        //----------------------
        OurDeffenseOneThird = 34 ,
        OurAttackOneThird = 35 ,
        OurMidOneThird = 36 ,
        //----------------------
        OurCornerTopRight = 37 ,
        OurCornerTopLeft = 38 ,
        OurCornerBottomRight = 39 ,
        OurCornerBottomLeft = 40 ,
        OppMidField = 41 ,
        OppCornerLineTop = 42 ,
        OppCornerLineBottom = 43 ,
        FieldGrid1Top = 44 ,
        FieldGrid1Center = 45 ,
        FieldGrid1Bottom = 46 ,
        FieldGrid2Top = 47 ,
        FieldGrid2Center = 48 ,
        FieldGrid2Bottom = 49 ,
        FieldGrid3Top = 50 ,
        FieldGrid3Center = 51 ,
        FieldGrid3Bottom = 52 ,
        FieldGrid4Top = 53 ,
        FieldGrid4Center = 54 ,
        FieldGrid4Bottom = 55 ,
        FieldGrid5Top = 56 ,
        FieldGrid5Center = 57 ,
        FieldGrid5Bottom = 58 ,
        OppCornerPointTop = 59 ,
        OppCornerPointBottom = 60 ,
        TheirPenaltyTop = 61 ,
        TheirPenaltyBottom = 62 ,
        AttackRecCornerTopB = 63 ,
        AttackRecCornerTopP1 = 64 ,
        AttackRecCornerTopP2 = 65 ,
        AttackRecCornerBottomB = 66 ,
        AttackRecCornerBottomP1 = 67 ,
        AttackRecCornerBottomp2 = 68 ,
        AttackRecMidTopB = 69 ,
        AttackRecMidTopP1 = 70 ,
        AttackRecMidTopP2 = 71 ,
        AttackRecMidBottomB = 72 ,
        AttackRecMidBottomP1 = 73 ,
        AttackRecMidBottomp2 = 74 ,
        TC2012Top1 = 75 ,
        TC2012Top2 = 76 ,
        TC2012Top3 = 77 ,
        TC2012Top4 = 78 ,
        TC2012Bottom1 = 79 ,
        TC2012Bottom2 = 80 ,
        TC2012Bottom3 = 81 ,
        TC2012Bottom4 = 82 ,
        TC2012Rect1 = 83 ,
        TC2012Rect2 = 84 ,
        TC2012Rect3 = 85 ,
        TC2012Rect4 = 86 ,
        OurCornerPointTop = 87 ,
        OurCornerPointBottom = 88 ,
        LargeOppCornerTop = 89 ,
        LargeOppCornerBottom = 90 ,
        LargeOppCornerTopRight = 91 ,
        LargeOppCornerTopLeft = 92 ,
        LargeOppCornerBottomRight = 93 ,
        LargeOppCornerBottomLeft = 94 ,
        BigCenter = 95
    };

    CField();

    ~CField();

    Vector2D center() const;

    Vector2D ourGoal() const;

    Vector2D oppGoal() const;

    Vector2D ourCornerL() const;

    Vector2D ourCornerR() const;

    Vector2D oppCornerL() const;

    Vector2D oppCornerR() const;

    Vector2D ourPenalty() const;

    Vector2D oppPenalty() const;

    Vector2D ourGoalL() const;

    Vector2D ourGoalR() const;

    Segment2D ourGoalLine() const;

    Vector2D oppGoalL() const;

    Vector2D oppGoalR() const;

    Segment2D oppGoalLine() const;

    Vector2D ourOneThirdL() const;

    Vector2D ourOneThirdR() const;

    Vector2D oppOneThirdL() const;

    Vector2D oppOneThirdR() const;

    Rect2D fieldRect() const;

    Rect2D marginedField() const;

    Rect2D ourPenaltyRect() const;

    Rect2D oppPenaltyRect() const;

    bool isInField(Vector2D point) const;

    bool isInOurPenaltyArea(Vector2D point) const;

    bool isInOppPenaltyArea(Vector2D point) const;

    Rect2D getRegion(Region region , double k = 0.25) const;

    Rect2D getRegion(QString name , double k = 0.25) const;

    Rect2D getCircleRegion(int n , int i) const;

    QList<Vector2D> ourBigPAreaIntersect(Line2D line , double scale = - 1 , float bias = 0) const;

    QList<Vector2D> ourPAreaIntersect(Line2D line) const;

    QList<Vector2D> ourPAreaIntersect(Segment2D segment) const;

    /////////////////////////////////////////////////////////////////////
    QList<Vector2D> ourPAreaIntersect(Circle2D circle) const;

    QList<Vector2D> oppPAreaIntersect(Line2D line) const;

    QList<Vector2D> oppPAreaIntersect(Segment2D segment) const;

    QList<Vector2D> oppPAreaIntersect(Circle2D circle) const;

    Vector2D ourPAreaPerpendicularVector(double angle , Vector2D &intersectpoint) const;
    ///////////////////////////////////////// AHZ ////////////////////////////////
    Rect2D ourBigPenaltyArea(double scale = 1 , double tolerance = 0 , bool doChangesWithScale  = 1);
    Rect2D oppBigPenaltyArea(double scale = 1, double tolerance = 0 , bool doChangesWithScale = 1);
    ////////////////////////////////////////////////////////////////////////////////
    double ourPAreaPos(double angle) const;

private:
    Vector2D fCenter;
    Vector2D fOurGoal;
    Vector2D fOppGoal;
    Vector2D fOurCornerL;
    Vector2D fOurCornerR;
    Vector2D fOppCornerL;
    Vector2D fOppCornerR;
    Vector2D fOurPenalty;
    Vector2D fOppPenalty;
    Vector2D fOurGoalL;
    Vector2D fOurGoalR;
    Vector2D fOppGoalL;
    Vector2D fOppGoalR;
    Rect2D fFieldRect;
    Rect2D fMarginedFieldRect;
    Rect2D fOurPenaltyRect;
    Rect2D fOppPenaltyRect;
    Vector2D fOurOneThirdL;
    Vector2D fOppOneThirdL;
    Vector2D fOurOneThirdR;
    Vector2D fOppOneThirdR;
};

#endif //PARSIAN_UTIL_FIELD_H
