#ifndef MATHTOOLS_H
#define MATHTOOLS_H

#include <QList>
#include <QPair>
#include <QVector>
#include <QtAlgorithms>
#include <QDebug>

#include <math.h>
#include <stdio.h>

#include "pack_util/math/matrix.h"
#include "pack_util/geom/geom.h"

double AvgWithoutOutliers(QList<double> data , double accuracy);

//mat is 2x2 matrix
double det2x2(double mat[2][2]);

//mat is 3x3 matrix
double det3x3(double mat[3][3]);

//mat is 4x4 matrix
double det4x4(double mat[4][4]);

//fits a y=c*x^2 + b*x + a polynomial to y array
void squarefit(unsigned int n, double *x, double *y, double &a, double &b, double &c);

//fits a line to y array: y = bx + a
void linefit(unsigned int n, double *x, double *y, double &a, double &b);


void fillmat2x2(
    double mat[2][2],
    double m00, double m01,
    double m10, double m11);


void fillmat3x3(
    double mat[3][3],
    double m00, double m01, double m02,
    double m10, double m11, double m12,
    double m20, double m21, double m22);


void fillmat4x4(
    double mat[4][4],
    double m00, double m01, double m02, double m03,
    double m10, double m11, double m12, double m13,
    double m20, double m21, double m22, double m23,
    double m30, double m31, double m32, double m33);


void mult3x3(double m1[3][3], double m2[3][3], double m[][3]);


void transpose3x3(double m[3][3], double t[][3]);


class CPolynomial {
public:
    CPolynomial();


    ~CPolynomial();


    void setCoefs(QList<double> _coefs);


    QList<double> getCoefs();


    double val(double x);


protected:
    bool initialized;

    QList< double > coefs;
};

class CPolynomialFit : public CPolynomial {
public:
    CPolynomialFit();


    ~CPolynomialFit();


    void fitToDataSet(QList< QPair<double, double> > newDataSet);


    double val(double x);


private:
    QList< QPair<double, double> > DataSet;
};

class CPolynomialRegression : public CPolynomial {
public:
    CPolynomialRegression();


    ~CPolynomialRegression();


    QVector<double> PolynomialRegression(QList<double> value, QList<int> key, int n/*order*/);


    void fitToDataSet(QList< QPair<double, double> > newDataSet, int _n = 1);
private:
    int n;


    Matrix A, B, C;


    QList< QPair<double, double> > DataSet;
};

class CHalfLogRegression {
public:
    CHalfLogRegression();


    ~CHalfLogRegression();


    void fitToDataSet(QList< QPair<double, double> > newDataSet);


    double val(double x);


    double invval(double y);


    double A, B;
private:
    CPolynomialRegression *pr;


    bool initialized;
};

class MWBM { //Maximum Weighted Bipartite Matching
private:
    int n, cap;
    double **W;
    double *U, *V, *Y; /* <-- weight variables */
    double *N, *P, *R, *S, *T;
    int *Q, *M;
public:
    MWBM();
    MWBM(int _m, int _n);
    void changeSize(int k, int r); //max(k,r) should be less than n (otherwise destroy() then create(...,...) )
    ~MWBM();
    void setWeight(int i, int j, double w);
    double getWeight(int i, int j);
    void create(int _m, int _n);
    void destroy();
    double findMatching();
    int getMatch(int i);
    //TODO : added by parsa but does not have impementation or needs to check implementations
    int findMinMinMatching();
    double findMaxMinMatching();
    bool hasPerfectMatching();
    void setWeightsByDists(QList <Vector2D> upNodes, QList <Vector2D> downNodes);
    //end of functions added by parsa
};

void linefit(QVector<Vector2D> p, double &a, double &b);
QList<QList<int> > generateCombinations(QList<int> l);
QList<QList<int> > generateSubsets(QList<int> l, int m);
Circle2D circleFit(QVector<Vector2D> P);

#endif // MATHTOOLS_H
