#include "pack_util/math/mathtools.h"



double AvgWithoutOutliers(QList<double> data , double accuracy) {
    //calculate a number for each member of list, saved in isOutlier list.
    //for non outlier members, most probable number to be produced is 1/2.
    //if it becaomes greater than 1 (or than "accuracy" :D) we can omit the related member.
    //output is the avrage of non outlier members.
    QList<double> isOutlier;
    double size = data.count() , n = 0;
    double avrg = 0 , a = 0 , b = 0 , res = 0;
    bool equal = true;

    if (size == 1) {
        return data.first();
    }

    for (int i = 0; i < size; i++) {
        avrg += data.at(i);
    }

    avrg /= size;

    for (int i = 0; i < size; i++) {
        a = (data.at(i) - avrg) * (data.at(i) - avrg);
        for (int j = 0; j < size; j++) {
            b += (data.at(j) - avrg) * (data.at(j) - avrg);
        }
        b -= a;
        isOutlier.append((double)(a * (size - 2) / (2 * b)) + (double)((size - 2) / (2 * (size - 1))));
        b = 0;
    }

    for (int i = 0; i < size; i++) {
        if (isOutlier.at(i) <= accuracy) {
            res += data.at(i);
            n++;
        }
    }

    for (int i = 0; i < size - 1; i++) { // all equal
        if (data.at(i) != data.at(i + 1)) {
            equal = false;
        }
    }

    if (equal && size > 0) {
        return data.last();
    }

    return res / n;
}


//fits a line to y array: y = bx + a
void linefit(unsigned int n, double *x, double *y, double &a, double &b);

double det2x2(double mat[2][2]) {
    double det = (mat[0][0] * mat[1][1]) - (mat[0][1] * mat[1][0]);
    return det;
}

double det3x3(double mat[3][3]) {
    double det = 0.0;
    det += mat[0][0] * ((mat[1][1] * mat[2][2]) - (mat[1][2] * mat[2][1]));
    det += mat[0][1] * ((mat[1][2] * mat[2][0]) - (mat[1][0] * mat[2][2]));
    det += mat[0][2] * ((mat[1][0] * mat[2][1]) - (mat[1][1] * mat[2][0]));
    return det;
}

void mult3x3(double m1[3][3], double m2[3][3], double m[][3]) {
    m[0][0] = m1[0][0] * m2[0][0] + m1[0][1] * m2[1][0] + m1[0][2] * m2[2][0];
    m[0][1] = m1[0][0] * m2[0][1] + m1[0][1] * m2[1][1] + m1[0][2] * m2[2][1];
    m[0][2] = m1[0][0] * m2[0][2] + m1[0][1] * m2[1][2] + m1[0][2] * m2[2][2];
    m[1][0] = m1[1][0] * m2[0][0] + m1[1][1] * m2[1][0] + m1[1][2] * m2[2][0];
    m[1][1] = m1[1][0] * m2[0][1] + m1[1][1] * m2[1][1] + m1[1][2] * m2[2][1];
    m[1][2] = m1[1][0] * m2[0][2] + m1[1][1] * m2[1][2] + m1[1][2] * m2[2][2];
    m[2][0] = m1[2][0] * m2[0][0] + m1[2][1] * m2[1][0] + m1[2][2] * m2[2][0];
    m[2][1] = m1[2][0] * m2[0][1] + m1[2][1] * m2[1][1] + m1[2][2] * m2[2][1];
    m[2][2] = m1[2][0] * m2[0][2] + m1[2][1] * m2[1][2] + m1[2][2] * m2[2][2];
}

void transpose3x3(double m[3][3], double t[][3]) {
    t[0][0] = m[0][0];
    t[0][1] = m[1][0];
    t[0][2] = m[2][0];
    t[1][0] = m[0][1];
    t[1][1] = m[1][1];
    t[1][2] = m[2][1];
    t[2][0] = m[0][2];
    t[2][1] = m[1][2];
    t[2][2] = m[2][2];
}
double det4x4(double mat[4][4]) {
    double det = 0.0;
    double matspec[3][3];
    for (int element = 0; element < 4; element++) {
        int row = 0;
        int col = 0;
        for (int i = 0; i < 4; i++) {
            if (i == 0) {
                continue;
            }
            for (int j = 0; j < 4; j++) {
                if (j == element) {
                    continue;
                }
                matspec[row][col] = mat[i][j];
                col++;
            }
            row++;
        }
        det += det3x3(matspec) * mat[0][element] * ((element % 2 == 0) ? 1.0 : -1.0);
    }
    return det;
}

void fillmat2x2(
    double mat[2][2],
    double m00, double m01,
    double m10, double m11) {
    mat[0][0] = m00;
    mat[0][1] = m01;
    mat[1][0] = m10;
    mat[1][1] = m11;
}

void fillmat3x3(
    double mat[3][3],
    double m00, double m01, double m02,
    double m10, double m11, double m12,
    double m20, double m21, double m22)

{
    mat[0][0] = m00;
    mat[0][1] = m01;
    mat[0][2] = m02;
    mat[1][0] = m10;
    mat[1][1] = m11;
    mat[1][2] = m12;
    mat[2][0] = m20;
    mat[2][1] = m21;
    mat[2][2] = m22;
}

void fillmat4x4(
    double mat[4][4],
    double m00, double m01, double m02, double m03,
    double m10, double m11, double m12, double m13,
    double m20, double m21, double m22, double m23,
    double m30, double m31, double m32, double m33)

{
    mat[0][0] = m00;
    mat[0][1] = m01;
    mat[0][2] = m02;
    mat[0][3] = m03;
    mat[1][0] = m10;
    mat[1][1] = m11;
    mat[1][2] = m12;
    mat[1][3] = m13;
    mat[2][0] = m20;
    mat[2][1] = m21;
    mat[2][2] = m22;
    mat[2][3] = m23;
    mat[3][0] = m30;
    mat[3][1] = m31;
    mat[3][2] = m32;
    mat[3][3] = m33;
}

void linefit(unsigned int n, double *x, double *y, double &a, double &b) {
    unsigned int i;
    double si = 0.0;
    double sxi = 0.0;
    double sxi2 = 0.0;
    double syi = 0.0;
    double sxiyi = 0.0;
    for (i = 0; i < n; i++) {
        sxi += x[i];
        sxi2 += x[i] * x[i];
        syi += y[i];
        sxiyi += x[i] * y[i];
    }
    si = n;
    double mat[2][2];
    double matA[2][2];
    double matB[2][2];
    fillmat2x2(mat,  si , sxi, sxi  , sxi2);
    fillmat2x2(matA, syi, sxi, sxiyi, sxi2);
    fillmat2x2(matB, si , syi, sxi,   sxiyi);

    double delta = det2x2(mat);
    if (delta == 0.0) {
        a = 0.0;
        b = 1e10;
    } else {
        a = det2x2(matA) / delta;
        b = det2x2(matB) / delta;
    }
}

void linefit(QVector<Vector2D> p, double &a, double &b) {
    unsigned int i;
    double si = 0.0;
    double sxi = 0.0;
    double sxi2 = 0.0;
    double syi = 0.0;
    double sxiyi = 0.0;
    for (i = 0; i < p.count(); i++) {
        sxi += p[i].x;
        sxi2 += p[i].x * p[i].x;
        syi += p[i].y;
        sxiyi += p[i].x * p[i].y;
    }
    si = p.count();
    double mat[2][2];
    double matA[2][2];
    double matB[2][2];
    fillmat2x2(mat,  si , sxi, sxi  , sxi2);
    fillmat2x2(matA, syi, sxi, sxiyi, sxi2);
    fillmat2x2(matB, si , syi, sxi,   sxiyi);

    double delta = det2x2(mat);
    if (delta == 0.0) {
        a = 0.0;
        b = 1e10;
    } else {
        a = det2x2(matA) / delta;
        b = det2x2(matB) / delta;
    }
}


void squarefit(unsigned int n, double *x, double *y, double &a, double &b, double &c) {
    unsigned int i;
    double si = 0.0;
    double sxi = 0.0;
    double sxi2 = 0.0;
    double sxi3 = 0.0;
    double sxi4 = 0.0;
    double syi = 0.0;
    double sxiyi = 0.0;
    double sxi2yi = 0.0;
    double temp;
    for (i = 0; i < n; i++) {
        sxi += x[i];
        temp = x[i] * x[i];
        sxi2 += temp;
        sxi3 += temp * x[i];
        sxi4 += temp * temp;

        syi += y[i];
        //temp = x[i] * y[i];
        sxiyi += x[i] * y[i];
        sxi2yi += x[i] * x[i] * y[i];
    }
    si = n;
    double mat[3][3];
    double matA[3][3];
    double matB[3][3];
    double matC[3][3];
    fillmat3x3(mat,  si , sxi, sxi2,   sxi  , sxi2 , sxi3 ,    sxi2  , sxi3  , sxi4);
    fillmat3x3(matA, syi, sxi, sxi2,   sxiyi, sxi2 , sxi3 ,    sxi2yi, sxi3  , sxi4);
    fillmat3x3(matB, si , syi, sxi2,   sxi  , sxiyi, sxi3 ,    sxi2  , sxi2yi, sxi4);
    fillmat3x3(matC, si , sxi, syi ,   sxi  , sxi2 , sxiyi,    sxi2  , sxi3  , sxi2yi);

    double delta = det3x3(mat);
    if (delta == 0.0) {
        a = 0.0;
        b = 0.0;
        c = 0.0;
    } else {
        a = det3x3(matA) / delta;
        b = det3x3(matB) / delta;
        c = det3x3(matC) / delta;
    }
}

void trifit(unsigned int n, double *x, double *y, double &a, double &b, double &c) {
    n;
    x;
    y;
    a;
    b;
    c;
    //TODO : what should happen to these ?!
    /*int i;
    double si = 0.0;
    double sxi = 0.0;
    double sxi2 = 0.0;
    double sxi3 = 0.0;
    double sxi4 = 0.0;
    double sxi5 = 0.0;
    double sxi6 = 0.0;
    double syi = 0.0;
    double sxiyi = 0.0;
    double sxi2yi = 0.0;
    double sxi3yi = 0.0;
    double temp;
    for (i = 0; i < n; i++)
    {
        sxi += x[i];
        temp = x[i] * x[i];
        sxi2 += temp;
        sxi3 += temp * x[i];
        sxi4 += temp * temp;
        sxi5 += temp * temp * x[i];
        sxi5 += temp * temp * temp;
        syi += y[i];
        sxiyi += x[i] * y[i];
        sxi2yi += x[i] * x[i] * y[i];
        sxi3yi += x[i] * x[i] * x[i] * y[i];
    }
    si = n;
    double mat[4][4];
    double matA[4][4];
    double matB[4][4];
    double matC[4][4];
    fillmat4x4(mat,  si , sxi, sxi2, sxi3,  sxi  , sxi2 , sxi3 , sxi4,   sxi2  , sxi3  , sxi4 , sxi5, sxi3, sxi4, sxi5, sxi6 );
    fillmat4x4(mat,  syi, sxi, sxi2, sxi3,  sxi  , sxi2 , sxi3 , sxi4,   sxi2  , sxi3  , sxi4 , sxi5, sxi3, sxi4, sxi5, sxi6 );
    fillmat4x4(mat,  si , sxi, sxi2, sxi3,  sxi  , sxi2 , sxi3 , sxi4,   sxi2  , sxi3  , sxi4 , sxi5, sxi3, sxi4, sxi5, sxi6 );
    fillmat4x4(mat,  si , sxi, sxi2, sxi3,  sxi  , sxi2 , sxi3 , sxi4,   sxi2  , sxi3  , sxi4 , sxi5, sxi3, sxi4, sxi5, sxi6 );
    double delta = det3x3(mat);
    if (delta == 0.0)
    {
        a = 0.0;
        b = 0.0;
        c = 0.0;
    }
    else
    {
        a = det3x3(matA) / delta;
        b = det3x3(matB) / delta;
        c = det3x3(matC) / delta;
    }
    */
}


CPolynomial::CPolynomial() {
    initialized = false;
    coefs.clear();
}

CPolynomial::~CPolynomial() {

}

void CPolynomial::setCoefs(QList<double> _coefs) {
    initialized = true;
    coefs.clear();
    coefs.append(_coefs);
}

QList<double> CPolynomial::getCoefs() {
    return coefs;
}

double CPolynomial::val(double x) {
    if (!initialized) {
        return 0;
    }
    double sum = 0;
    for (int i = 0; i < coefs.count(); i++) {
        sum += pow(x, i) * coefs[i];
    }
    return sum;
}



CPolynomialFit::CPolynomialFit(): CPolynomial() {
    initialized = false;
}

CPolynomialFit::~CPolynomialFit() {
}

void CPolynomialFit::fitToDataSet(QList< QPair<double, double> > newDataSet) {
    initialized = true;
    DataSet.clear();
    DataSet.append(newDataSet);
    double term;
    coefs.clear();
    for (int i = 0; i < DataSet.count(); i++) {
        term = 1;
        for (int j = 0; j < DataSet.count(); j++) {
            if (i == j) {
                continue;
            }
            term /= (DataSet[i].first - DataSet[j].first);
        }
        coefs.insert(i, term);
    }
}

double CPolynomialFit::val(double x) {
    if (!initialized) {
        return 0;
    }
    double sum = 0, term;
    for (int i = 0; i < DataSet.count(); i++) {
        term = DataSet[i].second * coefs[i];
        for (int j = 0; j < DataSet.count(); j++) {
            if (i == j) {
                continue;
            }
            term *= (x - DataSet[j].first);
        }
        sum += term;
    }
    return sum;
}


CPolynomialRegression::CPolynomialRegression(): CPolynomial() {
    initialized = false;
}

CPolynomialRegression::~CPolynomialRegression() {
}

void CPolynomialRegression::fitToDataSet(QList<QPair<double, double> > newDataSet, int _n) {
    DataSet.clear();
    DataSet.append(newDataSet);
    n = _n;
    A.resize(n + 1, 1);
    B.resize(n + 1, n + 1);
    B.e(0, 0) = DataSet.count();
    for (int i = 0; i <= 2 * n; i++) {
        double sum = 0;
        for (int j = 0; j < DataSet.count(); j++) {
            sum += pow(DataSet[j].first, i);
        }

        for (int k = 0; k <= n; k++) {
            if (k < n + 1 && i - k < n + 1) {
                B.e(k, i - k) = sum;
                //                qDebug()<<i<<k<<i-k;
            }
        }
    }

    for (int i = 0; i <= n; i++) {
        double sum = 0;
        for (int j = 0; j < DataSet.count(); j++) {
            sum += pow(DataSet[j].first, i) * DataSet[j].second;
        }

        A.e(i, 0) = sum;
    }

    //    fprintf(stderr,"\r\nA:\r\n");
    //    A.print();
    //    fprintf(stderr,"\r\nB:\r\n");
    //    B.print();
    //    if(A.determinant()==0)
    //    {
    //        qWarning()<<"0 determinant";
    //        return;
    //    }
    initialized = true;
    B.inverse();
    C = B * A;

    //    fprintf(stderr,"\r\nC:\r\n");
    //    C.print();


    coefs.clear();

    for (int i = 0; i < n + 1; i++) {
        coefs.append(C.e(i, 0));
    }
}

QVector<double> CPolynomialRegression::PolynomialRegression(QList<double> value, QList<int> key, int n/*order*/) {
    double X[2 * n + 1];
    double B[n + 1][n + 2];
    double Y[n + 1];
    int j , k, N = value.count();
    coefs.clear();
    QVector<double> coefs(QVector<double>(n + 1));
    if (value.count() > key.count()) {
        N = key.count();
    }
    if (value.count() > 0) {
        for (int i1 = 0; i1 < N; i1++) {
            double temp = value.at(i1) , valj = value.at(i1);
            X[0] = N;
            for (int i = 1; i < 2 * n + 1; i++) {
                X[i] += temp;
                temp *= valj;
            }
        }
        for (j = 0; j < N; j++) {
            Y[0] += key.at(j);
            double temp = value.at(j) , valj = value.at(j);
            for (int i = 1; i < n + 1; i++) {
                Y[i] += key.at(j) * temp;
                temp *= valj;
            }
        }
        for (int i = 0; i <= n; i++)
            for (j = 0; j <= n; j++) {
                B[i][j] = X[i + j];
            }
        for (int i = 0; i <= n; i++) {
            B[i][n + 1] = Y[i];
        }
        n = n + 1;
        for (int i = 0; i < n; i++)
            for (k = i + 1; k < n; k++)
                if (B[i][i] < B[k][i])
                    for (j = 0; j <= n; j++) {
                        double temp = B[i][j];
                        B[i][j] = B[k][j];
                        B[k][j] = temp;
                    }
        for (int i = 0; i < n - 1; i++)
            for (k = i + 1; k < n; k++) {
                double t = B[k][i] / B[i][i];
                for (j = 0; j <= n; j++) {
                    B[k][j] = B[k][j] - t * B[i][j];
                }
            }
        for (int i = n - 1; i >= 0; i--) {
            coefs[i] = B[i][n];
            for (j = 0; j < n; j++)
                if (j != i) {
                    coefs[i] = coefs[i] - B[i][j] * coefs[j];
                }
            coefs[i] = coefs[i] / B[i][i];
        }
    }
    return coefs;
}

CHalfLogRegression::CHalfLogRegression() : pr() {
    A = B = 0;
    initialized = false;
}

CHalfLogRegression::~CHalfLogRegression()
    = default;

void CHalfLogRegression::fitToDataSet(QList<QPair<double, double> > newDataSet) {
    QList<QPair<double, double> > ds;
    ds.clear();
    for (int i = 0; i < newDataSet.count(); i++) {
        ds.append(qMakePair(newDataSet[i].first, log(newDataSet[i].second)));
    }
    pr = new CPolynomialRegression();
    pr->fitToDataSet(ds);
    QList<double> c = pr->getCoefs();
    A = exp(c[0]);
    B = c[1];
    delete pr;

}

double CHalfLogRegression::val(double x) {
    return A * exp(B * x);
}

double CHalfLogRegression::invval(double y) {
    return (1.0 / B) * log(y / A);
}


QList<QList<int> > comb(QList<int> l) {
    QList<QList<int> > r;
    if (l.count() == 1) {
        r.append(l);
        return r;
    }
    for (int i = 0; i < l.count(); i++) {
        bool flag = false;
        if (i >= 1) {
            if (l[i] != l[i - 1]) {
                flag = true;
            }
        } else {
            flag = true;
        }
        if (flag) {
            QList<int> q = l;
            q.removeAt(i);
            QList<QList<int> > t = comb(q);
            for (int j = 0; j < t.count(); j++) {
                t[j].prepend(l[i]);
            }
            r.append(t);
        }
    }
    return r;
}

QList<QList<int> > generateCombinations(QList<int> l) {
    qSort(l.begin(), l.end());
    return comb(l);
}

void subs(QList<QList<int> >& res, QList<int> r, QList<int> l, int m, int k) {
    if (r.length() == m) {
        res.append(r);
    } else {
        for (int j = k; j < l.length(); j++) {
            r.append(l[j]);
            subs(res, r, l, m, j + 1);
            r.pop_back();
        }
    }
}

QList<QList<int> > generateSubsets(QList<int> l, int m) {
    QList<QList<int> > res;
    QList<int> r;
    subs(res, r, l, m, 0);
    return res;
}

/****************************************************************************
   Least squares fit of circle to set of points.
   by Dave Eberly (eberly@cs.unc.edu or eberly@ndl.com)
   ftp://ftp.cs.unc.edu/pub/users/eberly/magic/circfit.c
  ---------------------------------------------------------------------------
   Input:  (x_i,y_i), 1 <= i <= N, where N >= 3 and not all points
           are collinear
   Output:  circle center (a,b) and radius r

   Energy function to be minimized is

      E(a,b,r) = sum_{i=1}^N (L_i-r)^2

   where L_i = |(x_i-a,y_i-b)|, the length of the specified vector.
   Taking partial derivatives and setting equal to zero yield the
   three nonlinear equations

   E_r = 0:  r = Average(L_i)
   E_a = 0:  a = Average(x_i) + r * Average(dL_i/da)
   E_b = 0:  b = Average(y_i) + r * Average(dL_i/db)

   Replacing r in the last two equations yields

     a = Average(x_i) + Average(L_i) * Average(dL_i/da) = F(a,b)
     b = Average(y_i) + Average(L_i) * Average(dL_i/db) = G(a,b)

   which can possibly be solved by fixed point iteration as

     a_{n+1} = F(a_n,b_n),  b_{n+a} = G(a_n,b_n)

   with initial guess a_0 = Average(x_i) and b_0 = Average(y_i).
   Derivative calculations show that

     dL_i/da = (a-x_i)/L_i,  dL_i/db = (b-y_i)/L_i.

  ---------------------------------------------------------------------------
   WARNING.  I have not analyzed the convergence properties of the fixed
   point iteration scheme.  In a few experiments it seems to converge
   just fine, but I do not guarantee convergence in all cases.
 ****************************************************************************/

Circle2D circleFit(QVector<Vector2D> P) {
    /* user-selected parameters */
    const int maxIterations = 64;
    const double tolerance = 1e-06;

    double a, b, r;
    int N = P.count();
    /* compute the average of the data points */
    int i, j;
    double xAvr = 0.0;
    double yAvr = 0.0;

    for (i = 0; i < N; i++) {
        xAvr += P[i].x;
        yAvr += P[i].y;
    }
    xAvr /= N;
    yAvr /= N;

    /* initial guess */
    a = xAvr;
    b = yAvr;

    for (j = 0; j < maxIterations; j++) {
        /* update the iterates */
        double a0 = a;
        double b0 = b;

        /* compute average L, dL/da, dL/db */
        double LAvr = 0.0;
        double LaAvr = 0.0;
        double LbAvr = 0.0;

        for (i = 0; i < N; i++) {
            double dx = P[i].x - a;
            double dy = P[i].y - b;
            double L = sqrt(dx * dx + dy * dy);
            if (fabs(L) > tolerance) {
                LAvr += L;
                LaAvr -= dx / L;
                LbAvr -= dy / L;
            }
        }
        LAvr /= N;
        LaAvr /= N;
        LbAvr /= N;

        a = xAvr + LAvr * LaAvr;
        b = yAvr + LAvr * LbAvr;
        r = LAvr;

        if (fabs(a - a0) <= tolerance && fabs(b - b0) <= tolerance) {
            break;
        }
    }

    return Circle2D(Vector2D(a, b), r);
    //  return (j < maxIterations ? j : -1);
}

MWBM::MWBM() {
    W = NULL;
    n = cap = 0;
}

MWBM::MWBM(int _m, int _n) {
    W = NULL;
    create(_m, _n);
}

MWBM::~MWBM() {
    destroy();
}

void MWBM::create(int _m, int _n) {
    destroy();
    n = _m;
    if (_n > _m) {
        n = _n;
    }
    W = new double* [n];
    for (int i = 0; i < n; i++) {
        W[i] = new double [n];
    }
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            W[i][j] = 0;
        }
    U = new double [n];
    V = new double [n];
    Y = new double [n]; /* <-- weight variables */
    N = new double [n];
    M = new int [n];
    P = new double [n];
    Q = new int [n];
    R = new double [n];
    S = new double [n];
    T = new double [n];
    cap = n;
}

void MWBM::destroy() {
    if (W == NULL) {
        return;
    }
    for (int i = 0; i < cap; i++) {
        delete [] W[i];
    }
    delete [] W;
    delete [] U;
    delete [] V;
    delete [] Y;
    delete [] M;
    delete [] N;
    delete [] P;
    delete [] Q;
    delete [] R;
    delete [] S;
    delete [] T;
    W = NULL;
    n = 0;
    cap = 0;
}

void MWBM::changeSize(int k, int r) {
    int pp = max(k, r);
    if (pp > cap) {
        return;
    }
    n = pp;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            W[i][j] = 0;
        }
}

void MWBM::setWeight(int i, int j, double w) {
    if (i >= n || j >= n || i < 0 || j < 0) {
        return;
    }
    W[i][j] = w;
}

double MWBM::getWeight(int i, int j) {
    if (i >= n || j >= n || i < 0 || j < 0) {
        return 0;
    }
    return W[i][j];
}

int MWBM::getMatch(int i) {
    return M[i];
}

void MWBM::setWeightsByDists(QList <Vector2D> upNodes, QList <Vector2D> downNodes) {
    if (upNodes.size() > n || downNodes.size() > n) {
        return;
    }
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            setWeight(i, j, upNodes[i].dist(downNodes[j]));
        }
}

/* Graph Theory: Maximum Weighted Bipartite Matching
   Combinatorics: Assignment Problem
   =================================================================
   Description: Given N workers and N jobs to complete, where each worker has a
                certain compatibility (weight) to each job, find an assignment
                (perfect matching) of workers to jobs which maximizes the
                compatibility (weight).

   Complexity:  O(n^3), where n is the number of workers or jobs.
   -----------------------------------------------------------------
   Author:      Jason Klaus
   Date:        February 18, 2004
   References:  www.cs.umd.edu/class/fall2003/cmsc651/lec07.ps
   -----------------------------------------------------------------
   Reliability: 3
   Notes:       - W is a 2 dimensional array where W[i][j] is the weight of
                  worker i doing job j.  Weights must be non-negative.  If
                  there is no weight assigned to a particular worker and job
                  pair, set it to zero.  If there is a different number of
                  workers than jobs, create dummy workers or jobs accordingly
                  with zero weight edges.
                - M is a 1 dimensional array populated by the algorithm where
                  M[i] is the index of the job matched to worker i.
                - This algorithm could be used on non-negative floating point
                  weights as well.
*/

/* Returns the maximum weight, with the perfect matching stored in M. */
double MWBM::findMatching() {
    double w, y; /* <-- weight variables */
    int i, j, m, p, q, s, t, v;

    for (i = 0; i < n; i++) {
        M[i] = static_cast<int>(N[i] = -1);
        U[i] = V[i] = 0;

        for (j = 0; j < n; j++)
            if (W[i][j] > U[i]) {
                U[i] = W[i][j];
            }
    }

    for (m = 0; m < n; m++) {
        for (p = i = 0; i < n; i++) {
            T[i] = 0;
            Y[i] = -1;

            if (M[i] == -1) {
                S[i] = 1;
                P[p++] = i;
            } else {
                S[i] = 0;
            }
        }

        while (true) {
            for (q = s = 0; s < p; s++) {
                i = static_cast<int>(P[s]);

                for (j = 0; j < n; j++)
                    if (!T[j]) {
                        y = U[i] + V[j] - W[i][j];

                        if (y == 0) {
                            R[j] = i;
                            if (N[j] == -1) {
                                goto end_phase;
                            }
                            T[j] = 1;
                            Q[q++] = j;
                        } else if ((Y[j] == -1) || (y < Y[j])) {
                            Y[j] = y;
                            R[j] = i;
                        }
                    }
            }

            if (q == 0) {
                y = -1;

                for (j = 0; j < n; j++)
                    if ((T[j] == 0.0) && ((y == -1) || (Y[j] < y))) {
                        y = Y[j];
                    }

                for (j = 0; j < n; j++) {
                    if (T[j] != 0.0) {
                        V[j] += y;
                    }

                    if (S[j] != 0.0) {
                        U[j] -= y;
                    }
                }

                for (j = 0; j < n; j++)
                    if (T[j] == 0.0) {
                        Y[j] -= y;

                        if (Y[j] == 0) {
                            if (N[j] == -1) {
                                goto end_phase;
                            }
                            T[j] = 1;
                            Q[q++] = j;
                        }
                    }
            }

            for (p = t = 0; t < q; t++) {
                i = static_cast<int>(N[Q[t]]);
                S[i] = 1;
                P[p++] = i;
            }
        }

end_phase:
        i = static_cast<int>(R[j]);
        v = M[i];
        M[i] = j;
        N[j] = i;

        while (v != -1) {
            j = v;
            i = static_cast<int>(R[j]);
            v = M[i];
            M[i] = j;
            N[j] = i;
        }
    }

    for (i = w = 0.0; i < n; i++) {
        w += W[i][M[i]];
    }

    return w;
}

double MWBM::findMaxMinMatching() {
    double minWeight = 0;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            minWeight = min(minWeight, W[i][j]);
        }
    double tempW[n][n];
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            tempW[i][j] = W[i][j];
        }
//    qDebug() << "sdlfkjsaldfjiefj" << " " << minWeight << endl;
    double l = minWeight - 1, r = 0;
    for (int _t = 25; _t >= 0; _t--) {
        //qDebug() << l << " " << r << endl;
        double m = (r + l) / 2;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                if (W[i][j] < m) {
                    W[i][j] = -12345678;
                }
        double d = findMatching();
        if (d <= -12345678) {
            r = m;
        } else {
            l = m;
        }
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++) {
                W[i][j] = tempW[i][j];
            }
    }
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            if (W[i][j] < l) {
                W[i][j] = -12345678;
            }
    double w = findMatching();
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            W[i][j] = tempW[i][j];
        }
    return w;
}