#ifndef MATRIX_H
#define MATRIX_H

/* LICENSE: */

/**********************************************************************
 *
 * Kwun Han <kwunh@cs.cmu.edu>
 * March 1997
 *
 * Michael Bowling <mhb@cs.cmu.edu>
 * 1998-2002
 *
 * Determinant and inverse code is copied from mtrxmath under the GPL.
 *
 **********************************************************************/

class Matrix {


    // errr... rows and columns

    int r_;
    int c_;

    double* mat;

    void str_init(char* const init_string);

    Matrix *reduce_matrix(int cut_row, int cut_col) const;

public:
    Matrix(char* const init_string);
    Matrix(int rows, int columns);
    Matrix(int rows, int columns, const float *m);

    double determinant();
    double determinant(Matrix *mx, int n);


    // this makes an identity matrix
    Matrix(int identity_size);

    // standard stuffs.
    Matrix();
    Matrix(const Matrix& other);

    ~Matrix();

    void CopyData(float *data);
    void CopyData(double *data);

    const Matrix& operator= (const Matrix& other);

    const Matrix& operator= (char* const init_string);

    friend const Matrix operator+ (const Matrix& a, const Matrix& b);
    friend const Matrix operator- (const Matrix& a, const Matrix& b);
    friend const Matrix operator* (const Matrix& a, const Matrix& b);
    friend const Matrix operator* (const double a , const Matrix& m);
    friend Matrix kron(const Matrix &a , const Matrix &b);
    friend const Matrix inverse(const Matrix& a);
    friend const Matrix transpose(const Matrix& a);
    friend const Matrix pseudoinverse(const Matrix& a);

    friend const Matrix& m_multiply(Matrix& out, const Matrix& a,
                                    const Matrix& b);
    friend const Matrix& m_inverse(Matrix& out, const Matrix& in);
    friend const Matrix& m_add(Matrix& out, const Matrix& a,
                               const Matrix& b);
    friend const Matrix& m_subtract(Matrix& out, const Matrix& a,
                                    const Matrix& b);
    friend const Matrix& m_transpose(Matrix& out, const Matrix& in);
    friend const Matrix& m_pseudoinverse(Matrix& out, const Matrix& in);

    const Matrix& transpose();
    const Matrix& identity(int size);
    const Matrix& inverse();
    const Matrix& resize(int row, int col);
    const Matrix& resizeS(int row, int col);
    const Matrix& pseudoinverse();

    const Matrix& scale(double factor);

    inline double& e(int row, int col) const {
        return mat[row * c_ + col];
    }

    int nrows() const {
        return r_;
    }
    int ncols() const {
        return c_;
    }

    void print() const;

    bool equals(double val);
    double  dot(Matrix &m);
    Matrix& dotP(Matrix &m);
};

Matrix kron(const Matrix &a , const Matrix &b);

#endif // MATRIX_H
