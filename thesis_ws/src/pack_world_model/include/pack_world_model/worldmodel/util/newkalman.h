#ifndef NEWKALMAN_H
#define NEWKALMAN_H

#include <deque>
//using namespace std;

#include <pack_util/math/matrix.h>
#include <eigen3/Eigen/Dense>


class newKalman {
public:
    newKalman();
};

template <int DIM, int MDIM>
class KalmanFilter {
public:
    typedef Eigen::Matrix<double, DIM, DIM> Matrix;
    typedef Eigen::Matrix<double, MDIM, DIM> MatrixM;
    typedef Eigen::Matrix<double, MDIM, MDIM> MatrixMM;
    typedef Eigen::Matrix<double, DIM, 1> Vector;
    typedef Eigen::Matrix<double, MDIM, 1> VectorM;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit KalmanFilter(const Vector &x) :
        F(Matrix::Identity()),
        B(Matrix::Identity()),
        u(Vector::Zero()),
        Q(Matrix::Zero()),
        H(MatrixM::Zero()),
        R(MatrixMM::Zero()),
        z(VectorM::Zero()),
        m_xm(x),
        m_Pm(Matrix::Identity()),
        m_x(x),
        m_P(Matrix::Identity()) {
    }

public:
    void predict(bool permanentUpdate) {
        m_xm = F * m_x + u;
        m_Pm = B * m_P * B.transpose() + Q;
        if (permanentUpdate) {
            m_x = m_xm;
            m_P = m_Pm;
        }
    }

    void update() {
        VectorM y = z - H * m_xm;
        MatrixMM S = H * m_Pm * H.transpose() + R;
        Eigen::Matrix<double, DIM, MDIM> K = m_Pm * H.transpose() * S.inverse();
        m_x = m_xm + K * y;
        m_P = (Matrix::Identity() - K * H) * m_Pm;
    }

    const Vector& state() const {
        return m_xm;
    }

    const Vector& baseState() const {
        return m_x;
    }

    // !!! Use with care
    void modifyState(int index, double value) {
        m_xm(index) = value;
    }

public:
    //! state transition model
    Matrix F;
    //! state transition jacobian
    Matrix B;
    //! control input
    Vector u;
    //! covariance of the process noise
    Matrix Q;
    //! observation model
    MatrixM H;
    //! covariance of the observation noise
    MatrixMM R;

    //! observation
    VectorM z;

private:
    //! predicted state
    Vector m_xm;
    //! predicted error covariance matrix
    Matrix m_Pm;
    //! updated state
    Vector m_x;
    //! updated error covariance matrix
    Matrix m_P;
};

#endif // NEWKALMAN_H
