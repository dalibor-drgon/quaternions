
#ifndef _KALMAN_HPP_
#define _KALMAN_HPP_

#include <eigen3/Eigen/src/Core/util/Constants.h>
#include <stdint.h>
#include <eigen3/Eigen/Dense>

template<class T>
class Kalman {

public:

    Eigen::MatrixXf F;
    Eigen::MatrixXf G;
    Eigen::VectorXf x;
    Eigen::MatrixXf P;
    Eigen::MatrixXf H;
    Eigen::MatrixXf Q;
    Eigen::MatrixXf R;

    Eigen::MatrixXf K;

    Kalman(
            Eigen::MatrixXf F, 
            Eigen::MatrixXf G,
            Eigen::VectorXf x,
            Eigen::MatrixXf P,
            Eigen::MatrixXf H,
            Eigen::MatrixXf Q,
            Eigen::MatrixXf R)
    :   F(F), 
        G(G), 
        x(x),
        P(P),
        H(H),
        Q(Q),
        R(R)
    {
    }

    Eigen::VectorXf update(Eigen::VectorXf u, Eigen::VectorXf z) {
        x = F * x + G * u;
        P = F * P * F.transpose() + Q;
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        x = x + K * (z - H * x);
        Eigen::MatrixXf ikh = K * H;
        ikh = Eigen::MatrixXf::Identity(ikh.rows(), ikh.cols()) - ikh;
        P = ikh * P * ikh.transpose() + K * R * K.transpose();
    }

};

#endif

