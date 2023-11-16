#ifndef _TRACKING_HPP_
#define _TRACKING_HPP_

#include <iostream>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

static Eigen::IOFormat EigenCommaFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "; ", "", "", "[", "]");
static Eigen::IOFormat EigenCsvFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", ",", "", "", "", "");

static inline std::ostream & operator<<(std::ostream &out, const Eigen::Quaternionf & q) {
    //return out << "[" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]";
    return out << q.coeffs().format(EigenCommaFormat);
}

class Tracking {
public:
    Eigen::Vector3f p_acc = Eigen::Vector3f::Zero();
    Eigen::Vector3f p_vec = Eigen::Vector3f::Zero();
    Eigen::Vector3f p = Eigen::Vector3f::Zero();

    Tracking() {}
    void on_entry_acc(Eigen::Vector3f acc_calc, float t);

};


#endif

