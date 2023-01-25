#ifndef _TRACKING_HPP_
#define _TRACKING_HPP_

#include <iostream>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

static Eigen::IOFormat EigenCommaFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "; ", "", "", "[", "]");

static inline std::ostream & operator<<(std::ostream &out, const Eigen::Quaternionf & q) {
    //return out << "[" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]";
    return out << q.coeffs().format(EigenCommaFormat);
}

class Tracking {


public:
    Eigen::Vector3f p_acc;
    Eigen::Vector3f p_vec;
    Eigen::Vector3f p;
    Eigen::Vector3f g {0, 0, -9.81};

    Eigen::Vector3f inc_acc_nfa;
    Eigen::Vector3f inc_acc_fa;

    //Quaternion rot_acc;
    //Quaternion rot_vec;
    Eigen::Quaternionf rot;

public:

    Tracking() {}

    Tracking(Eigen::Vector3f g0)
            : p(0.0f, 0.0f, 0.0f),
              p_vec(0.0f, 0.0f, 0.0f),
              p_acc(0.0f, 0.0f, 0.0f),
              inc_acc_nfa(g0),
              inc_acc_fa(normalize(g0)) {
        Eigen::Vector3f v = normalize(g0);
        //std::cout << v.format(EigenCommaFormat) << std::endl;
#if 0
        Eigen::Matrix3f desired;
        desired << -9.81, 0, 0, 0, -9.81, 0, 0, 0, -9.81;
        Eigen::Matrix3f current;
        current << v[2], v[0], v[1],  v[1], v[2], v[0],  v[0], v[1], v[2];
        Eigen::Matrix3f new_rot = desired * current.inverse();
#elif 0
        Eigen::Vector3f base = v * (1.0f / 9.81);
        Eigen::Matrix3f new_rot;
        new_rot << base[2], base[0], base[1], base[1], base[2], base[0], base[0], base[1], base[2];
#elif 1
        Eigen::Vector3f xyz = v.cross(g);
        float w = 9.81 * 9.81 + v.dot(g);
        Eigen::Quaternionf q {w, xyz[0], xyz[1], xyz[2]};
        q.normalize();
        Eigen::Matrix3f new_rot (q);
#else
        Eigen::Vector3f dif = g - v;
        Eigen::Matrix3f rot_dif = dif * pinv(v).transpose(); // / (9.81 * 9.81);
        Eigen::Matrix3f new_rot;
        new_rot.setIdentity();
        new_rot += rot_dif;
#endif
        /*
        std::cout << new_rot.format(EigenCommaFormat) << std::endl;
        std::cout << (new_rot * v).format(EigenCommaFormat) << std::endl;
        std::cout << (new_rot * Eigen::Vector3f{v[2],v[0],v[1]}).format(EigenCommaFormat) << std::endl;
        std::cout << (new_rot * Eigen::Vector3f{v[2],v[1],v[0]}).format(EigenCommaFormat) << std::endl;
        rot = Eigen::Quaternionf(new_rot);
        std::cout << rot.coeffs().format(EigenCommaFormat) << std::endl;
        */
        rot = Eigen::Quaternionf(new_rot);
    }


    static Eigen::Vector3f pinv(Eigen::Vector3f vec) {
        float len2 = vec.dot(vec);
        return vec * (1.0f / len2);
    }

    static Eigen::Vector3f normalize(const Eigen::Vector3f & acc);
    Eigen::Matrix3f compute_correction(Eigen::Matrix3f rot, Eigen::Vector3f inc_acc);
    void on_entry(Eigen::Quaternionf rot_vec, Eigen::Vector3f inc_acc, float t);
    void on_entry_acc(Eigen::Vector3f acc_calc, float t);


};


#endif

