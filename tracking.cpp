
#include "tracking.hpp"
#include <iomanip>
#include <iostream>

using namespace Eigen;
using namespace std;


Vector3f Tracking::normalize(const Vector3f & acc) {
    float len = sqrt(acc.dot(acc));
    return acc * (9.81 / len);
}

Matrix3f Tracking::compute_correction(Matrix3f rot, Vector3f inc_acc) {
    Vector3f dif = g - rot * inc_acc;
    return dif * inc_acc.transpose() / (9.81 * 9.81);
}


void Tracking::on_entry_acc(Eigen::Vector3f acc_calc, float t) {
    Vector3f dif = acc_calc - g;
    p_vec = p_vec + dif * t;
    p = p + p_vec * t;
    cout << dif.format(EigenCommaFormat) << "\t" << p_vec.format(EigenCommaFormat) << endl;
}

void Tracking::on_entry(Eigen::Quaternionf rot_vec, Eigen::Vector3f inc_acc, float t) {
    float q = 0.4;
    float qq = 0.4;

    //rot_acc = inc_rot_acc;// * rot_acc;
    //rot_vec = rot_acc.slerp(t) * rot_vec;
    rot = rot_vec * rot; 
    //rot = rot_vec.slerp(t) * rot;

    Matrix3f mat (rot);

    Vector3f cur_acc = mat * inc_acc;
    Vector3f dif = g - cur_acc;
    float dotp = dif.dot(dif);

    inc_acc_nfa = rot_vec._transformVector(inc_acc_nfa) * (1-q) + normalize(inc_acc) * q;
    inc_acc_fa = rot_vec._transformVector(inc_acc_fa) * (1 - q) + inc_acc * q;
    Matrix3f mat_c = compute_correction(mat, inc_acc_nfa);

    //cout << cur_acc.format(EigenCommaFormat) << " " << dotp << "\t";

    if (dotp < 0.5 || 1) {
        Quaternionf rot2 = Quaternionf(mat + mat_c);
        rot = rot2.slerp(qq, rot);
    }
    p_acc = (mat + mat_c) * inc_acc;
    on_entry_acc(p_acc, t);
}

