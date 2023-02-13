
#include "tracking.hpp"
#include <iomanip>
#include <iostream>
#include "kalman.hpp"
#include <memory>

#define DT (1/250.f)

using namespace Eigen;
using namespace std;

std::shared_ptr<Kalman<MatrixXf>> kalman;

void init_kalman() {
    MatrixXf F = MatrixXf::Identity(6, 6);
    F(0, 3) = F(1, 4) = F(2, 5) = DT;
    MatrixXf G(6, 3);
    G(0, 0) = G(1, 1) = G(2, 2) = 0.5 * DT * DT;
    G(3, 0) = G(4, 1) = G(5, 2) = DT;
    VectorXf x0 = VectorXf::Zero(6);
    MatrixXf Q = MatrixXf::Zero(6, 6);
    float DT2 = DT * DT;
    float DT3 = DT2 * DT / 2;
    float DT4 = DT3 * DT / 2;
    Q(0,0) = Q(1,1) = Q(2,2) = DT4;
    Q(3,3) = Q(4,4) = Q(5,5) = DT2;
    Q(3,0) = Q(4,1) = Q(5,2) = DT3;
    Q(0,3) = Q(1,4) = Q(2,5) = DT3;
    Q *= 0.005;

   
    MatrixXf P = MatrixXf::Identity(6, 6) * 1000;
}

    

Vector3f Tracking::normalize(const Vector3f & acc) {
    float len = sqrt(acc.dot(acc));
    return acc * (9.81 / len);
}

Matrix3f Tracking::compute_correction(Matrix3f rot, Vector3f inc_acc) {
    Vector3f dif = g - rot * inc_acc;
    return dif * inc_acc.transpose() / (9.81 * 9.81);
}

Eigen::Quaternionf Tracking::calculate_rotation(Eigen::Vector3f from, Eigen::Vector3f to) {
    Eigen::Vector3f xyz = from.cross(to);
    float w = sqrt(from.dot(from) * to.dot(to)) + from.dot(to);
    Eigen::Quaternionf q {w, xyz[0], xyz[1], xyz[2]};
    q.normalize();
    return q;
}

#include "history.hpp"
#include "acc-feature-extraction.hpp"

AccFeatureExtraction ext0, ext1, ext2;

History<Vector3f,32> hist_acc;


void Tracking::on_entry_acc(Eigen::Vector3f acc_calc, float t) {
    Vector3f acc_dif = acc_calc - g;
    hist_acc.add(acc_dif);
    Vector3f acc_avg = hist_acc.average(Vector3f{0,0,0});

    p = Vector3f{
        ext0.extract(acc_avg[0], t),
        ext1.extract(acc_avg[1], t),
        ext2.extract(acc_avg[2], t)
    };
    
    p_vec = Vector3f {
        ext0.veloc,
        ext1.veloc,
        ext2.veloc
    };


    cerr << acc_dif.format(EigenCommaFormat) << "\t" << p_vec.format(EigenCommaFormat) << "\t" << p.format(EigenCommaFormat) << endl;
    cout << acc_dif.format(EigenCsvFormat) << "," << p_vec.format(EigenCsvFormat) << "," << p.format(EigenCsvFormat) << endl;
}



void Tracking::on_entry(Eigen::Quaternionf rot_vec, Eigen::Vector3f inc_acc, float t) {
    float q = 0.4;
    float qq = 0; //0.0001;


    cout << inc_acc.format(EigenCsvFormat) << ","; 

    //rot_acc = inc_rot_acc;// * rot_acc;
    //rot_vec = rot_acc.slerp(t) * rot_vec;
    float inc_acc_len = inc_acc.dot(inc_acc);
    if (pow(9.81-0.5, 2) < inc_acc_len && inc_acc_len < pow(9.81+0.5, 2)) {
        Quaternionf rot_vec2 = calculate_rotation(normalize(inc_acc), rot.inverse()._transformVector(g));
        rot = rot * (rot_vec.slerp(qq, rot_vec2)); 
    }
    //cerr << endl << rot_vec.coeffs().format(EigenCommaFormat) << " " << rot_vec2.coeffs().format(EigenCommaFormat) << endl;
    //rot = rot_vec.slerp(t) * rot;

    Vector3f cur_acc = rot._transformVector(inc_acc);

    inc_acc_nfa = rot_vec._transformVector(inc_acc_nfa) * (1-q) + normalize(inc_acc) * q;
    inc_acc_fa = rot_vec._transformVector(inc_acc_fa) * (1 - q) + inc_acc * q;

    //cout << cur_acc.format(EigenCommaFormat) << " " << dotp << "\t";

#if 0
    //if (1) {
        Quaternionf rot2 = calculate_rotation(cur_acc, g);
        //Quaternionf rot2 = Quaternionf(mat + mat_c);
        float rate = rot2.angularDistance(rot) / M_PI;
        if (rate > 0.1)
            qq = (1.1 - rate) * qq;
        rot = rot.slerp(qq, rot2);
    //}
#endif
    cerr << rot.norm() << "\t";
    rot.normalize();
    p_acc = rot._transformVector(inc_acc);
    //cerr << p_acc.format(EigenCommaFormat) << "\t" << rot.coeffs().format(EigenCommaFormat) << "\t" << rot2.coeffs().format(EigenCommaFormat) << endl;
    on_entry_acc(p_acc, t);
}

