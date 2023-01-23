#ifndef _TRACKING_HPP_
#define _TRACKING_HPP_

#include "quaternion.hpp"
#include "vector3.hpp"

#include <iomanip>

class Tracking {

public:
    Vector3 p_acc;
    Vector3 p_vec;
    Vector3 p;
    Vector3 g;

    //Quaternion rot_acc;
    //Quaternion rot_vec;
    Quaternion rot;

public:

    Tracking() {}

    Tracking(Vector3 g)
            : p(0.0f, 0.0f, 0.0f),
              p_vec(0.0f, 0.0f, 0.0f),
              p_acc(0.0f, 0.0f, 0.0f),
              g(g) {}

    void on_entry(Quaternion rot_vec, Vector3 inc_acc, float t) {
        //rot_acc = inc_rot_acc;// * rot_acc;
        //rot_vec = rot_acc.slerp(t) * rot_vec;
        rot = rot_vec * rot; 
        //rot = rot_vec.slerp(t) * rot;


        Vector3 m1 = rot.rotate(Vector3{1.0, 0.0, 0.0});
        Vector3 m2 = rot.rotate(Vector3{0.0, 1.0, 0.0});
        Vector3 m3 = rot.rotate(Vector3{0.0, 0.0, 1.0});

        Vector3 cur_acc = inc_acc.matrixMult(m1, m2, m3);
        Vector3 dif = g - cur_acc;
        double dotp = dif.dot(dif);

        //std::cout << cur_acc << "\t" << g << std::endl;
        std::cout << std::fixed << std::setprecision(4) << cur_acc << " " << dotp << "\t";

        p_acc = rot.rotate(inc_acc);
        if (dotp < 0.1) {
            //g = cur_acc;
        }
        p_vec = p_vec + (p_acc - g) * t;
        p = p + p_vec * t;
    }


};


#endif

