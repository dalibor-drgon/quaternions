#ifndef _TRACKING_HPP_
#define _TRACKING_HPP_

#include "quaternion.hpp"
#include "vector3.hpp"

class Tracking {

public:
    Vector3 p_acc;
    Vector3 p_vec;
    Vector3 p;
    Vector3 g;

    Quaternion rot_acc;
    Quaternion rot_vec;
    Quaternion rot;

public:

    Tracking(Vector3 g)
            : p(0.0f, 0.0f, 0.0f),
              p_vec(0.0f, 0.0f, 0.0f),
              p_acc(0.0f, 0.0f, 0.0f),
              g(g) {}

    void onEntry(Quaternion inc_rot_acc, Vector3 inc_acc, float t) {
        rot_acc = inc_rot_acc;// * rot_acc;
        rot_vec = rot_acc.slerp(t) * rot_vec;
        rot = rot_vec.slerp(t) * rot;

        p_acc = rot.rotate(inc_acc);
        p_vec = p_vec + (p_acc - g) * t;
        p = p + p_vec * t;
    }


};


#endif

