
#include "quaternion.hpp"
#include "tracking.hpp"
#include <iostream>

int main() {
    Quaternion q1 (1.0f, 1.0f, 0.0f, 0.0f);
    Quaternion q2 (1.0f, 1.1f, 0.0f, 0.0f);

    std::cout << q1 << std::endl;
    std::cout << q2 << std::endl;
    std::cout << q1.exp(0.0f) << std::endl;
    std::cout << q1.exp(1.0f) << std::endl;

    std::cout << q1.slerp(q2, 0.0f) << std::endl;
    std::cout << q1.slerp(q2, 1.0f) << std::endl;
    std::cout << q1.slerp(q2, 0.5f) << std::endl;

    std::cout << q1.slerp(q1, 0.0f) << std::endl;
    std::cout << q1.slerp(q1, 0.5f) << std::endl;
    std::cout << q1.slerp(q1, 1.0f) << std::endl;


    std::cout << std::endl << "Integration:" << std::endl;
    
    Quaternion rot_vec;
    Quaternion rot_acc;
    Quaternion rot;

    Quaternion r1 = Quaternion::rotate(10.f / 180.f * M_PI, 0, 0, 1.0f);
    Quaternion r2 = Quaternion::rotate(-10.f / 180.f * M_PI, 0, 0, 1.0f);


    float n = 10.0f;
    float t = 1.0f / n;
    float t2 = 1.0f / 10.0f;

    rot_acc = r1 * rot_acc;
    rot_vec = rot_acc.slerp(t2) * rot_vec;
    rot = rot_vec.slerp(t2) * rot;

    std::cout << "Rot: " << rot     << std::endl;
    std::cout << "Vec: " << rot_vec << std::endl;
    std::cout << "Acc: " << rot_acc << std::endl;
    std::cout << "Vec: " << rot_vec.slerp(t) << std::endl;
    std::cout << "Acc: " << rot_acc.slerp(t) << std::endl;
    std::cout << std::endl;

    rot_acc = r2 * rot_acc;
    rot_vec = rot_acc.slerp(t2) * rot_vec;
    rot = rot_vec.slerp(t2) * rot;

    std::cout << "Rot: " << rot     << std::endl;
    std::cout << "Vec: " << rot_vec << std::endl;
    std::cout << "Acc: " << rot_acc << std::endl;
    std::cout << std::endl;

    for(unsigned i = 0; i < (int) n - 1; i++) {
        rot_vec = rot_acc.slerp(t) * rot_vec;
        rot = rot_vec.slerp(t) * rot;
    }

    std::cout << "Rot: " << rot     << std::endl;
    std::cout << "Vec: " << rot_vec << std::endl;
    std::cout << "Acc: " << rot_acc << std::endl;
    std::cout << (::acos(rot[0]) * 180 / M_PI) << std::endl;
    std::cout << (::acos(rot_vec[0]) * 180 / M_PI) << std::endl;
    std::cout << std::endl;

    rot_acc = r2 * rot_acc;
    rot_vec = rot_acc.slerp(t2) * rot_vec;
    rot = rot_vec.slerp(t2) * rot;

    std::cout << "Rot: " << rot     << std::endl;
    std::cout << "Vec: " << rot_vec << std::endl;
    std::cout << "Acc: " << rot_acc << std::endl;
    std::cout << std::endl;

    
    Quaternion q {0.0325482, -0.888301, 0.416549, -0.190654};
    Vector3 v {1.18314, -0.177231, -9.68546};

    Vector3 m1 = q.rotate(Vector3{1.0, 0.0, 0.0});
    Vector3 m2 = q.rotate(Vector3{0.0, 1.0, 0.0});
    Vector3 m3 = q.rotate(Vector3{0.0, 0.0, 1.0});

    std::cout << m1 << std::endl;
    std::cout << m2 << std::endl;
    std::cout << m3 << std::endl;

    return 0;
}

