#ifndef _VECTOR3_HPP_
#define _VECTOR3_HPP_

#include <iostream>

class Vector3 {
    float values[3] = {0.0f, 0.0f, 0.0f};

public:
    Vector3() {}
    Vector3(float x, float y, float z)
        : values{x, y, z} {}
    Vector3(const Vector3 & x)
        : values{x.values[0], x.values[1], x.values[2]} {}

    Vector3 operator*(float alpha) const {
        return Vector3(
            values[0], values[1], values[2]
        );
    }

    Vector3 operator+(const Vector3 & vec) const {
        return Vector3(
            values[0] + vec.values[0], values[1] + vec.values[1], values[2] + vec.values[2]
        );
    }

    Vector3 operator-(const Vector3 & vec) const {
        return Vector3(
            values[0] - vec.values[0], values[1] - vec.values[1], values[2] - vec.values[2]
        );
    }

    float operator[](unsigned n) const {
        return values[n];
    }

    friend std::ostream & operator<<(std::ostream &out, const Vector3 & q) {
        return out << "[" << q.values[0] << ", " << q.values[1] << ", " << q.values[2] << "]";
    }
};


#endif

