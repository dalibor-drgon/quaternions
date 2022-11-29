#ifndef _QUATERNION_HPP_
#define _QUATERNION_HPP_

#include <math.h>
#include <iostream>
#include "vector3.hpp"

class Quaternion {
private:
    float values[4] = { 0.0f };

public:
    Quaternion()
            : values{1.0f, 0.0f, 0.0f, 0.0f} {}

    Quaternion(float i, float j, float k)
            : values{0.0f, i, j, k} {
    }

    Quaternion(float x, const Quaternion & v)
            : values{x, v.values[1], v.values[2], v.values[3]} {
    }

    Quaternion(const Vector3 & vec)
            : values{0.0f, vec[0], vec[1], vec[2]} {
    }

    Quaternion(float x, const Vector3 & vec)
            : values{x, vec[0], vec[1], vec[2]} {
    }

    Quaternion(float x, float i, float j, float k)
            : values{x, i, j, k} {
    }

    Quaternion operator*(const Quaternion & x) const {
        return Quaternion(
            values[0] * x.values[0] - values[1] * x.values[1] - values[2] * x.values[2] - values[3] * x.values[3],
            values[0] * x.values[1] + values[1] * x.values[0] + values[2] * x.values[3] - values[3] * x.values[2],
            values[0] * x.values[2] + values[2] * x.values[0] + values[3] * x.values[1] - values[1] * x.values[3],
            values[0] * x.values[3] + values[3] * x.values[0] + values[1] * x.values[2] - values[2] * x.values[1]
        );
    }

    float dot(const Quaternion & x) const {
        return values[0] * x.values[0] + values[1] * x.values[1] + values[2] * x.values[2] + values[3] * x.values[3];
    }

    Quaternion operator*(float f) const {
        return Quaternion (
            f * values[0], f * values[1], f * values[2], f * values[3]
        );
    }

    Quaternion operator+(const Quaternion & x) const {
        return Quaternion (
            values[0] + x.values[0], values[1] + x.values[1], values[2] + x.values[2], values[3] + x.values[3]
        );
    }

    Quaternion conjugate() const {
        return Quaternion (
            values[0], -values[1], -values[2], -values[3]
        );
    }

    float length() const {
        return sqrt(dot(*this));
    }

    Quaternion normalize() const {
        return *this * (1.0f / length());
    }

    Quaternion inverse() const {
        return conjugate() * (1.0f / dot(*this));
    }

    Quaternion log() const {
        Quaternion v (0.0f, *this);
        float v_len = v.length();
        float q_len = length();
        if (v_len == 0.0f)
            return Quaternion(
                ::log(q_len), 0.0f, 0.0f, 0.0f
            );
        return Quaternion(
            ::log(q_len),
            v * (::acos(values[0] / q_len) / v_len)
        );
    }

    Quaternion exp() const {
        Quaternion v (0.0f, *this);
        float v_len = v.length();
        if (v_len == 0.0f) {
            return Quaternion(::exp(values[0]), 0.0f, 0.0f, 0.0f);
        }
        return Quaternion(
            ::cos(v_len),
            v * (::sin(v_len) / v_len)
        ) * (::exp(values[0]));
    }

    Quaternion exp(float t) const {
        return (log() * t).exp();
    }

    // Same as exp(t)
    Quaternion slerp(float t) const {
        return Quaternion().slerp(*this, t);
    }

    Quaternion slerp(const Quaternion & p2, float t) const {
        float ratio = dot(p2) / (length() * p2.length());
        float theta = ::acos(ratio);
        if (ratio >= 0.9999)
            return *this;
        return *this * (::sin((1.0f - t) * theta) / ::sin(theta)) + p2 * (::sin(t * theta) / ::sin(theta));
    }

    float operator[](unsigned n) const {
        return values[n];
    }

    friend std::ostream & operator<<(std::ostream &out, const Quaternion & q) {
        return out << "[" << q.values[0] << ", " << q.values[1] << ", " << q.values[2] << ", " << q.values[3] << "]";
    }

    static Quaternion rotate(float angle, float i, float j, float k) {
        return Quaternion(
            ::cos(angle),
            Quaternion(i,j,k) * ::sin(angle)
        );
    }

    Vector3 rotate(const Vector3 & point) const {
        Quaternion res = *this * Quaternion(point) * this->inverse();
        return Vector3(res[1], res[2], res[3]);
    }

};


#endif

