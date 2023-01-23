#ifndef _VECTOR3_HPP_
#define _VECTOR3_HPP_

#include <iostream>

class Vector3 {
    double values[3] = {0.0f, 0.0f, 0.0f};

public:
    Vector3() {}
    Vector3(double x, double y, double z)
        : values{x, y, z} {}
    Vector3(const Vector3 & x)
        : values{x.values[0], x.values[1], x.values[2]} {}

    Vector3 operator*(double alpha) const {
        return Vector3(
            values[0] * alpha, values[1] * alpha, values[2] * alpha
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

    double dot(const Vector3 & vec) const {
        return values[0] * vec[0] + values[1] * vec[1] + values[2] * vec[2];
    }

    const double & operator[](unsigned n) const {
        return values[n];
    }

    double & operator[](unsigned n) {
        return values[n];
    }

    Vector3 matrixMult(const Vector3 & r1, const Vector3 & r2, const Vector3 & r3) {
        return Vector3(
            this->dot(r1),
            this->dot(r2),
            this->dot(r3)
        );
    }

    friend std::ostream & operator<<(std::ostream &out, const Vector3 & q) {
        return out << "[" << q.values[0] << ", " << q.values[1] << ", " << q.values[2] << "]";
    }
};

class Matrix3 {
public:
    Vector3 cols[3];

    Matrix3 operator+(const Matrix3 & mat) const {
        return Matrix3{
            cols[0] + mat.cols[0],
            cols[1] + mat.cols[1],
            cols[2] + mat.cols[2]
        };
    }

    Vector3 operator*(const Vector3 & vec) const {
        return Vector3 {
            cols[0].dot(vec),
            cols[1].dot(vec),
            cols[2].dot(vec)
        };
    }

    Matrix3 transpose() const {
        return Matrix3();
    }

    static Matrix3 mult3x3(const Vector3 & x, const Vector3 & y) {
        return Matrix3 {
            Vector3{ x[0] * y[0], x[0] * y[1], x[0] * y[2] },
            Vector3{ x[1] * y[0], x[1] * y[1], x[1] * y[2] },
            Vector3{ x[2] * y[0], x[2] * y[1], x[2] * y[2] }
        };
    }

};


#endif

