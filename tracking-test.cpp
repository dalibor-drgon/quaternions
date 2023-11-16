#include <array>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>

#include "tracking.hpp"
#include "history.hpp"
#include <Fusion.h>


using namespace std;
using namespace Eigen;


// Initialise algorithms
FusionOffset offset;
FusionAhrs ahrs;
Tracking track;

void on_measurement(array<double, 6> arr);

void split(const string & str, double *out, unsigned cnt) {
    string::size_type start = 0;
    for(unsigned i = 0; i < cnt; i++) {
        string::size_type index = str.find(",", start);
        string occurence = str.substr(start, index == string::npos ? string::npos : (index - start));
        //cout << occurence << endl;
        out[i] = stod(occurence);
        start = index + 1;
    }
}

void parse6(vector<array<double, 6>> &out, ifstream & in) {
    string line;
    while (getline(in, line)) {
        if (line == "") 
            continue;
        double arr[6];
        split(line, arr, sizeof(arr)/sizeof(*arr));
        out.push_back(array<double, 6>{arr[0], arr[1], arr[2], arr[3], arr[4], arr[5]});
    }
}

void parse6(ifstream & in) {
    string line;
    while (getline(in, line)) {
        if (line == "") 
            continue;
        double arr[6];
        split(line, arr, sizeof(arr)/sizeof(*arr));
        array<double, 6> data {arr[0], arr[1], arr[2], arr[3], arr[4], arr[5]};
        on_measurement(data);
    }
}

Vector3f from_acc(double x, double y, double z) {
    Vector3f vec = Vector3f{(float) x, (float) y, (float) z} * (9.81 / 2048 / 4);
    //cerr << "Acc:  " << vec << endl;
    //vec = vec - Vector3f{-0.159, 0.030, 0.139};
    vec -= Vector3f{0.190, -0.378, 0.304};
    //vec = vec - Vector3f{-0.249111, 0.177253, -0.341889};
    return vec * (1.0f / 9.81);
}

#define DX 10

Quaternionf from_gyro(const Vector3f & vec, float dt) {
    Quaternionf rot (
        AngleAxisf(vec[0] * dt, Vector3f::UnitX()) *
        AngleAxisf(vec[1] * dt, Vector3f::UnitY()) *
        AngleAxisf(vec[2] * dt, Vector3f::UnitZ()));
    return rot;

}

#define max(a,b) (a > b) ? (a) : (b)
#define min(a,b) (a < b) ? (a) : (b)

double check(double gyr) {
    if (gyr > 0)
        return max(gyr - 1, 0);
    else 
        return min(gyr + 1, 0);
}

Vector3f from_gyro(double x, double y, double z) {
    //x = check(x+14);
    //y = check(y-50);
    //z = check(z+2);
    x = check(x+10);
    y = check(y-13);
    z = check(z-7);

    Vector3f vec{(float) x, (float) y, (float) z};
    return vec * (1.0f / 32.768);
    //return vec * (1.0f / 16);

}

FusionVector to_fusion(const Eigen::Vector3f & vec) {
    return (FusionVector) {vec[0], vec[1], vec[2]};
}

Vector3f to_eigen(const FusionVector & vec) {
    return Vector3f { vec.array[0], vec.array[1], vec.array[2] };
}

Vector3f to_eigen(const FusionEuler & vec) {
    return Vector3f { vec.array[0], vec.array[1], vec.array[2] };
}

void on_measurement(array<double, 6> arr) {
    float dt = 4e-3;
    Vector3f acc = from_acc(arr[0], arr[1], arr[2]);
    Vector3f gyr = from_gyro(arr[3], arr[4], arr[5]);

    // Update gyroscope offset correction algorithm
    FusionVector gyroscope = FusionOffsetUpdate(&offset, to_fusion(gyr));
    FusionVector accelerometer = to_fusion(acc);


    static int cnt = 0;
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dt);
    //Vector3f linacc = to_eigen(FusionAhrsGetLinearAcceleration(&ahrs)) * 9.81;
    Vector3f linacc = to_eigen(FusionAhrsGetEarthAcceleration(&ahrs)) * 9.81;

    Vector3f axes = to_eigen(FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs)));
    //cerr << axes.format(EigenCommaFormat) << "\t";
    cerr << to_eigen(gyroscope).format(EigenCommaFormat) << "\t";

    if (cnt == 200) {
        track.on_entry_acc(linacc, dt);
    } else cnt++;

}


int main(int argc, char **argp) {
    if (argc != 2) {
        return 1;
    }
    cout << fixed << setprecision(6);
    cerr << fixed << setprecision(4);

    FusionOffsetInitialise(&offset, 200);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .gain = 0.5f,
            .accelerationRejection = 10.0f,
            .magneticRejection = 20.0f,
            .rejectionTimeout = 200, 
    };
    FusionAhrsSetSettings(&ahrs, &settings);


    ifstream in(argp[1]);
    parse6(in);
    return 0;
}

