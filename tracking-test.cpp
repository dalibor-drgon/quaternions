#include <array>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>

#include "tracking.hpp"


using namespace std;
using namespace Eigen;

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
    Vector3f vec = Vector3f{(float) x, (float) y, (float) z} * (9.81 / 2048);
    //cerr << "Acc:  " << vec << endl;
    vec = vec - Vector3f{-0.159, 0.030, 0.139};
    return vec;
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
        return max(gyr - 2, 0);
    else 
        return min(gyr + 2, 0);
}

Quaternionf from_gyro(double x, double y, double z, float dt) {
    x = check(x+5);
    y = check(y-7);
    z = check(z-3);

    Vector3f vec{(float) x, (float) y, (float) z};
    vec = vec * (M_PI / 180 / (32.768 / 2));
    //cerr << "Gyro: " << vec << endl;
    return from_gyro(vec, dt);
}

void on_measurement(array<double, 6> arr) {
    static int cnt = 0;
    if (++cnt == 1) {
        Tracking track_(from_acc(arr[0], arr[1], arr[2]));
        track = track_;
    }

    float dt = 4e-3;
    Vector3f acc = from_acc(arr[0], arr[1], arr[2]);
    Quaternionf gyr = from_gyro(arr[3], arr[4], arr[5], dt);
    track.on_entry(gyr, acc, dt);
    //cout << "GyrQ: " << gyr << endl;
    //cout << track.rot << "\t" << acc.format(EigenCommaFormat) << "\t" << track.p_acc.format(EigenCommaFormat) << endl;
#if 0
    cout << "Rota: " << track.rot << endl;
    cout << "Acce: " << track.p_acc << endl;
    cout << "Vect: " << track.p_vec << endl;
    cout << "Loca: " << track.p << endl;
#endif
}


int main(int argc, char **argp) {
    if (argc != 2) {
        return 1;
    }
    cout << fixed << setprecision(4);
    ifstream in(argp[1]);


#if 0
    vector<array<double, 6>> vec;
    parse6(vec, in);

    if (0) {
        float dt = 1.0;
        cout << Quaternion::rotate(M_PI, 1, 0, 0) << endl;
        cout << from_gyro(Vector3(1, 0, 0), dt) << endl;
        cout << from_gyro(Vector3(0, 1, 0), dt) << endl;
        cout << from_gyro(Vector3(0, 0, 1), dt) << endl;
        cout << from_gyro(Vector3(1, 0, 1), dt) << endl;
        return 0;
    }
    for (auto & arr : vec) {
        on_measurement(arr);
    }
#else
    parse6(in);
#endif

    return 0;
}

