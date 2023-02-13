
#include <iostream>
#include <fstream>
#include <array>
#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <iomanip>

using namespace std;
using namespace Eigen;

void split(const string & str, double *out, unsigned cnt) {
    string::size_type start = 0;
    for(unsigned i = 0; i < cnt; i++) {
        string::size_type index = str.find(",", start);
        string occurence = str.substr(start, index == string::npos ? string::npos : (index - start));
        //cout << occurence << endl;
        out[i] = stod(occurence) * 9.81 / (2048 * 4);
        start = index + 1;
    }
}

void process(double data[3]);

void parse3(ifstream & in) {
    string line;
    int cnt = 0;
    while (getline(in, line)) {
        if (line == "") 
            continue;
        double arr[3];
        split(line, arr, sizeof(arr)/sizeof(*arr));
        if(cnt > 10)
            process(arr);
        else cnt++;
        //out.push_back(array<double, 4>{arr[0], arr[1], arr[2], arr[3]});
    }
}

#include "history.hpp"

History<Vector3f,4> hist;

vector<Vector3f> measurements;
Vector3f sum{0,0,0};
unsigned sum_len = 1;

#include "tracking.hpp"


vector<Vector3f> last_entries;

void on_entry(Vector3f acc) {
    bool contains = false;
    for(Vector3f entry : last_entries) {
        if(entry.normalized().dot(acc.normalized()) > 0.7) {
            contains = true;
            break;
        }
    }
    if (!contains) {
        last_entries.push_back(acc);
        cout << acc.format(EigenCsvFormat) << endl;
    }
}

void process(double data[3]) {
    Vector3f x{(float) data[0], (float) data[1], (float) data[2]};
    hist.add(x);
    x = hist.average(Vector3f{0,0,0});
    Vector3f sum_cur = sum * (1.0f / sum_len);
    Vector3f dif = x - sum_cur;
    //cout << dif.format(EigenCommaFormat) << " " << dif.dot(dif) << endl;
    if (dif.dot(dif) < 0.05) {
        sum += x;
        sum_len ++;
    } else {
        if (sum_len > 50) {
            float len = sqrt(sum_cur.dot(sum_cur));
            if (abs(len - 9.81) < 1) {
                on_entry(sum_cur);
            }
        }
        sum_len = 1;
        sum = x;
    }
}

int main(int argc, char **argp) {
    const char *file = "/home/deli/workspace/esp/scan/client/traffic.csv";
    if (argc > 1)
        file = argp[1];

    ifstream is(file);
    cout << fixed << setprecision(6);
    parse3(is);
    double zeros[3] = {0,0,0};
    process(zeros);
}
