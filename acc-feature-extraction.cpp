

#if 1

#include <iostream>
#include <fstream>
#include "acc-feature-extraction.hpp"
#include <iomanip>

using namespace std;

AccFeatureExtraction ext;

int main(int argc, const char **argp) {
    ifstream fd(argp[1]);
    cout << setprecision(4) << fixed;
    float cur;
    while (fd >> cur) {
        ext.extract(cur, 0.004);
    }
}

#endif

