
#include "tracking.hpp"
#include <iomanip>
#include <iostream>
#include "kalman.hpp"
#include <memory>

#define DT (1/250.f)

using namespace Eigen;
using namespace std;


#include "history.hpp"
//#include "acc-feature-extraction.hpp"
#include <deque>
#include <vector>

//AccFeatureExtraction ext0, ext1, ext2;

History<Vector3f,1> hist_acc;

vector<Vector3f> accs;

static void show_me_baby(float dt, Vector3f &p_vec, Vector3f &p) {
    vector<Vector3f> vecs;
    Vector3f cur_vec = Vector3f::Zero();
    for(Vector3f & acc : accs) {
        cur_vec += acc * dt;
        vecs.push_back(cur_vec);
    }
    

    Vector3f vec_base = vecs[0];
    Vector3f vec_m = (vecs[vecs.size()-1] - vecs[0]) * (1.0f / (vecs.size() - 1));
    for(unsigned i = 0; i < vecs.size(); i++) {
        vecs[i] -= vec_base + vec_m * (float) i;
    }

    p_vec = Vector3f::Zero();
    for(unsigned i = 0; i < vecs.size(); i++) {
        p += vecs[i] * dt;
    }
}

void Tracking::on_entry_acc(Eigen::Vector3f acc_dif, float dt) {
    hist_acc.add(acc_dif);
    Vector3f acc_avg = hist_acc.average(Vector3f{0,0,0});

    if (acc_avg.dot(acc_avg) >= pow(1.0, 2)) {
        accs.push_back(acc_avg);
    } else {
        if (accs.size() > 2)
            show_me_baby(dt, p_vec, p);
        accs.clear();
    }

    cerr << acc_dif.format(EigenCommaFormat) << "\t" << p_vec.format(EigenCommaFormat) << "\t" << p.format(EigenCommaFormat) << endl;
    cout << acc_dif.format(EigenCsvFormat) << "," << p_vec.format(EigenCsvFormat) << "," << p.format(EigenCsvFormat) << endl;
}



