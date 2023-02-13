
#ifndef _ACC_FEATURE_EXTRACTION_HPP_
#define _ACC_FEATURE_EXTRACTION_HPP_

#include <iostream>

#define ACCERR 0.5

class AccFeatureExtraction {

public:
    float prev_veloc = 0;
    float veloc;
    float distance;

    unsigned cnt = 0;
    unsigned lid_up = 0;
    unsigned lid_down = 0;
    float li_up = 0;
    float li_down = 0;
    unsigned last_dir = 0;

    void reset() {
        lid_up = lid_down = 0;
        li_up = li_down = 0;
        cnt = 0;
        veloc = 0;
    }

    float extract(float acc, float dt) {
        if (acc > -ACCERR && acc < ACCERR) {
            acc = 0;
        }
        if(veloc != 0) {
            cnt++;
        }
        unsigned dir = 0;
        if (acc > 0) {
            lid_up += 1;
            li_up += acc;
        } else {
            dir = 1;
            lid_down += 1;
            li_down += acc;
        }
        veloc += acc * dt;

        //float dif0 = li_up / (lid_up+1);
        //float dif1 = li_down / (lid_down+1);
        float dif = veloc / (cnt+1) / dt;
        float dif_req = ACCERR;
        //if (-dif_req < dif0+dif1 && dif0+dif1 < dif_req)
        if(-dif_req < dif && dif < dif_req) {
            if (last_dir == dir && abs(veloc) < abs(prev_veloc)) {
                // continue
            } else {
                reset();
            }
        }

        last_dir = dir;
        distance = distance + veloc * dt;
        prev_veloc = veloc;

        //std::cout << acc << "\t" << cnt << "\t" << veloc << "\t" << distance << "\t" << dif << "\t" << dif << std::endl;
        return distance;
    }

    static float abs(float x) {
        return (x > 0) ? x : -x;
    }

    static bool is_smaller(float x, float y, unsigned dir) {
        return (dir == 0) ? (x < y) : (x > y);
    }

};


#endif
