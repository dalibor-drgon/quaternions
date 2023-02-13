
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "acc-calibration.h"

typedef struct {
    float x, y, z;
} acc_entry;

static inline float cost_function_local(const float *dm, float *der, acc_entry entry) {
    float dif_x = entry.x - dm[0];
    float dif_y = entry.y - dm[1];
    float dif_z = entry.z - dm[2];
    float len = sqrtf(dif_x * dif_x + dif_y * dif_y + dif_z * dif_z);
    //printf("%f %f %f %f\n", dif_x, dif_y, dif_z, len);
    float dif = len - 9.81;
    if (der) {
        der[0] = -2 * dif * dif_x / len;
        der[1] = -2 * dif * dif_y / len;
        der[2] = -2 * dif * dif_z / len;
    }
    return dif * dif;
}

static float cost_function(const float *dm, float *der,
        const acc_entry * entries, unsigned n) {
    if(der) {
        der[0] = der[1] = der[2] = 0.0f;
    }
    float cost = 0;
    for (unsigned i = 0; i < n; i++) {
        float local_der[3];
        cost = cost_function_local(dm, local_der, entries[i]);
        if(der) {
            for(int j = 0; j < 3; j++) {
                der[j] += local_der[j];
            }
        }
    }
    //printf("%p %p %f\n", dm, der, cost);
    return cost;
}

#if 0
#include <nlopt.h>

typedef struct {
    const acc_entry *entries;
    unsigned n;
} acc_data;

static float cost_function_nlopt(unsigned n, const float *args, float *der, void *f_data) {
    acc_data *data = (acc_data *) f_data;
    return cost_function(args, der, data->entries, data->n);
}

static float find_offsets(float *dm, 
        const acc_entry * entries, unsigned n) {
    acc_data data = {.entries = entries, .n = n};
    nlopt_opt opt = nlopt_create(NLOPT_LD_SLSQP, 3);
    nlopt_set_min_objective(opt, cost_function_nlopt, &data);
    nlopt_set_xtol_rel(opt, 0.0001);
    
    float res = 0.0f;
    int ret = nlopt_optimize(opt, dm, &res);
    printf("Returned %d with cost %f\n", ret, res);
    return res;
}
#elif 0
float coef(int i) {
    return 0.35;
}

static float find_offsets(float *dm, 
        const acc_entry * entries, unsigned n) {
    float cost = 0;
    for(unsigned i = 0; i < 2; i++) {
        float der[3] = {0};
        cost = cost_function(dm, der, entries, n);
        for(unsigned j = 0; j < 3; j++) {
            dm[j] -= coef(i) * der[j];
        }
    }
    return cost;
}
#else
#include <eigen3/Eigen/Dense>
using namespace Eigen;

static float find_guess(float *dm, 
        const acc_entry * entries, unsigned n) {

    // Create matrices A and b for the equation: A * dm = b
    MatrixXf A(n-1, 3);
    VectorXf b(n-1);
    Vector3f first = Vector3f{(float) entries[0].x, (float) entries[0].y, (float) entries[0].z};
    float first_dot = first.dot(first);
    for(unsigned i = 1; i < n; i++) {
        Vector3f cur = Vector3f{(float) entries[i].x, (float) entries[i].y, (float) entries[i].z};
        A.row(i-1) = 2 * (cur - first);
        b[i-1] = cur.dot(cur) - first_dot;
    }

    Vector3f res = (A.transpose() * A).inverse() * A.transpose() * b;
    for(unsigned j = 0; j < 3; j++) {
        dm[j] = res[j];
    }
    return cost_function(dm, NULL, entries, n);
}

#include "optimizer.hpp"

//float minimize_muller_step(float p0, float p1, float p2, float y0, float y1, float y2);

static float find_offsets(float *dm, 
        const acc_entry * entries, unsigned n) {
    //find_guess(dm, entries, n);
    float cost = 0;
    for(unsigned i = 0; i < 4; i++) {
        float der[3] = {0};
        cost = cost_function(dm, der, entries, n);
        printf("%f, %f, %f\n", der[0], der[1], der[2]);
        float p0 = 0.0;
        float p1 = -0.3;
        float p2 = -1.0;
        float y0 = cost_function(dm, NULL, entries, n);
        for(unsigned j = 0; j < 3; j++) {
            dm[j] += (p1 - p0) * der[j];
        }
        float y1 = cost_function(dm, NULL, entries, n);
        for(unsigned j = 0; j < 3; j++) {
            dm[j] += (p2 - p1) * der[j];
        }
        float y2 = cost_function(dm, NULL, entries, n);
        float p3 = minimize_muller_step(p0, p1, p2, y0, y1, y2);
        for(unsigned j = 0; j < 3; j++) {
            dm[j] += (p3 - p2) * der[j];
        }
        printf("f(p3=%f)=%f\n", p3, cost_function(dm, NULL, entries, n));
    }
    cost = cost_function(dm, NULL, entries, n);
    return cost;
}
#endif

#if 1
#define F(x) (x * (9.81 / 2048.0))
#define FI(y) (y * (2048.0 / 9.81))
#define FI2(y) y * (2048.0 * 2048.0 / (9.81 * 9.81))
#else
#define F(x) x
#define FI(y) y
#define FI2(y) y
#endif

extern "C" int32_t acc_find_offsets(int16_t dm[3], const acc_calibration_entry * entries, unsigned n) {
    acc_entry *entries_float = (acc_entry*) malloc(sizeof(*entries_float) * n);
    for (unsigned i = 0; i < n; i++) {
        entries_float[i].x = F(entries[i].x);
        entries_float[i].y = F(entries[i].y);
        entries_float[i].z = F(entries[i].z);
    }
    float dm_float[3];
    float cost = find_offsets(dm_float, entries_float, n);
    for(unsigned j = 0; j < 3; j++) {
        dm[j] = (int16_t) round(FI(dm_float[j]));
    }
    return (int32_t) FI2(cost);
}

int main(int argc, const char **argp) {
#if 0
    acc_entry entries[4] = {
        {9.81+0.2, 0+0.1, 0+0.3},
        {-9.81+0.2, 0+0.1, 0+0.3},
        {0.2, -9.81+0.1, 0.3},
        {0.2, 0.1, 9.81+0.3}
    };
#endif
#if 0
    acc_calibration_entry entries[4] = {
        {(-978),(-1807),(142)},
        {(84),(-1518),(-1334)},
        {(-1521),(188),(-1367)},
        {(208),(-206),(2052)}
    };

    int16_t off[3] = {0};
    float cost = acc_find_offsets(off, entries, sizeof(entries)/sizeof(*entries));
    printf("Found offsets [%d] %d %d %d\n", (int) cost, (int) off[0], (int) off[1], (int) off[2]);
#else
    acc_entry entries[6] = {
        {-1.059890,0.399846,10.054238},
        {0.075886,9.409678,0.340540},
        {0.053654,-10.080501,-0.165683},
        {-9.706564,-0.417788,-0.308230},
        {9.862615,-0.214275,0.494906},
        {0.052734,-0.165477,-9.570475}
    };

    float off[3] = {0};
    float cost = find_offsets(off, entries, sizeof(entries)/sizeof(*entries));
    printf("Found offsets [%f] %f %f %f\n", cost, off[0], off[1], off[2]);
#endif
#if 0
    printf("data=[");
    for(unsigned i = 0; i < 4; i++) {
        printf("%f %f %f; ", entries[i].x, entries[i].y, entries[i].z);
    }
    printf("]\n");
#endif
}

