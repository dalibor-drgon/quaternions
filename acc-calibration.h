#ifndef _ACC_CALIBRATION_H_
#define _ACC_CALIBRATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    int16_t x, y, z;
} acc_calibration_entry;

typedef struct {
    int16_t x, y, z;
} gyro_calibration_entry;

// Returns calculated offsets in `dm`, based on `n` measurements stored in `entries` array, where we need at least 4 different entries, each of them at least ~80 degrees or more apart from each other and that absolute value of all three axes in at least one measurement reach at least 0.7 * g. The measurements must be taken when the device is laying and not moving.
int32_t acc_find_offsets(int16_t dm[3], const acc_calibration_entry * entries, unsigned n);


#ifdef __cplusplus
}
#endif

#endif

