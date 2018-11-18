#ifndef LIDARS_FUSION_MEASUREMENT_H
#define LIDARS_FUSION_MEASUREMENT_H

#include <stdint-gcc.h>

namespace data_wrappers {

    struct Measurement {

        float angle = 0;
        uint16_t distance = 0;

        Measurement(uint16_t d, float a) {
            angle = a;
            distance = d;
        }

        bool operator<(const Measurement &other) {
            return angle < other.angle;
        }
    };

}

#endif //LIDARS_FUSION_MEASUREMENT_H
