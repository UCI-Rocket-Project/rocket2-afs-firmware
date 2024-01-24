#ifndef GPS_H
#define GPS_H

#include <cstdint>

// https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf

enum GPSFix {
    NO_FIX = 0,
    DEAD_RECKONING_ONLY = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    GPS_AND_DEAD_RECKONING = 4,
    TIME_ONLY_FIX = 5
};


// datasheet page 388
typedef struct {
    uint32_t iTOW;
    GPSFix gpsFix;
    char flags;
    char fixStat;
    char flags2;
    uint32_t ttff;
    uint32_t msss;
} UBX_NAV_STATUS;

#endif