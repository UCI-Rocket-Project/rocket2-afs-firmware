#pragma once
#include <stdint.h>

enum class GPSFixType : uint8_t {
    NO_FIX,
    DEAD_RECKONING,
    FIX_2D,
    FIX_3D,
    GNSS_AND_DEAD_RECKONING,
    TIME_ONLY_FIX
};

static uint8_t PVT_MESSAGE[4] = {0x01, 0x06, 0x00, 0x00};

// https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
#pragma pack(push, 1)
struct UBX_NAV_PVT_PAYLOAD {
    uint32_t iTOW;
    int32_t fTOW;
    int16_t week;
    GPSFixType gpsFix;
    uint8_t flags;
    int32_t ecefX;
    int32_t ecefY;
    int32_t ecefZ;
    uint32_t pAcc;
    int32_t ecefVX;
    int32_t ecefVY;
    int32_t ecefVZ;
    uint32_t sAcc;
    uint16_t pDOP;
    uint8_t reserved1;
    uint8_t numSV;
    uint8_t reserved2[4];
};
#pragma pack(pop)

#pragma pack(push, 1)
struct UCIRP_GPS_PAYLOAD {  // our custom struct
    GPSFixType gpsFix;
    double ecefX;
    double ecefY;
    double ecefZ;
    double positionAcc;
    double ecefVX;
    double ecefVY;
    double ecefVZ;
    double speedAcc;
};
#pragma pack(pop)
