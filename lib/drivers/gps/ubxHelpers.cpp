#include "ubxHelpers.h"
#include "ubxPacket.h"
#include "ubxMessages.h"

static uint8_t magicBytes[2] = {0xB5, 0x62};

void sendUBX(uint8_t* message, uint16_t len, I2C_HandleTypeDef* i2c) {
    uint8_t CK_A{0}, CK_B{0};

    HAL_I2C_Master_Transmit(i2c, 0x42 << 1, magicBytes, 2, 100);
    for(uint16_t i=0;i<len;i++) {
        CK_A = CK_A + message[i];
        CK_B = CK_B + CK_A;
    }

    uint8_t CK[2] = {CK_A, CK_B};
    HAL_I2C_Master_Transmit(i2c, 0x42 << 1, message, len, 100);
    HAL_I2C_Master_Transmit(i2c, 0x42 << 1, CK, 2, 100);
}

UBXPacketReader packet;

template<typename PayloadType>
bool pollUBXPacket(uint8_t* message, uint16_t len, I2C_HandleTypeDef* i2c, PayloadType* payloadBuffer) {
    sendUBX(message, len, i2c);

    uint8_t lenBytes[2];
    uint16_t dataLen = 0; 
    uint8_t buffer[512];

    while(dataLen<1) {
        HAL_I2C_Mem_Read(i2c, 0x42 << 1, 0xFD, 1, lenBytes, 2, 100);

        dataLen = lenBytes[0] << 8 | lenBytes[1];
        if(dataLen==0) {
            HAL_Delay(100);
            continue;
        }

        HAL_I2C_Master_Receive(i2c, 0x42 << 1, buffer, dataLen, 100);
    }

    for(uint16_t i = 0; i<dataLen; i++) {
        if(buffer[i] == 0xB5 && buffer[i+1] == 0x62) {
            i+=2; // skip the header
            while(i<dataLen && !packet.isComplete()) {
                UBXPacketUpdateResult res = packet.update(buffer[i]);
                if(res != UBXPacketUpdateResult::UPDATE_OK) {
                    packet.reset();
                    return false;
                }
                i++;
            }
            
            void* payload = packet.getPayload();
            *payloadBuffer = *(PayloadType*)payload;
            packet.reset();
            return true;
        }
    }
    return false;
}

/**
 * @param pvt pointer to the UBX_NAV_PVT_PAYLOAD struct to fill with the GPS data
 * @return true if we get valid GPS data, false otherwise
 * Blocks until we receive some data from the GPS, and returns
 * true if that data is valid, false otherwise (non-UBX packet or checksum failed)
*/
bool checkGPSFullNavInfo(UBX_NAV_PVT_PAYLOAD* pvt, I2C_HandleTypeDef* i2c) {
    uint8_t navMessage[4] = {0x01, 0x07, 0x00, 0x00};
    return pollUBXPacket(navMessage, 4, i2c, pvt);
}

bool checkGPSNavStatus(UBX_NAV_STATUS_PAYLOAD* status, I2C_HandleTypeDef* i2c) {
    uint8_t navMessage[4] = {0x01, 0x03, 0x00, 0x00};
    return pollUBXPacket(navMessage, 4, i2c, status);
}

bool checkGPSNavSatellites(UBX_NAV_SAT_PAYLOAD* sat, I2C_HandleTypeDef* i2c) {
    uint8_t navMessage[4] = {0x01, 0x35, 0x00, 0x00};
    return pollUBXPacket(navMessage, 4, i2c, sat);
}