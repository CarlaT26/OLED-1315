#ifndef INC_AM2320_H_
#define INC_AM2320_H_

#include "mbed.h"

#define AM2320_ADDRESS (0x5C << 1)

typedef struct {
    I2C* i2c;
    uint8_t device_address;
    uint8_t data[8];
} Am2320_HandleTypeDef;

Am2320_HandleTypeDef am2320_Init(I2C* i2c, uint8_t device_address);
uint8_t am2320_ReadValue(Am2320_HandleTypeDef *am2320);
void am2320_GetTemperatureAndHumidity(Am2320_HandleTypeDef *am2320, float *temperature, float *humidity);

#endif /* INC_AM2320_H_ */
