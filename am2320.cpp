#include "am2320.h"

Am2320_HandleTypeDef am2320_Init(I2C* i2c, uint8_t device_address) {
    Am2320_HandleTypeDef am2320;
    am2320.i2c = i2c;
    am2320.device_address = device_address;
    return am2320;
}

uint8_t am2320_ReadValue(Am2320_HandleTypeDef *am2320) {
    char wake_up[1] = {0x00};
    char command[3] = {0x03, 0x00, 0x04};

    am2320->i2c->write(am2320->device_address, wake_up, 1);
    ThisThread::sleep_for(1ms);

    if (am2320->i2c->write(am2320->device_address, command, 3) != 0) {
        return 1; // Error
    }
    ThisThread::sleep_for(2ms);

    if (am2320->i2c->read(am2320->device_address, (char*)am2320->data, 8) != 0) {
        return 2; // Error
    }

    if (am2320->data[0] != 0x03 || am2320->data[1] != 0x04) {
        return 3; // Error
    }

    return 0; // Success
}

void am2320_GetTemperatureAndHumidity(Am2320_HandleTypeDef *am2320, float *temperature, float *humidity) {
    int hum_raw = (am2320->data[2] << 8) | am2320->data[3];
    int temp_raw = (am2320->data[4] << 8) | am2320->data[5];
    *humidity = hum_raw / 10.0;
    *temperature = temp_raw / 10.0;
}
