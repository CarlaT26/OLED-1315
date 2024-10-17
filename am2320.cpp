#include "Adafruit_AM2320.h"
#include "mbed.h"

// Se asume que los pines de I2C están definidos en el sistema Mbed
I2C i2c_bus(D14, D15); // Pines SCL y SDA de Mbed (ajusta según tu placa)

// Constructor de la clase, inicializa el objeto I2C
Adafruit_AM2320::Adafruit_AM2320(I2C *theI2C, int32_t tempSensorId, int32_t humiditySensorId)
    : _temp(this, tempSensorId), _humidity(this, humiditySensorId), i2c_dev(theI2C) {}

/**************************************************************************/
/*!
    @brief  Setups the hardware
    @return true if sensor initialized correctly
*/
/**************************************************************************/
bool Adafruit_AM2320::begin() {
  char wakeup_command[1] = {0x00};
  int ack = i2c_dev->write(0xB8, wakeup_command, 1); // Dirección del sensor AM2320 con un write
  
  if (ack != 0) {
    return false; // No se recibió respuesta del sensor
  }
  
  // Despertar el sensor
  ThisThread::sleep_for(10ms); // Esperar 10 ms
  return true;
}

/**************************************************************************/
/*!
    @brief  Lee la temperatura del dispositivo
    @return la temperatura en un valor de punto flotante
*/
/**************************************************************************/
float Adafruit_AM2320::readTemperature() {
  uint32_t data = readRegister32(AM2320_REG_TEMP_H);
  if (data == 0xFFFFFFFF)
    return NAN;

  float temp;
  if (data & 0x8000) {
    temp = -(int16_t)(data & 0x7FFF);
  } else {
    temp = (int16_t)(data & 0xFFFF);
  }
  return temp / 10.0;
}

/**************************************************************************/
/*!
    @brief  Lee la humedad del dispositivo
    @return la humedad como un valor de punto flotante
*/
/**************************************************************************/
float Adafruit_AM2320::readHumidity() {
  uint32_t data = readRegister32(AM2320_REG_HUM_H);
  if (data == 0xFFFFFFFF)
    return NAN;

  return (data >> 16) / 10.0;
}

/**************************************************************************/
/*!
    @brief  Lee 4 bytes de un registro del sensor
    @param reg el registro a leer
    @return el valor leído como un entero de 4 bytes
*/
/**************************************************************************/
uint32_t Adafruit_AM2320::readRegister32(uint8_t reg) {
  char buffer[8] = {0};
  char cmd[3] = {AM2320_CMD_READREG, reg, 4}; // 4 bytes

  // Intentar leer hasta 3 veces
  for (int i = 0; i < 3; i++) {
    int ack = i2c_dev->write(0xB8, cmd, 3); // Enviar comando de lectura
    if (ack == 0) {
      break;
    }
    ThisThread::sleep_for(5ms);
  }

  // Leer respuesta del sensor
  int result = i2c_dev->read(0xB8, buffer, 8);
  if (result != 0 || buffer[0] != 0x03 || buffer[1] != 4) {
    return 0xFFFFFFFF; // Error de lectura
  }

  // CRC y datos
  uint32_t ret = (buffer[2] << 24) | (buffer[3] << 16) | (buffer[4] << 8) | buffer[5];
  return ret;
}

/**************************************************************************/
/*!
    @brief  Realiza un chequeo CRC
    @param buffer el puntero a los datos a verificar
    @param nbytes la cantidad de bytes para calcular el CRC
    @return el CRC calculado
*/
/**************************************************************************/
uint16_t Adafruit_AM2320::crc16(uint8_t *buffer, uint8_t nbytes) {
  uint16_t crc = 0xffff;
  for (int i = 0; i < nbytes; i++) {
    uint8_t b = buffer[i];
    crc ^= b;
    for (int x = 0; x < 8; x++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}
