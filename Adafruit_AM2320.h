#ifndef MBED_AM2320
#define MBED_AM2320

#include "mbed.h"

// Definiciones para el sensor AM2320
#define AM2320_SENSOR_VERSION 1 ///< la versión del sensor
#define AM2320_CMD_READREG 0x03 ///< comando para leer registro
#define AM2320_REG_TEMP_H 0x02  ///< dirección del registro de temperatura
#define AM2320_REG_HUM_H 0x00   ///< dirección del registro de humedad

/**************************************************************************/
/*!
    @brief  Clase que almacena el estado y funciones para interactuar con el
    sensor AM2320 de Temperatura y Humedad
*/
/**************************************************************************/
class Mbed_AM2320 {
public:
  Mbed_AM2320(I2C *i2c, int address = 0x5C); // Constructor con el objeto I2C y la dirección
  bool begin();

  float readTemperature(); // Lee la temperatura
  float readHumidity();    // Lee la humedad
  uint16_t crc16(uint8_t *buffer, uint8_t nbytes); // Calcula el CRC-16 (si es necesario)

private:
  I2C *_i2c;               ///< El objeto I2C
  int _address;            ///< Dirección del sensor en el bus I2C
  bool readData(uint8_t *buffer, uint8_t length); // Función para leer los datos crudos
};

/**************************************************************************/
/*!
    @brief  Constructor para el sensor AM2320
    @param i2c Puntero al objeto I2C
    @param address Dirección del sensor en el bus I2C
*/
/**************************************************************************/
Mbed_AM2320::Mbed_AM2320(I2C *i2c, int address) : _i2c(i2c), _address(address) {}

/**************************************************************************/
/*!
    @brief  Inicializa la comunicación con el sensor
    @return true si la inicialización fue exitosa
*/
/**************************************************************************/
bool Mbed_AM2320::begin() {
  char cmd[3];
  
  // Enviar un dummy write para despertar el sensor
  cmd[0] = 0x00;
  _i2c->write(_address << 1, cmd, 1);
  ThisThread::sleep_for(2ms); // Espera para que el sensor se despierte

  // Verificar si el sensor responde
  if (_i2c->write(_address << 1, cmd, 0) != 0) {
    return false; // Falló la inicialización
  }
  
  return true;
}

/**************************************************************************/
/*!
    @brief  Lee la temperatura del sensor
    @return la temperatura en grados Celsius
*/
/**************************************************************************/
float Mbed_AM2320::readTemperature() {
  uint8_t data[8];
  
  if (readData(data, 8)) {
    int raw_temperature = (data[4] << 8) | data[5];
    
    // Verificar si la temperatura es negativa
    if (raw_temperature & 0x8000) {
      raw_temperature = -(raw_temperature & 0x7FFF);
    }

    return raw_temperature / 10.0f;
  }
  return NAN; // Retorna NaN si hay un error
}

/**************************************************************************/
/*!
    @brief  Lee la humedad del sensor
    @return la humedad en porcentaje
*/
/**************************************************************************/
float Mbed_AM2320::readHumidity() {
  uint8_t data[8];
  
  if (readData(data, 8)) {
    int raw_humidity = (data[2] << 8) | data[3];
    return raw_humidity / 10.0f;
  }
  return NAN; // Retorna NaN si hay un error
}

/**************************************************************************/
/*!
    @brief  Función auxiliar para leer datos del sensor
    @param buffer Puntero al buffer de datos
    @param length Longitud de los datos a leer
    @return true si la lectura fue exitosa
*/
/**************************************************************************/
bool Mbed_AM2320::readData(uint8_t *buffer, uint8_t length) {
  char cmd[3] = {AM2320_CMD_READREG, AM2320_REG_HUM_H, 4}; // Comando de lectura de 4 registros
  _i2c->write(_address << 1, cmd, 3);                      // Enviar comando de lectura
  ThisThread::sleep_for(2ms);                              // Esperar por la respuesta del sensor
  if (_i2c->read(_address << 1, (char *)buffer, length) != 0) {
    return false; // Falló la lectura
  }
  
  // Omitir chequeo de CRC para simplificar
  return true;
}

#endif
