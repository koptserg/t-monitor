#ifndef SCD4X_H
#define SCD4X_H

//The default I2C address for the SCD4x is 0x62.
#define SCD4x_ADDRESS_READ  (0x62 << 1) | 0x01
#define SCD4x_ADDRESS_WRITE (0x62 << 1) | 0x00

//Available commands

//Basic Commands
#define SCD4x_COMMAND_START_PERIODIC_MEASUREMENT              0x21b1
#define SCD4x_COMMAND_READ_MEASUREMENT                        0xec05 // execution time: 1ms
#define SCD4x_COMMAND_STOP_PERIODIC_MEASUREMENT               0x3f86 // execution time: 500ms

//On-chip output signal compensation
#define SCD4x_COMMAND_SET_TEMPERATURE_OFFSET                  0x241d // execution time: 1ms
#define SCD4x_COMMAND_GET_TEMPERATURE_OFFSET                  0x2318 // execution time: 1ms
#define SCD4x_COMMAND_SET_SENSOR_ALTITUDE                     0x2427 // execution time: 1ms
#define SCD4x_COMMAND_GET_SENSOR_ALTITUDE                     0x2322 // execution time: 1ms
#define SCD4x_COMMAND_SET_AMBIENT_PRESSURE                    0xe000 // execution time: 1ms

//Field calibration
#define SCD4x_COMMAND_PERFORM_FORCED_CALIBRATION              0x362f // execution time: 400ms
#define SCD4x_COMMAND_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED  0x2416 // execution time: 1ms
#define SCD4x_COMMAND_GET_AUTOMATIC_SELF_CALIBRATION_ENABLED  0x2313 // execution time: 1ms

//Low power
#define SCD4x_COMMAND_START_LOW_POWER_PERIODIC_MEASUREMENT    0x21ac
#define SCD4x_COMMAND_GET_DATA_READY_STATUS                   0xe4b8 // execution time: 1ms

//Advanced features
#define SCD4x_COMMAND_PERSIST_SETTINGS                        0x3615 // execution time: 800ms
#define SCD4x_COMMAND_GET_SERIAL_NUMBER                       0x3682 // execution time: 1ms
#define SCD4x_COMMAND_PERFORM_SELF_TEST                       0x3639 // execution time: 10000ms
#define SCD4x_COMMAND_PERFORM_FACTORY_RESET                   0x3632 // execution time: 1200ms
#define SCD4x_COMMAND_REINIT                                  0x3646 // execution time: 20ms

//Low power single shot - SCD41 only
#define SCD4x_COMMAND_MEASURE_SINGLE_SHOT                     0x219d // execution time: 5000ms
#define SCD4x_COMMAND_MEASURE_SINGLE_SHOT_RHT_ONLY            0x2196 // execution time: 50ms


typedef union
{
  int16_t signed16;
  uint16_t unsigned16;
} scd4x_signedUnsigned16_t; // Avoid any ambiguity casting int16_t to uint16_t

typedef union
{
  uint16_t unsigned16;
  uint8_t bytes[2];
} scd4x_unsigned16Bytes_t; // Make it easy to convert 2 x uint8_t to uint16_t

typedef enum
{
  SCD4x_SENSOR_SCD40 = 0,
  SCD4x_SENSOR_SCD41
} scd4x_sensor_type_e;

static scd4x_sensor_type_e sensorType = SCD4x_SENSOR_SCD40;

  // Please see the code examples for an explanation of what measBegin, autoCalibrate and skipStopPeriodicMeasurements do
extern bool SCD4x_begin(bool measBegin, bool autoCalibrate, bool skipStopPeriodicMeasurements);
extern bool SCD4x_startPeriodicMeasurement(void); // Signal update interval is 5 seconds

  // stopPeriodicMeasurement can be called before .begin if required
  // If the sensor has been begun (_i2cPort is not NULL) then _i2cPort is used
  // If the sensor has not been begun (_i2cPort is NULL) then wirePort and address are used (which will default to Wire)
  // Note that the sensor will only respond to other commands after waiting 500 ms after issuing the stop_periodic_measurement command.
extern bool SCD4x_stopPeriodicMeasurement(uint16_t delayMillis);

extern bool SCD4x_readMeasurement(void); // Check for fresh data; store it. Returns true if fresh data is available

extern uint16_t SCD4x_getCO2(void); // Return the CO2 PPM. Automatically request fresh data is the data is 'stale'
extern float SCD4x_getHumidity(void); // Return the RH. Automatically request fresh data is the data is 'stale'
extern float SCD4x_getTemperature(void); // Return the temperature. Automatically request fresh data is the data is 'stale'

  // Define how warm the sensor is compared to ambient, so RH and T are temperature compensated. Has no effect on the CO2 reading
  // Default offset is 4C
extern bool SCD4x_setTemperatureOffset(float offset, uint16_t delayMillis); // Returns true if I2C transfer was OK
extern float SCD4x_getTemperatureOffset(void); // Will return zero if offset is invalid
extern bool SCD4x_getTemperatureOffsetArg(float *offset); // Returns true if offset is valid

  // Define the sensor altitude in metres above sea level, so RH and CO2 are compensated for atmospheric pressure
  // Default altitude is 0m
extern bool SCD4x_setSensorAltitude(uint16_t altitude, uint16_t delayMillis);
extern uint16_t SCD4x_getSensorAltitude(void); // Will return zero if altitude is invalid
extern bool SCD4x_getSensorAltitudeArg(uint16_t *altitude); // Returns true if altitude is valid

  // Define the ambient pressure in Pascals, so RH and CO2 are compensated for atmospheric pressure
  // setAmbientPressure overrides setSensorAltitude
extern bool SCD4x_setAmbientPressure(float pressure, uint16_t delayMillis);

extern float SCD4x_performForcedRecalibration(uint16_t concentration); // Returns the FRC correction value
extern bool SCD4x_performForcedRecalibrationArg(uint16_t concentration, float *correction); // Returns true if FRC is successful

extern bool SCD4x_setAutomaticSelfCalibrationEnabled(bool enabled, uint16_t delayMillis);
extern bool SCD4x_getAutomaticSelfCalibrationEnabled(void);
extern bool SCD4x_getAutomaticSelfCalibrationEnabledArg(uint16_t *enabled);

extern bool SCD4x_startLowPowerPeriodicMeasurement(void); // Start low power measurements - receive data every 30 seconds
extern bool SCD4x_getDataReadyStatus(void); // Returns true if fresh data is available

extern bool SCD4x_persistSettings(uint16_t delayMillis); // Copy sensor settings from RAM to EEPROM
extern bool SCD4x_getSerialNumber(char *serialNumber); // Returns true if serial number is read correctly
extern bool SCD4x_performSelfTest(void); // Takes 10 seconds to complete. Returns true if the test is successful
extern bool SCD4x_performFactoryReset(uint16_t delayMillis); // Reset all settings to the factory values
extern bool SCD4x_reInit(uint16_t delayMillis); // Re-initialize the sensor, load settings from EEPROM

extern bool SCD4x_measureSingleShot(void); // SCD41 only. Request a single measurement. Data will be ready in 5 seconds
extern bool SCD4x_measureSingleShotRHTOnly(void); // SCD41 only. Request RH and T data only. Data will be ready in 50ms

extern bool SCD4x_sendCommandArg(uint16_t command, uint16_t arguments);
extern bool SCD4x_sendCommand(uint16_t command);
extern bool SCD4x_readData(uint8 *buf, uint16 len);

extern bool SCD4x_readRegister(uint16_t registerAddress, uint16_t *response, uint16_t delayMillis);

extern uint8_t SCD4x_computeCRC8(uint8 *data, uint8_t len);

extern int8 scd41_performForcedRecalibration(uint16 targetCo2Concentration, uint16* frcCorrection);

#endif
