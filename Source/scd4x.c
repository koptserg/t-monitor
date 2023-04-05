/*
  SCD40 sensor driver for CC2530 
  Product: https://sensirion.com/products/catalog/SCD40/
  Written by Koptyakov Sergey 
  From source https://github.com/sparkfun/SparkFun_SCD4x_Arduino_Library                                                             
  With the participation of Andrey Lamchenko, aka Berk, LLC EfektaLab, Moscow, Russia
*/

#include "OSAL.h"
#include "scd4x.h"
#include "hal_i2c.h"

//Global main datums
float co2 = 0;
float temperature = 0;
float humidity = 0;

//These track the staleness of the current data
//This allows us to avoid calling readMeasurement() every time individual datums are requested
bool co2HasBeenReported = true;
bool humidityHasBeenReported = true;
bool temperatureHasBeenReported = true;

//Keep track of whether periodic measurements are in progress
bool periodicMeasurementsAreRunning = false;
  
bool measBegin = true;
bool autoCalibrate = true; 
bool skipStopPeriodicMeasurements = false;
  
//Convert serial number digit to ASCII
char convertHexToASCII(uint8_t digit);
  
static void delay(uint32 period);
static void delayUs(uint16 microSecs);

//Initialize the Serial port
bool SCD4x_begin(bool measBegin, bool autoCalibrate, bool skipStopPeriodicMeasurements) 
{
  bool success = 1;

  //If periodic measurements are already running, getSerialNumber will fail...
  //To be safe, let's stop period measurements before we do anything else
  //The user can override this by setting skipStopPeriodicMeasurements to true
  if (skipStopPeriodicMeasurements == 0)
    success &= SCD4x_stopPeriodicMeasurement(500); // Delays for 500ms...

  char serialNumber[13]; // Serial number is 12 digits plus trailing NULL
  success &= SCD4x_getSerialNumber(serialNumber); // Read the serial number. Return false if the CRC check fails.
  if (success == false)
    return (false);

  if (autoCalibrate == 1) // Must be done before periodic measurements are started
  {
    success &= SCD4x_setAutomaticSelfCalibrationEnabled(true, 1); // 1 ms datasheet
    success &= (SCD4x_getAutomaticSelfCalibrationEnabled() == true);
  }
  else
  {
    success &= SCD4x_setAutomaticSelfCalibrationEnabled(false, 1); // 1 ms datasheet
    success &= (SCD4x_getAutomaticSelfCalibrationEnabled() == false);
  }

  if (measBegin == 1)
  {
    success &= SCD4x_startPeriodicMeasurement();
  }

  return (success);
}

//Start periodic measurements. See 3.5.1
//signal update interval is 5 seconds.
bool SCD4x_startPeriodicMeasurement(void)
{
  if (periodicMeasurementsAreRunning)
  {
    return (true); //Maybe this should be false?
  }

  bool success = SCD4x_sendCommand(SCD4x_COMMAND_START_PERIODIC_MEASUREMENT);
  
  if (success)
    periodicMeasurementsAreRunning = true;
  return (success);
}

//Stop periodic measurements. See 3.5.3
//Stop periodic measurement to change the sensor configuration or to save power.
//Note that the sensor will only respond to other commands after waiting 500 ms after issuing
//the stop_periodic_measurement command.
bool SCD4x_stopPeriodicMeasurement(uint16_t delayMillis)
{
  bool success = SCD4x_sendCommand(SCD4x_COMMAND_STOP_PERIODIC_MEASUREMENT);

  if (success == 1)
  {
    periodicMeasurementsAreRunning = 0;
    if (delayMillis > 0)
      delay(delayMillis);
    return(true);
  }

  return (false);
}

static void delayUs(uint16 microSecs) {
  while(microSecs--) {
    /* 32 NOPs == 1 usecs */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
}

static void delay(uint32 period) { delayUs(period * 1000); }

//Get 9 bytes from SCD4x. See 3.5.2
//Updates global variables with floats
//Returns true if data is read successfully
//Read sensor output. The measurement data can only be read out once per signal update interval as the
//buffer is emptied upon read-out. If no data is available in the buffer, the sensor returns a NACK.
//To avoid a NACK response, the get_data_ready_status can be issued to check data status
//(see chapter 3.8.2 for further details).
bool SCD4x_readMeasurement(void)
{
  //Verify we have data from the sensor
  if (SCD4x_getDataReadyStatus() == false)
    return (false);

  scd4x_unsigned16Bytes_t tempCO2;
  tempCO2.unsigned16 = 0;
  scd4x_unsigned16Bytes_t  tempHumidity;
  tempHumidity.unsigned16 = 0;
  scd4x_unsigned16Bytes_t  tempTemperature;
  tempTemperature.unsigned16 = 0;
  
  bool success = SCD4x_sendCommand(SCD4x_COMMAND_READ_MEASUREMENT);
  if(success == 0){
    return false;
  }

  delay(1); //Datasheet specifies this
  
  uint8 data[9] = {0};
  success = SCD4x_readData(data, 9);
  if(success == 0){
    return false;
  }
  
  bool error = false;

    uint8 bytesToCrc[2];
    for (uint8 x = 0; x < 9; x++)
    {
      uint8 incoming = data[x];
      uint8_t foundCrc;
      switch (x)
      {
      case 0:
      case 1:
        tempCO2.bytes[x == 0 ? 1 : 0] = incoming; // Store the two CO2 bytes in little-endian format
        bytesToCrc[x] = incoming; // Calculate the CRC on the two CO2 bytes in the order they arrive
        break;
      case 3:
      case 4:
        tempTemperature.bytes[x == 3 ? 1 : 0] = incoming; // Store the two T bytes in little-endian format
        bytesToCrc[x % 3] = incoming; // Calculate the CRC on the two T bytes in the order they arrive
        break;
      case 6:
      case 7:
        tempHumidity.bytes[x == 6 ? 1 : 0] = incoming; // Store the two RH bytes in little-endian format
        bytesToCrc[x % 3] = incoming; // Calculate the CRC on the two RH bytes in the order they arrive
        break;
      default: // x == 2, 5, 8
        //Validate CRC
        foundCrc = SCD4x_computeCRC8(bytesToCrc, 2); // Calculate what the CRC should be for these two bytes
        if (foundCrc != incoming) // Does this match the CRC byte from the sensor?
        {
          error = true;
        }
        break;
      }
    }

  if (error)
  {
    return (false);
  }
  //Now copy the int16s into their associated floats
  co2 = (float)tempCO2.unsigned16;
  temperature = -45 + (((float)tempTemperature.unsigned16) * 175 / 65536);
  humidity = ((float)tempHumidity.unsigned16) * 100 / 65536;

  //Mark our global variables as fresh
  co2HasBeenReported = false;
  humidityHasBeenReported = false;
  temperatureHasBeenReported = false;

  return (true); //Success! New data available in globals.
}

//Returns true when data is available. See 3.8.2
bool SCD4x_getDataReadyStatus(void)
{
  uint16_t response;
  bool success = SCD4x_readRegister(SCD4x_COMMAND_GET_DATA_READY_STATUS, &response, 1);

  if (success == false)
    return (false);

  //If the least significant 11 bits of word[0] are 0 ? data not ready
  //else ? data ready for read-out
  if ((response & 0x07ff) == 0x0000)
    return (false);
  return (true);
}

//Returns the latest available CO2 level
//If the current level has already been reported, trigger a new read
uint16_t SCD4x_getCO2(void)
{
  if (co2HasBeenReported == true) //Trigger a new read
    SCD4x_readMeasurement();            //Pull in new co2, humidity, and temp into global vars

  co2HasBeenReported = true;

  return (uint16_t)co2; //Cut off decimal as co2 is 0 to 10,000
}

//Returns the latest available humidity
//If the current level has already been reported, trigger a new read
float SCD4x_getHumidity(void)
{
  if (humidityHasBeenReported == true) //Trigger a new read
    SCD4x_readMeasurement();                 //Pull in new co2, humidity, and temp into global vars

  humidityHasBeenReported = true;

  return humidity;
}

//Returns the latest available temperature
//If the current level has already been reported, trigger a new read
float SCD4x_getTemperature(void)
{
  if (temperatureHasBeenReported == true) //Trigger a new read
    SCD4x_readMeasurement();                    //Pull in new co2, humidity, and temp into global vars

  temperatureHasBeenReported = true;

  return temperature;
}

bool SCD4x_readData(uint8 *buf, uint16 len)
{ 
  bool error;
  error = HalI2CReceive(SCD4x_ADDRESS_READ, buf, len);

  return !error;
}

//Get 9 bytes from SCD4x. Convert 48-bit serial number to ASCII chars. See 3.9.2
//Returns true if serial number is read successfully
//Reading out the serial number can be used to identify the chip and to verify the presence of the sensor.
bool SCD4x_getSerialNumber(char *serialNumber)
{
  if (periodicMeasurementsAreRunning)
  {
    return (false);
  }

  bool success = SCD4x_sendCommand(SCD4x_COMMAND_GET_SERIAL_NUMBER);  
  if (success == 0)
  {
    return (false);
  }
  delay(1); //Datasheet specifies this
  
  uint8 buffer[9] = {0};
  success = SCD4x_readData(buffer, 9);
  if (success == 0)
  {
    return (false);  
  }
  
  bool error = false;

    uint8 bytesToCrc[2];
    uint8 foundCrc;
    int digit = 0;
    for (uint8 x = 0; x < 9; x++)
    {
      uint8 incoming = buffer[x];

      switch (x)
      {
      case 0: // The serial number arrives as: two bytes, CRC, two bytes, CRC, two bytes, CRC
      case 1:
      case 3:
      case 4:
      case 6:
      case 7:
        serialNumber[digit++] = convertHexToASCII(incoming >> 4); // Convert each nibble to ASCII
        serialNumber[digit++] = convertHexToASCII(incoming & 0x0F);
        bytesToCrc[x % 3] = incoming;
        break;
      default: // x == 2, 5, 8
        //Validate CRC
        foundCrc = SCD4x_computeCRC8(bytesToCrc, 2); // Calculate what the CRC should be for these two bytes
        if (foundCrc != incoming) // Does this match the CRC byte from the sensor?
        {
          error = true;
        }
        break;
      }
      serialNumber[digit] = 0; // NULL-terminate the string
    }

  if (error)
  {
    return (false);
  }

  return (true); //Success!
}

//Given an array and a number of bytes, this calculate CRC8 for those bytes
//CRC is only calc'd on the data portion (two bytes) of the four bytes being sent
//From: http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
//Tested with: http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
//x^8+x^5+x^4+1 = 0x31
uint8_t SCD4x_computeCRC8(uint8 *data, uint8_t len)
{
  uint8_t crc = 0xFF; //Init with 0xFF
  for (uint8_t x = 0; x < len; x++)
  {
    crc ^= data[x]; // XOR-in the next input byte
    for (uint8_t i = 0; i < 8; i++)
    {
      if ((crc & 0x80) != 0)
        crc = (uint8_t)((crc << 1) ^ 0x31);
      else
        crc <<= 1;
    }
  }
  return crc; //No output reflection
}

//PRIVATE: Convert serial number digit to ASCII
static char convertHexToASCII(uint8_t digit)
{
  if (digit <= 9)
    return ((char)(digit + 0x30));
  else
    return ((char)(digit + 0x41 - 10)); // Use upper case for A-F
}

//Enable/disable automatic self calibration. See 3.7.2
//Set the current state (enabled / disabled) of the automatic self-calibration. By default, ASC is enabled.
//To save the setting to the EEPROM, the persist_setting (see chapter 3.9.1) command must be issued.
bool SCD4x_setAutomaticSelfCalibrationEnabled(bool enabled, uint16_t delayMillis)
{
  if (periodicMeasurementsAreRunning)
  {
    return (false);
  }

  uint16_t enabledWord = enabled == true ? 0x0001 : 0x0000;
  bool success = SCD4x_sendCommandArg(SCD4x_COMMAND_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED, enabledWord);
  if (delayMillis > 0)
    delay(delayMillis);
  return (success);
}

//Check if automatic self calibration is enabled. See 3.7.3
bool SCD4x_getAutomaticSelfCalibrationEnabled(void)
{
  if (periodicMeasurementsAreRunning)
  {
    return (false);
  }

  uint16_t enabled;
  bool success = SCD4x_getAutomaticSelfCalibrationEnabledArg(&enabled);
  if (success == false)
  {
    return (false);
  }
  return (enabled == 0x0001);
}

//Check if automatic self calibration is enabled. See 3.7.3
bool SCD4x_getAutomaticSelfCalibrationEnabledArg(uint16_t *enabled)
{
  if (periodicMeasurementsAreRunning)
  {
    return (false);
  }

  return (SCD4x_readRegister(SCD4x_COMMAND_GET_AUTOMATIC_SELF_CALIBRATION_ENABLED, enabled, 1));
}

//Gets two bytes from SCD4x plus CRC.
//Returns true if endTransmission returns zero _and_ the CRC check is valid
bool SCD4x_readRegister(uint16_t registerAddress, uint16_t *response, uint16_t delayMillis)
{ 
  bool success = SCD4x_sendCommand(registerAddress);
  if (success != 1)
    return (false);

  delay(delayMillis);
  
  uint8_t data[3];
  success = SCD4x_readData(data, 3);// Request data and CRC
  if (success != 1)
    return (false);
  
    uint8_t crc = data[2];
    *response = (uint16_t)data[0] << 8 | data[1];
    uint8_t expectedCRC = SCD4x_computeCRC8(data, 2);
    if (crc == expectedCRC) // Return true if CRC check is OK
      return (true);
    
  return (false);
}

//Perform self test. Takes 10 seconds to complete. See 3.9.3
//The perform_self_test feature can be used as an end-of-line test to check sensor functionality
//and the customer power supply to the sensor.
bool SCD4x_performSelfTest(void)
{
  if (periodicMeasurementsAreRunning)
  {
    return (false);
  }

  uint16_t response;

  bool success = SCD4x_readRegister(SCD4x_COMMAND_PERFORM_SELF_TEST, &response, 10000);

  return (success && (response == 0x0000)); // word[0] = 0 ? no malfunction detected
}

//Peform factory reset. See 3.9.4
//The perform_factory_reset command resets all configuration settings stored in the EEPROM
//and erases the FRC and ASC algorithm history.
bool SCD4x_performFactoryReset(uint16_t delayMillis)
{
  if (periodicMeasurementsAreRunning)
  {
    return (false);
  }

  bool success = SCD4x_sendCommand(SCD4x_COMMAND_PERFORM_FACTORY_RESET);
  if (delayMillis > 0)
    delay(delayMillis);
  return (success);
}

//Persist settings: copy settings (e.g. temperature offset) from RAM to EEPROM. See 3.9.1
//Configuration settings such as the temperature offset, sensor altitude and the ASC enabled/disabled parameter
//are by default stored in the volatile memory (RAM) only and will be lost after a power-cycle. The persist_settings
//command stores the current configuration in the EEPROM of the SCD4x, making them persistent across power-cycling.
//To avoid unnecessary wear of the EEPROM, the persist_settings command should only be sent when persistence is required
//and if actual changes to the configuration have been made. The EEPROM is guaranteed to endure at least 2000 write
//cycles before failure.
bool SCD4x_persistSettings(uint16_t delayMillis)
{
  if (periodicMeasurementsAreRunning)
  {
    return (false);
  }

  bool success = SCD4x_sendCommand(SCD4x_COMMAND_PERSIST_SETTINGS);
  if (delayMillis > 0)
    delay(delayMillis);
  return (success);
}

//Reinit. See 3.9.5
//The reinit command reinitializes the sensor by reloading user settings from EEPROM.
//Before sending the reinit command, the stop measurement command must be issued.
//If the reinit command does not trigger the desired re-initialization,
//a power-cycle should be applied to the SCD4x.
bool SCD4x_reInit(uint16_t delayMillis)
{
  if (periodicMeasurementsAreRunning)
  {
    return (false);
  }

  bool success = SCD4x_sendCommand(SCD4x_COMMAND_REINIT);
  if (delayMillis > 0)
    delay(delayMillis);
  return (success);
}

//Sends just a command, no arguments, no CRC
bool SCD4x_sendCommand(uint16 command)
{ 
  command = ((command << 8) & 0xff00) | ((command >> 8) & 0x00ff);
  bool error;
  error = HalI2CSend(SCD4x_ADDRESS_WRITE, (uint8*)&command, 2);

  return !error;
}

//Sends a command along with arguments and CRC
bool SCD4x_sendCommandArg(uint16_t command, uint16_t arguments)
{
  uint8 data[2];
  data[0] = (arguments & 0xFF00) >> 8;
  data[1] = (arguments & 0x00FF) >> 0;
  uint8 crc = SCD4x_computeCRC8(data, 2);
  uint8 command_arg[5];
  command_arg[0] = (command & 0xFF00) >> 8;
  command_arg[1] = (command & 0x00FF) >> 0;
  command_arg[2] = data[0];
  command_arg[3] = data[1];
  command_arg[4] = crc;
  int8 error = HalI2CSend(SCD4x_ADDRESS_WRITE, command_arg, 5);
  if (error != 0)
    return (false);
  
  return (true);
}

//Set the temperature offset (C). See 3.6.1
//Max command duration: 1ms
//The user can set delayMillis to zero f they want the function to return immediately.
//The temperature offset has no influence on the SCD4x CO2 accuracy.
//Setting the temperature offset of the SCD4x inside the customer device correctly allows the user
//to leverage the RH and T output signal.
bool SCD4x_setTemperatureOffset(float offset, uint16_t delayMillis)
{
 
  if (periodicMeasurementsAreRunning)
  {
    return (false);
  }

  if (offset < 0)
  {
    return (false);
  }
  if (offset >= 175)
  {
    return (false);
  }
  uint16_t offsetWord = (uint16_t)(offset * 65536 / 175); // Toffset [°C] * 2^16 / 175
  bool success = SCD4x_sendCommandArg(SCD4x_COMMAND_SET_TEMPERATURE_OFFSET, offsetWord);

  if (delayMillis > 0)
    delay(delayMillis);
  return (success);  
}

//Get the temperature offset. See 3.6.2
float SCD4x_getTemperatureOffset(void)
{
 if (periodicMeasurementsAreRunning)
  {
    return (0.0);
  }

  float offset;

  SCD4x_getTemperatureOffsetArg(&offset);

  return (offset);
}

//Get the temperature offset. See 3.6.2
// Per default, the temperature offset is set to 4° C
bool SCD4x_getTemperatureOffsetArg(float *offset)
{
  if (periodicMeasurementsAreRunning)
  {
    return (false);
  }

  uint16_t offsetWord = 0; // offset will be zero if readRegister fails
  bool success = SCD4x_readRegister(SCD4x_COMMAND_GET_TEMPERATURE_OFFSET, &offsetWord, 1);
  *offset = ((float)offsetWord) * 175.0 / 65535.0;
  return (success);
}

//Set the sensor altitude (metres above sea level). See 3.6.3
//Max command duration: 1ms
//The user can set delayMillis to zero if they want the function to return immediately.
//Reading and writing of the sensor altitude must be done while the SCD4x is in idle mode.
//Typically, the sensor altitude is set once after device installation. To save the setting to the EEPROM,
//the persist setting (see chapter 3.9.1) command must be issued.
//Per default, the sensor altitude is set to 0 meter above sea-level.
bool SCD4x_setSensorAltitude(uint16_t altitude, uint16_t delayMillis)
{
  if (periodicMeasurementsAreRunning)
  {
    return (false);
  }

  bool success = SCD4x_sendCommandArg(SCD4x_COMMAND_SET_SENSOR_ALTITUDE, altitude);
  if (delayMillis > 0)
    delay(delayMillis);
  return (success);
}

//Get the sensor altitude. See 3.6.4
uint16_t SCD4x_getSensorAltitude(void)
{
  if (periodicMeasurementsAreRunning)
  {
    return (0);
  }
  uint16_t altitude = 0;
  SCD4x_getSensorAltitudeArg(&altitude);
  return (altitude);
}

//Get the sensor altitude. See 3.6.4
bool SCD4x_getSensorAltitudeArg(uint16_t *altitude)
{
  if (periodicMeasurementsAreRunning)
  {
    return (false);
  }
  return (SCD4x_readRegister(SCD4x_COMMAND_GET_SENSOR_ALTITUDE, altitude, 1));
}

//Set the ambient pressure (Pa). See 3.6.5
//Max command duration: 1ms
//The user can set delayMillis to zero if they want the function to return immediately.
//The set_ambient_pressure command can be sent during periodic measurements to enable continuous pressure compensation.
//setAmbientPressure overrides setSensorAltitude
bool SCD4x_setAmbientPressure(float pressure, uint16_t delayMillis)
{
  if (pressure < 0)
  {
    return (false);
  }
  if (pressure > 6553500)
  {
    return (false);
  }
  uint16_t pressureWord = (uint16_t)(pressure / 100);
  bool success = SCD4x_sendCommandArg(SCD4x_COMMAND_SET_AMBIENT_PRESSURE, pressureWord);
  if (delayMillis > 0)
    delay(delayMillis);
  return (success);
}

//Perform forced recalibration. See 3.7.1
float SCD4x_performForcedRecalibration(uint16_t concentration)
{
  if (periodicMeasurementsAreRunning)
  {
    return (0.0);
  }
  float correction = 0.0;
  SCD4x_performForcedRecalibrationArg(concentration, &correction);
  return (correction);
}

//Perform forced recalibration. See 3.7.1
//To successfully conduct an accurate forced recalibration, the following steps need to be carried out:
//1. Operate the SCD4x in the operation mode later used in normal sensor operation (periodic measurement,
//   low power periodic measurement or single shot) for > 3 minutes in an environment with homogenous and
//   constant CO2 concentration.
//2. Issue stop_periodic_measurement. Wait 500 ms for the stop command to complete.
//3. Subsequently issue the perform_forced_recalibration command and optionally read out the FRC correction
//   (i.e. the magnitude of the correction) after waiting for 400 ms for the command to complete.
//A return value of 0xffff indicates that the forced recalibration has failed.
//Note that the sensor will fail to perform a forced recalibration if it was not operated before sending the command.

bool SCD4x_performForcedRecalibrationArg(uint16_t concentration, float *correction)
{
  if (periodicMeasurementsAreRunning)
  {
    return (false);
  }
  uint16_t correctionWord;
  bool success = true;
  success = SCD4x_sendCommandArg(SCD4x_COMMAND_PERFORM_FORCED_CALIBRATION, concentration);
  if (success == false)
    return (false);

  delay(400); //Datasheet specifies this
 
  uint8_t data[3];
  success = SCD4x_readData(data, 3);// Request data and CRC
  if (success != 1)
    return (false);
   
  bool error = false;

    uint8 bytesToCrc[2];
    bytesToCrc[0] = data[0];
    correctionWord = ((uint16_t)bytesToCrc[0]) << 8;
    bytesToCrc[1] = data[1];
    correctionWord |= (uint16_t)bytesToCrc[1];
    
    uint8 incomingCrc = data[2];
    uint8_t foundCrc = SCD4x_computeCRC8(bytesToCrc, 2);
    
//  *correction = ((float)correctionWord) - 32768; // FRC correction [ppm CO2] = word[0] – 0x8000
  *correction = data[0] << 8 | data[1];
  if (correctionWord == 0xffff) //A return value of 0xffff indicates that the forced recalibration has failed
    return (false);

    if (foundCrc != incomingCrc)
    {
      error = true;
    }

  if (error)
  {
    return (false);
  }
  
  return (true);
}

int8 scd41_performForcedRecalibration(uint16 targetCo2Concentration, uint16* frcCorrection) {
  int8  error;
  uint16 command = 0x2F36;
  uint8 bufferCRCtemp[2];
  bufferCRCtemp[0] = (uint8)(( targetCo2Concentration & 0xFF00) >> 8);
  bufferCRCtemp[1] = (uint8)(( targetCo2Concentration & 0x00FF) >> 0);
  uint8 crc = SCD4x_computeCRC8(bufferCRCtemp, 2);
  uint8 bufferTargetCo2Concentration[5];
  bufferTargetCo2Concentration[1] = (uint8)((command & 0xFF00) >> 8);
  bufferTargetCo2Concentration[0] = (uint8)((command & 0x00FF) >> 0);
  bufferTargetCo2Concentration[2] = bufferCRCtemp[0];
  bufferTargetCo2Concentration[3] = bufferCRCtemp[1];
  bufferTargetCo2Concentration[4] = crc;
  error = HalI2CSend(SCD4x_ADDRESS_WRITE, bufferTargetCo2Concentration, 5); 
  if (error) {
        return error;
    }
  
  delay(400);
  
  uint8 bufferFRC[2] = {0};

  error = HalI2CReceive(SCD4x_ADDRESS_READ, bufferFRC, 2); 
  uint16 frcCorrectionTemp = bufferFRC[0] << 8 | bufferFRC[1];
  *frcCorrection = frcCorrectionTemp;
  
  return error;
}

