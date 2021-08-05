#include "Arduino.h"
#include "AS5600.h"

AS5600::AS5600() {
  // NOTE: will defualt startup to whatever address is stored in the address register 
  Wire.begin();
}

/*
 * Function: getPosition
 * ----------------------------
 *   returns: the unscaled and unmodified angle from the RAW ANGLE register.
 */
uint16_t AS5600::getPosition() {
  return getRawAngle();
}

/*
 * Function: getAngle
 * ----------------------------
 *   returns: the scaled output value available in the ANGLE register.
 */
uint16_t AS5600::getAngle() {
  return _getRegisters2(_ANGLEAddressMSB, _ANGLEAddressLSB);
}

/*
 * Function: getRawAngle
 * ----------------------------
 *   returns: the unscaled and unmodified angle from the RAW ANGLE register.
 */
uint16_t AS5600::getRawAngle() {
  return _getRegisters2(_RAWANGLEAddressMSB, _RAWANGLEAddressLSB);
}

/*
 * Function: getScaledAngle
 * ----------------------------
 *   returns: the raw angle as a value between 0 and 360 degrees.
 */
float AS5600::getScaledAngle() {
  int ang_hi = _getRegister(_RAWANGLEAddressMSB);
  int ang_lo = _getRegister(_RAWANGLEAddressLSB);
  return ang_hi * 22.5 + ang_lo * 0.087890625;
}

/*
 * Function: getStatus
 * ----------------------------
 *   returns: The the value in the STATUS register. The STATUS register provides bits that indicate the current state of the AS5600.
 *
 *   register format: X X MD ML MH X X X
 *            
 *   MH: AGC minimum gain overflow, magnet too strong
 *   ML: AGC maximum gain overflow, magnet too weak
 *   MD: Magnet was detected
 *
 */
uint8_t AS5600::getStatus() {
  return _getRegister(_STATUSAddress) & 0b00111000;
}

/*
 * Function: isMagnetTooStrong
 * ----------------------------
 *   returns: true if magnet is too close to AS5600.
 */
bool AS5600::isMagnetTooStrong() {
  uint8_t _b=0;
  _b = getStatus();
  if (_b & (1<<5)) { return true; }
  return false;
}

/*
 * Function: isMagnetDetected
 * ----------------------------
 *   returns: true if magnet is too far to AS5600.
 */
bool AS5600::isMagnetTooWeak() {
  uint8_t _b=0;
  _b = getStatus();
  if (_b & (1<<4)) { return true; }
  return false;
}

/*
 * Function: isMagnetDetected
 * ----------------------------
 *   returns: true if magnet is detected by AS5600.
 */
bool AS5600::isMagnetDetected() {
  uint8_t _b=0;
  _b = getStatus();
  if (_b & (1<<3)) { return true; }
  return false;
}

/*
 * Function: getGain
 * ----------------------------
 *   returns: the value contained in the Automatic Gain Control (AGC) register.
 *    
 *   In 5V operation, the AGC range is 0-255 
 *   In 3.3V operation, the AGC range is 0-128
 */
uint8_t AS5600::getGain() {
  return _getRegister(_AGCAddress);
}

/*
 * Function: getMagnet
 * ----------------------------
 *   returns: getGain().
 */
uint8_t AS5600::getMagnet() {
  return getGain();
}

/*
 * Function: getMagnitude
 * ----------------------------
 *   returns: the value contained in the MAGNITUDE REGISTER. The MAGNITUDE register indicates the magnitude value of the
 *            internal Coordinate Rotation Digital Computer (CORDIC). 
 */
uint16_t AS5600::getMagnitude() {
  return _getRegisters2(_MAGNITUDEAddressMSB, _MAGNITUDEAddressLSB);
}

/*
 * Function: setPowerMode
 * ----------------------------
 *   powerMode: the desired power mode. Valid input values are 0, 1, 2, and 3, corresponding to Normal Mode, Low Power Mode 1, Low Power Mode 2, Low Power Mode 3.
 * 
 *   returns: boolean indicating is value was set. 
 */
bool AS5600::setPowerMode(uint8_t powerMode) {
  if (powerMode != POWER_MODE_NORM && powerMode != POWER_MODE_LPM1 && powerMode != POWER_MODE_LPM2 && powerMode != POWER_MODE_LPM3) {
    return false;
  } 

  uint8_t currentCONFLSB = _getRegister(_CONFAddressLSB);
  uint8_t writeCONFLSB = currentCONFLSB & 0b11111100 | powerMode;
  _writeRegister(_CONFAddressLSB, writeCONFLSB);

  currentCONFLSB = _getRegister(_CONFAddressLSB);

  return currentCONFLSB == writeCONFLSB;
  
}

/*
 * Function: setHysteresis
 * ----------------------------
 *   hysteresis: the desired hysteresis. Valid values are 0, 1, 2, and 3, corresponding to OFF, 1 LSB, 2 LSBs, 3 LSBs.
 * 
 *   returns: boolean indicating is value was set. 
 */
bool AS5600::setHysteresis(uint8_t hysteresis) {
  if (hysteresis != HYSTERESIS_OFF && hysteresis != HYSTERESIS_1LSB && hysteresis != HYSTERESIS_2LSB && hysteresis != HYSTERESIS_3LSB) {
    return false;
  } 

  uint8_t currentCONFLSB = _getRegister(_CONFAddressLSB);
  uint8_t writeCONFLSB = currentCONFLSB & 0b11110011 | (hysteresis << 2);
  _writeRegister(_CONFAddressLSB, writeCONFLSB);

  currentCONFLSB = _getRegister(_CONFAddressLSB);

  return currentCONFLSB == writeCONFLSB;
}

/*
 * Function: setOutputStage
 * ----------------------------
 *   outputStage: the desired outputStage. Valid values are 0, 1 and 2 
 * 
 *   returns: boolean indicating is value was set. 
 */
bool AS5600::setOutputStage(uint8_t outputStage) {
  if (outputStage != OUTPUT_STAGE_ANALOG_FULL && outputStage != OUTPUT_STAGE_ANALOG_REDUCED && outputStage != OUTPUT_STAGE_DIGITAL_PWM) {
    return false;
  } 

  uint8_t currentCONFLSB = _getRegister(_CONFAddressLSB);
  uint8_t writeCONFLSB = currentCONFLSB & 0b11001111 | (outputStage << 4);
  _writeRegister(_CONFAddressLSB, writeCONFLSB);

  currentCONFLSB = _getRegister(_CONFAddressLSB);

  return currentCONFLSB == writeCONFLSB;
}

/*
 * Function: setPWMFrequency
 * ----------------------------
 *   frequency: the desired PWM frequency for PWM output. Valid values are 115 Hz, 230 Hz, 460 Hz, 920 Hz.
 * 
 *   returns: boolean indicating is value was set. 
 */
 
bool AS5600::setPWMFrequency(uint8_t frequency) {

  uint8_t currentCONFLSB = _getRegister(_CONFAddressLSB);
  uint8_t writeCONFLSB = currentCONFLSB & 0b00111111 | (frequency << 6);
  _writeRegister(_CONFAddressLSB, writeCONFLSB);

  currentCONFLSB = _getRegister(_CONFAddressLSB);

  return currentCONFLSB == writeCONFLSB;
}

/*
 * Function: setSlowFilter
 * ----------------------------
 *   outputStage: the desired outputStage. Valid values are 0, 1, 2.
 * 
 *   returns: boolean indicating is value was set. 
 */
bool AS5600::setSlowFilter(uint8_t slowFilter) {
  if (slowFilter != SLOW_FILTER_16X && slowFilter != SLOW_FILTER_8X && slowFilter != SLOW_FILTER_4X && slowFilter != SLOW_FILTER_2X) {
    return false;
  } 

  uint8_t currentCONFLSB = _getRegister(_CONFAddressLSB);
  uint8_t writeCONFLSB = currentCONFLSB & 0b11111100 | slowFilter;
  _writeRegister(_CONFAddressLSB, writeCONFLSB);

  currentCONFLSB = _getRegister(_CONFAddressMSB);

  return currentCONFLSB == writeCONFLSB;
}

uint8_t AS5600::getCONF() {

  return _getRegister(_CONFAddressLSB);

}

/*
 * Function: setFastFilterThreshold
 * ----------------------------
 *   fastFilterThreshold: the desired fastFilterThreshold. Valid values are 0, 1, 2, 3, 4, 5, 6, 7
 * 
 *   returns: boolean indicating is value was set. 
 */
bool AS5600::setFastFilterThreshold(uint8_t fastFilterThreshold) {
  if (fastFilterThreshold != FAST_FILTER_THRESHOLD_SLOW && 
      fastFilterThreshold != FAST_FILTER_THRESHOLD_6LSB && 
      fastFilterThreshold != FAST_FILTER_THRESHOLD_7LSB && 
      fastFilterThreshold != FAST_FILTER_THRESHOLD_9LSB && 
      fastFilterThreshold != FAST_FILTER_THRESHOLD_18LSB && 
      fastFilterThreshold != FAST_FILTER_THRESHOLD_21LSB && 
      fastFilterThreshold != FAST_FILTER_THRESHOLD_24LSB && 
      fastFilterThreshold != FAST_FILTER_THRESHOLD_10LSB) {
    return false;
  } 

  uint8_t currentCONFLSB = _getRegister(_CONFAddressLSB);
  uint8_t writeCONFLSB = currentCONFLSB & 0b11100011 | (fastFilterThreshold << 3);
  _writeRegister(_CONFAddressLSB, writeCONFLSB);

  currentCONFLSB = _getRegister(_CONFAddressMSB);

  return currentCONFLSB == writeCONFLSB;
}

/*
 * Function: _getRegister
 * ----------------------------
 *   register1: register address
 *
 *   returns: the value within a register.
 */
uint8_t AS5600::_getRegister(byte register1) {  
  uint8_t _b=0;

  Wire.beginTransmission(_AS5600Address);
  Wire.write(register1);
  Wire.endTransmission();

  Wire.requestFrom(_AS5600Address, 1);

  while (Wire.available() == 0) { }
  _b = Wire.read();

  return _b;
}

/*
 * Function: _getRegisters2
 * ----------------------------
 *   registerMSB: register address of Most Significant Byte (MMSB)
 *   registerLSB: register address of Least Significant Byte (MMSB)
 *
 *   returns: the value of the 16 bit number stored in registerMSB and registerLSB.
 */
uint16_t AS5600::_getRegisters2(byte registerMSB, byte registerLSB) {

  uint16_t _hi=0, _lo=0;

  _hi = _getRegister(registerMSB);
  _lo = _getRegister(registerLSB);
  return (_hi<<8) | (_lo);
}


/*
 * Function: _writeRegister
 * ----------------------------
 *   registerAddress: register address to write to
 *   value: value to write to register at registerAddress
 *
 *   returns: void
 */
void AS5600::_writeRegister(byte registerAddress, byte value) {
  Wire.beginTransmission(_AS5600Address);
	Wire.write((uint8_t)registerAddress); //module function register address
	Wire.write((uint8_t)value); //data bytes
	Wire.endTransmission();
}



// Added by Erez Krimsky, 7/21/21



#ifdef AS5600L
/*
*  
* --------------------------------------
*
*
*/
bool AS5600::changeI2CAddress(byte newAddress) {
  // If MSB ever written to 0 cannot be changed to 1 
  // to prevent this, lets ONLY allow addresses with MSB = 1 
  // PERMANENT CHANGE 

  // NOTE: should limit this to 0x40, 0x41, 0x42...0x44 to 
  // to avoid confusion -- each of these sets the MSB and only one other 
  // bit to 1 
  
  // 0x40, 0x41, 0x42, 0x44, 0x48, 0x50, 0x60
  if ((newAddress == 0x40) || (newAddress == 0x41) || (newAddress == 0x42) ||
      (newAddress == 0x44) || (newAddress == 0x48) || (newAddress == 0x50) ||
      (newAddress == 0x60))
  {

    // address should be 7 bit anyway for I2C 

    // From the datasheet, we are not supposed to modify unused bits in the register map
    // this would include bit 0 for the I2C addres 
    // so, do read modify write 

    byte init_byte = _getRegister(_I2CADDR);

    // & 0x01 --> clears all the bits except in position 0 
    // then or it with the address left shifted 
    byte address_outbyte = (newAddress << 1) | (0x01 & init_byte);

    // Step 1 - Write new address to I2CADDR 
    // Register map on page 19 -- write address to bit positions 1-7 (bit 0 unused)
    // Left shift by 1 to do this 
    _writeRegister(_I2CADDR, address_outbyte); // datasheet page 28 

    // Page 27 footnote 3 -- except for MSB, ANY bit written to 1 cannot be 
    // written back to 0 

    // From datasheet on burning angle, seems may need to wait at least 1 ms 
    delay(2);


    // Step 2 - Perform Burn Settings to make permanent
    // page 24 "To perform a BURN_SETTING command, write the value 0x40 into register 0xFF."
    _writeRegister(_BURNAddress, 0x40); 

    // NOTE: burn also burns in any other setting bit which has been changed to 1 

    // change locally because otherwise comms will drop after the burn 
    setI2CAddress(newAddress);  // NEED TO RESTART THE DEVICE ANYWAY!!!!
    // NOTE: presumably the address doesnt change until we do the burn, otherwise the the second write will fail 
    return true;
  } else {
    return false; 
  }



}


void AS5600::setI2CAddress(byte newAddress) {
  // changes the address WE WILL BE CALLING on the bus 
  // does NOT perform a burn operation and actually change the address of the I2C 
  _AS5600Address = newAddress;
}

#endif