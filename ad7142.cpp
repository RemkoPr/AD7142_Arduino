/** Based on Analog Devices AD7142 datasheet rev. B https://www.analog.com/media/en/technical-documentation/data-sheets/AD7142.pdf
* Started 28/01/2022 by Remko Proesmans
* Code structure analogous to https://github.com/disk91/LIS2DH
* Maintained at https://github.com/RemkoPr/AD7142_Arduino
* Changelog:
*     ... - ongoing development release
* NOTE: THIS IS ONLY A PARIAL RELEASE. 
*/

#include "Arduino.h"
#include "stdint.h"
#include "ad7142.h"
#include "settings.h"
#include "Wire.h"


AD7142::AD7142( uint8_t add0, uint8_t add1 ) {
  // add0 is the least significant I2C address bit, add1 the second-to-least significant
  if (add0 > 1 or add1 > 1) {
    AD7142_LOG_ERROR("Invalid lower bits for I2C address!");
  } else {
    _address = AD7142_I2C_BASE_ADDRESS + (add1<<1) + add0;
  }
}

/*AD7142::~AD7142() {
  delete[] _resultsRaw;
  delete[] _resultsPf;
}*/

bool AD7142::init() {
  Wire.begin(); 
  if ( checkDeviceId() ) {
    bool ret = true;
    // Settings in comment are already true by default, included here to indicate the default settings.
    // Arguments are declared in settings.h and start with an underscore.
	
    ret &= setPwrControl(_pwrCtrl);
    ret &= setInterrupt(_interruptCtrl);
    ret &= setConnectionSetup(_setupCin0_6, _setupCin7_13);
    ret &= enableAfeOffsets(_afeOffsetDisable);
    ret &= setAfeOffsets(_afeOffset);
    return ret;
  } else {
    AD7142_LOG_ERROR(("AD7142 Init failed: device ID/revision code did not match."));
    return false;      
  }
}

bool AD7142::checkDeviceId(void) {
  AD7142_LOG_INFO(String("checkDeviceId: ") + String(readRegister(AD7142_REVISION_CODE_DEVID_ADDR)));
  return (AD7142_REVISION_CODE_DEVID == readRegister(AD7142_REVISION_CODE_DEVID_ADDR));
}

// ******************
//   Power control
// ******************

bool AD7142::setPwrControl( uint8_t pwrCtrl[8] ) {
  // writes the entire register, fnctns below write specific bits
  uint16_t val = 
    pwrCtrl[0] +          //operatingMode
    (pwrCtrl[1]<<2) +     //lowPowerConvDelay
    (pwrCtrl[2]<<4) +     //sequenceStageNum
    (pwrCtrl[3]<<8) +     //decimation
    (pwrCtrl[4]<<11) +    //interruptPolarityControl
    (pwrCtrl[5]<<12) +    //srcControl
    (pwrCtrl[6]<<13) +    //invSrcControl
    (pwrCtrl[7]<<14) ;    //cdcBiasCurrentControl
  _numStages = pwrCtrl[2];
  writeRegister(AD7142_PWR_CONTROL, val);
}

bool AD7142::setOperatingMode(uint8_t mode) {
  if ( mode > 3 ) {
    AD7142_LOG_ERROR("Invalid operating mode passed.");
    return false;
  }
  uint16_t pwr_control_reg = readRegister(AD7142_PWR_CONTROL);
  pwr_control_reg = (pwr_control_reg & 0b1111111111111100) | mode;
  return writeRegister(AD7142_PWR_CONTROL, pwr_control_reg);
}

bool AD7142::setLowPowerConvDelay(uint8_t delay_val) {
  if ( delay_val > 3 ) {
    AD7142_LOG_ERROR("Invalid delay option passed.");
    return false;
  }
  uint16_t pwr_control_reg = readRegister(AD7142_PWR_CONTROL);
  pwr_control_reg = (pwr_control_reg & 0b1111111111110011) | (delay_val<<2);
  return writeRegister(AD7142_PWR_CONTROL, pwr_control_reg);
}

bool AD7142::setSequenceStageNum(uint8_t num) {
  if ( num > 11 ) {
    AD7142_LOG_ERROR("Invalid number of conversion stages passed.");
    return false;
  }
  _numStages = num;
  uint16_t pwr_control_reg = readRegister(AD7142_PWR_CONTROL);
  pwr_control_reg = (pwr_control_reg & 0b1111111100001111) | ( (num - 1) << 4);
  return writeRegister(AD7142_PWR_CONTROL, pwr_control_reg);
}

bool AD7142::setDecimation(uint8_t rate) {
  if ( rate > 3 ) {
    AD7142_LOG_ERROR("Invalid decimation rate passed.");
    return false;
  }
  uint16_t pwr_control_reg = readRegister(AD7142_PWR_CONTROL);
  pwr_control_reg = (pwr_control_reg & 0b1111110011111111) | (rate<<8);
  return writeRegister(AD7142_PWR_CONTROL, pwr_control_reg);
}

bool AD7142::swReset(void) {
  return writeRegister(AD7142_PWR_CONTROL, 0x400); // 0000 0100 0000 0000
}

bool AD7142::setInterruptPolarityControl(uint8_t ctrl) {
  if ( ctrl > 1 ) {
    AD7142_LOG_ERROR("Invalid decimation polarity control passed.");
    return false;
  }
  uint16_t pwr_control_reg = readRegister(AD7142_PWR_CONTROL);
  pwr_control_reg = (pwr_control_reg & 0b1111011111111111) | (ctrl<<11);
  return writeRegister(AD7142_PWR_CONTROL, pwr_control_reg);
}

bool AD7142::setSrcControl(uint8_t ctrl) {
  if ( ctrl > 1 ) {
    AD7142_LOG_ERROR("Invalid source control passed for pin 15.");
    return false;
  }
  uint16_t pwr_control_reg = readRegister(AD7142_PWR_CONTROL);
  pwr_control_reg = (pwr_control_reg & 0b1110111111111111) | (ctrl<<12);
  return writeRegister(AD7142_PWR_CONTROL, pwr_control_reg);
}

bool AD7142::setInvSrcControl(uint8_t ctrl) {
  if ( ctrl > 1 ) {
    AD7142_LOG_ERROR("Invalid (inverted) source control passed for pin 16.");
    return false;
  }
  uint16_t pwr_control_reg = readRegister(AD7142_PWR_CONTROL);
  pwr_control_reg = (pwr_control_reg & 0b1101111111111111) | (ctrl<<13);
  return writeRegister(AD7142_PWR_CONTROL, pwr_control_reg);
}

bool AD7142::setCdcBiasCurrentControl(uint8_t ctrl) {
  if ( ctrl > 3 ) {
    AD7142_LOG_ERROR("Invalid CDC bias current option passed.");
    return false;
  }
  uint16_t pwr_control_reg = readRegister(AD7142_PWR_CONTROL);
  pwr_control_reg = (pwr_control_reg & 0b0011111111111111) | (ctrl<<14);
  return writeRegister(AD7142_PWR_CONTROL, pwr_control_reg);
}

// *************************************
//   Environmental calibration control
// *************************************

bool AD7142::resetConversionSequence() {
  return writeRegisterBits(AD7142_AMB_COMP_CTRL0, 15, 15, AD7142_CONV_RESET_TRUE);
}

// ************************
//   Interrupt control
// ************************

bool AD7142::setInterrupt(uint8_t interruptCtrl[12][3]) {
  uint16_t lowIntReg = 0;
  uint16_t highIntReg = 0;
  uint16_t completeIntReg = 0;
  for( int stage = 0 ; stage < 12 ; stage++ ) {
    lowIntReg       += interruptCtrl[stage][0] << stage;
    highIntReg      += interruptCtrl[stage][1] << stage;
    completeIntReg  += interruptCtrl[stage][2] << stage;
  }
  bool ret = true;
  ret &= writeRegister(AD7142_STAGE_LOW_INT_EN, lowIntReg);
  ret &= writeRegister(AD7142_STAGE_HIGH_INT_EN, highIntReg);
  ret &= writeRegister(AD7142_STAGE_COMPLETE_INT_EN, completeIntReg);
  return ret;
}

bool AD7142::isStageComplete(uint8_t stage) {
	uint16_t stage_complete_reg = readRegister(AD7142_STAGE_COMPLETE_LIMIT_INT);
	return stage_complete_reg & (1 << stage);
}

// ************************
//   CIN connection setup
// ************************

bool AD7142::setConnectionSetup(uint8_t cin0_6[12][7], uint8_t cin7_13[12][7]) {
  bool ret = true;
  for (int stage = 0 ; stage < 12 ; stage++ ) {
    uint16_t regContent0_6 = 0x0000;
    uint16_t regContent7_13 = 0x0000;
    for (int x = 0 ; x < 7 ; x++ ) {
      regContent0_6 = regContent0_6 | (cin0_6[stage][x] << 2*x);
      regContent7_13 = regContent7_13 | (cin7_13[stage][x] << 2*x);
    }
    ret &= writeRegister(AD7142_STAGE0_CONNECTION_CIN6_0 + stage*AD7142_STAGEX_BANK2_ADDR_OFFSET, regContent0_6);
    ret &= writeRegister(AD7142_STAGE0_CONNECTION_CIN13_7 + stage*AD7142_STAGEX_BANK2_ADDR_OFFSET, regContent7_13);
  }
  return ret;
}

bool AD7142::enableAfeOffsets(uint8_t afeOffsetDisable[12][2]) {
  bool ret = true;
  for( int stage = 0 ; stage < 12 ; stage++ ) {
    if( afeOffsetDisable[stage][0] > 1 or afeOffsetDisable[stage][1] > 1 ) {
      AD7142_LOG_ERROR(String("Invalid value passed to enableAfeOffsets for stage ") + String(stage));
    }
    uint8_t val = afeOffsetDisable[stage][0] + (afeOffsetDisable[stage][1]<<1);
    ret &= writeRegisterBits(
        AD7142_STAGE0_CONNECTION_CIN13_7 + stage*AD7142_STAGEX_BANK2_ADDR_OFFSET, 
        14, 15, val
        );
  }
  return ret;
}

/*uint8_t pfTo7bit(uint8_t pf) {
  // helper function that translates pF to a 7bit offset value for offset control
  if( pf > 20 ) {
    AD7142_LOG_ERROR("Invalid offset [pF] passed.");
  }
  // AD7142_LOG_DEBUG(String("pfTo7bit [pf] [output]: ") + String(pf) + String((uint8_t)round(pf / 0.16)));
  return (uint8_t)round(pf / 0.16);
}*/

bool AD7142::setAfeOffsets(uint8_t offsets[12][4]) {
	// Entirely writes all 12 STAGEX_AFE_OFFSET registers
  bool ret = true;
  for( int stage = 0 ; stage < 12 ; stage++ ) {
    uint16_t val = 
      (offsets[stage][0]) + 
      (offsets[stage][1]<<7) + 
      (offsets[stage][2]<<8) + 
      (offsets[stage][3]<<15);
    ret &= writeRegister(
      AD7142_STAGE0_AFE_OFFSET + stage*AD7142_STAGEX_BANK2_ADDR_OFFSET,
      val
      );
  }
  return ret;
}

bool AD7142::setPosAfeOffsets(uint8_t offsets[12]) {
  bool ret = true;
  for( int stage = 0 ; stage < 12 ; stage++ ) {
    ret &= writeRegisterBits(
      AD7142_STAGE0_AFE_OFFSET + stage*AD7142_STAGEX_BANK2_ADDR_OFFSET,
	  8, 14,
      offsets[stage]
      );
  }
  return ret;
}

bool AD7142::setNegAfeOffsets(uint8_t offsets[12]) {
  bool ret = true;
  for( int stage = 0 ; stage < 12 ; stage++ ) {
    ret &= writeRegisterBits(
      AD7142_STAGE0_AFE_OFFSET + stage*AD7142_STAGEX_BANK2_ADDR_OFFSET,
	  0, 6,
      offsets[stage]
      );
  }
  return ret;
}

// *****************************
//    Getters/Setters
// *****************************

	// Directly implemented in ad7142.h

// *****************************
//    Actual functionality
// *****************************

void AD7142::readResults() {
  AD7142_LOG_DEBUG(String("Attempting to read all results at once"));
  
  Wire.beginTransmission(_address); //open communication with 
  Wire.write(highByte(AD7142_ADC_RESULT_S0));  
  Wire.write(lowByte(AD7142_ADC_RESULT_S0));
  byte result = Wire.endTransmission(); 
  checkTransmissionResult(result);

  Wire.requestFrom(_address, 2*_numStages); // 16-bit registers
  byte buf[2*_numStages];
  Wire.readBytes(buf, 2*_numStages);
  
  for( int stage = 0 ; stage < _numStages ; stage++ ) {
	uint8_t valL = buf[2*stage + 1];
	uint8_t valH = buf[2*stage];
    _resultsRaw[stage] = (valH << 8) | valL;
  }
  formatResults();
}

/*
void AD7142::readResults() {
  for( int stage = 0 ; stage < _numStages ; stage++ ) {
	  // TODO: dedicated function that reads all conv data registers using only one I2C query (req 24 bytes starting ADC_RESULT_S0)
    _resultsRaw[stage] = readRegister(AD7142_STAGE0_CONV_DATA + stage*AD7142_STAGEX_BANK3_ADDR_OFFSET);
  }
  formatResults();
}*/

void AD7142::formatResults() {
  for( int stage = 0 ; stage < _numStages ; stage++ ) {
    uint16_t rawVal = _resultsRaw[stage];
    float pfVal = rawVal / 2450.0 * 0.16;
    _resultsPf[stage] = pfVal;
  }
}

void AD7142::printResults() {
  for( int stage = 0 ; stage < _numStages ; stage++ ) {
    AD7142_LOG_INFO(String("Stage ") + String(stage) + String(": ") + String(_resultsRaw[stage]) + String(" / ") + String(_resultsPf[stage]) + String("pF") + String("\n"));
  }
}

// *****************************
//    Low-level register I/O
// *****************************

bool AD7142::writeRegister(const uint16_t register_addr, const uint16_t val) {
  AD7142_LOG_DEBUG(String("Attempting write of value ") + String(val) + String(" to register ") + String(register_addr));
  Wire.beginTransmission(_address);
  Wire.write(highByte(register_addr));  
  Wire.write(lowByte(register_addr));
  Wire.write(highByte(val));  
  Wire.write(lowByte(val));
  byte result = Wire.endTransmission(true);
  return checkTransmissionResult(result);
}

bool AD7142::writeRegisterBits(const uint16_t register_addr, uint16_t start_bit, uint16_t end_bit, const uint16_t val) {
  /*
   * Writes specific conescutive bits of a register. The LSB is bit 0.
   * If 1 bit is to be written, start_bit and end_bit are
   * the same, i.e. end_bit is included in written bits.
   */
  if ( start_bit > 15 or end_bit > 15 ) {
    AD7142_LOG_ERROR("Invalid bit range passed to writeRegisterBits.");
    return false;
  } else if ( val >= pow(2, end_bit - start_bit + 1) ) {
    AD7142_LOG_ERROR(
		String("Value ") 
		+ String(val)
		+ String(" too large for passed bit range ")
		+ String(start_bit)
		+ String("-")
		+ String(end_bit)
		);
    return false;
  }
  uint16_t reg_content = readRegister(register_addr);
  uint16_t mask = pow(2, 16) - 1;
  for (int i = start_bit ; i <= end_bit ; i++ ) {
    mask = mask - (uint16_t)pow(2, i);
  }
  reg_content = (reg_content & mask) | (val<<start_bit);
  return writeRegister(register_addr, reg_content);
}

uint16_t AD7142::readRegister(const uint16_t register_addr) {
  AD7142_LOG_DEBUG(String("Attempting read from register ") + String(register_addr));
  
  Wire.beginTransmission(_address); //open communication with 
  Wire.write(highByte(register_addr));  
  Wire.write(lowByte(register_addr));
  byte result = Wire.endTransmission(); 
  checkTransmissionResult(result);

  Wire.requestFrom(_address, 2); // 16-bit registers
  uint16_t buf;
  Wire.readBytes((uint8_t*)&buf, 2);
  buf = (buf << 8) | (buf >> 8); // Flip bytes
  return buf;
}

bool AD7142::checkTransmissionResult(byte result) {
  switch ( result ) {
    case 0:
      AD7142_LOG_DEBUG("Succesfully wrote to register.");
      return(true);
    case 1:
      AD7142_LOG_ERROR("Data too long to fit in transmit buffer.");
      return(false);
    case 2:
      AD7142_LOG_ERROR("Received NACK on transmit of address.");
      return(false);
    case 3:
      AD7142_LOG_ERROR("Received NACK on transmit of data.");
      return(false);
    case 4:
      AD7142_LOG_ERROR("Unknown error.");
      return(false);
    default:
      AD7142_LOG_ERROR("Unknown return value of Wire.endTransmission, check documentation.");
      return(false);
  }
}
