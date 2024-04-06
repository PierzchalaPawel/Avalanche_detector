/*!
 * @file BMP280.cpp
 * @brief Provides an Arduino library for reading and interpreting Bosch BMP280 data over I2C. 
 * @n Used to read current temperature, air pressure and calculate altitude.
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [Frank](jiehan.guo@dfrobot.com)
 * @version  V1.0
 * @date  2022-11-01
 * @url https://github.com/dvarrel/BMP280.git
 */

#include "BMP280.h"

const BMP280::sRegs_t PROGMEM   _sRegs = BMP280::sRegs_t();
#ifdef __AVR__
typedef uint16_t    platformBitWidth_t;
#else
typedef uint32_t    platformBitWidth_t;
#endif

const platformBitWidth_t    _regsAddr = (platformBitWidth_t) &_sRegs;

#define writeRegBitsHelper(reg, flied, val)   writeRegBits(regOffset(&(reg)), *(uint8_t*) &(flied), *(uint8_t*) &(val))

#define __DBG   0
#if __DBG
# define __DBG_CODE(x)   Serial.print("__DBG_CODE: "); Serial.print(__FUNCTION__); Serial.print(" "); Serial.print(__LINE__); Serial.print(" "); x; Serial.println()
#else
# define __DBG_CODE(x)
#endif

uint8_t regOffset(const void *pReg)
{
  return ((platformBitWidth_t) pReg - _regsAddr + BMP280_REG_START);
}

BMP280::BMP280(const uint8_t deviceAddress)
{
  _deviceAddress = deviceAddress;
}

uint8_t BMP280::begin()
{
  __DBG_CODE(Serial.print("last register addr: "); Serial.print(regOffset(&_sRegs.temp), HEX));
  __DBG_CODE(Serial.print("first register addr: "); Serial.print(regOffset(&_sRegs.calib), HEX));
  __DBG_CODE(Serial.print("status register addr: "); Serial.print(regOffset(&_sRegs.status), HEX));
  __DBG_CODE(Serial.print("id register addr: "); Serial.print(regOffset(&_sRegs.chip_id), HEX));
  __DBG_CODE(Serial.print("res0 register addr: "); Serial.print(regOffset(&_sRegs.reserved0), HEX));

  uint8_t   temp = getReg(regOffset(&_sRegs.chip_id));
  if((temp == BMP280_REG_CHIP_ID_DEFAULT) && (lastOperateStatus == eStatusOK)) {
    reset();
    delay(200);
    getCalibrate();
    setCtrlMeasSamplingPress(eSampling_X8);
    setCtrlMeasSamplingTemp(eSampling_X8);
    setConfigFilter(eConfigFilter_off);
    setConfigTStandby(eConfigTStandby_125);
    setCtrlMeasMode(eCtrlMeasModeNormal);    // set control measurement mode to make these settings effective
  } else
    lastOperateStatus = eStatusErrDeviceNotDetected;
  return lastOperateStatus;
}

float BMP280::getTemperature()
{
  int32_t   raw = getTemperatureRaw();
  int32_t   v1, v2, T;
  if(lastOperateStatus == eStatusOK) {
    v1 = ((((raw >> 3) - ((int32_t) _sCalib.t1 << 1))) * ((int32_t) _sCalib.t2)) >> 11;
    v2 = (((((raw >> 4) - ((int32_t) _sCalib.t1)) * ((raw >> 4) - ((int32_t) _sCalib.t1))) >> 12) * ((int32_t) _sCalib.t3)) >> 14;
    _t_fine = v1 + v2;
    __DBG_CODE(Serial.print("t_fine: "); Serial.print(_t_fine));
    T = (_t_fine * 5 + 128) >> 8;
    return (float)(T)/100.0;
  }
  return 0;
}

uint32_t BMP280::getPressure()
{
  getTemperature();   // update _t_fine
  int32_t   raw = getPressureRaw();
  int64_t   rslt = 0;
  int64_t   v1, v2;
  if(lastOperateStatus == eStatusOK) {
    v1 = ((int64_t) _t_fine) - 128000;
    v2 = v1 * v1 * (int64_t) _sCalib.p6;
    v2 = v2 + ((v1 * (int64_t) _sCalib.p5) << 17);
    v2 = v2 + (((int64_t) _sCalib.p4) << 35);
    v1 = ((v1 * v1 * (int64_t) _sCalib.p3) >> 8) + ((v1 * (int64_t) _sCalib.p2) << 12);
    v1 = (((((int64_t) 1) << 47) + v1)) * ((int64_t) _sCalib.p1) >> 33;
    if(v1 == 0)
      return 0;
    rslt = 1048576 - raw;
    rslt = (((rslt << 31) - v2) * 3125) / v1;
    v1 = (((int64_t) _sCalib.p9) * (rslt >> 13) * (rslt >> 13)) >> 25;
    v2 = (((int64_t) _sCalib.p8) * rslt) >> 19;
    rslt = ((rslt + v1 + v2) >> 8) + (((int64_t) _sCalib.p7) << 4);
    return (uint32_t) (rslt / 256);
  }
  return 0;
}

int16_t BMP280::calAltitude(uint32_t pressure, float seaLevelPressure)
{
  float z = 44330 * (1.0f - pow(pressure / 100 / seaLevelPressure, 0.1903));
  return round(z);
}

void BMP280::reset()
{
  uint8_t   temp = 0xb6;
  writeReg(regOffset(&_sRegs.reset), (uint8_t*) &temp, sizeof(temp));
  delay(100);
}

void BMP280::setCtrlMeasMode(eCtrlMeasMode_t eMode)
{
  sRegCtrlMeas_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.mode = 0b11; //normal mode
  sRegVal.mode = eMode;
  writeRegBitsHelper(_sRegs.ctrlMeas, sRegFlied, sRegVal);
}

void BMP280::setCtrlMeasSamplingTemp(eSampling_t eSampling)
{
  sRegCtrlMeas_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.osrs_t = 0b111;
  sRegVal.osrs_t = eSampling;
  writeRegBitsHelper(_sRegs.ctrlMeas, sRegFlied, sRegVal);
}

void BMP280::setCtrlMeasSamplingPress(eSampling_t eSampling)
{
  sRegCtrlMeas_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.osrs_p = 0b111;
  sRegVal.osrs_p = eSampling;
  writeRegBitsHelper(_sRegs.ctrlMeas, sRegFlied, sRegVal);
}

void BMP280::setConfigFilter(eConfigFilter_t eFilter)
{
  sRegConfig_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.filter = 0b111;
  sRegVal.filter = eFilter;
  writeRegBitsHelper(_sRegs.config, sRegFlied, sRegVal);
}

void BMP280::setConfigTStandby(eConfigTStandby_t eT)
{
  sRegConfig_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.t_sb = 0b111;
  sRegVal.t_sb = eT;
  writeRegBitsHelper(_sRegs.config, sRegFlied, sRegVal);
}

void BMP280::getCalibrate()
{
  readReg(regOffset(&_sRegs.calib), (uint8_t*) &_sCalib, sizeof(_sCalib));
}

int32_t BMP280::getTemperatureRaw()
{
  sRegTemp_t    sReg;
  readReg(regOffset(&_sRegs.temp), (uint8_t*) &sReg, sizeof(sReg));
  int32_t raw = (((uint32_t) sReg.msb << 12) | ((uint32_t) sReg.lsb << 4) | ((uint32_t) sReg.xlsb));
  __DBG_CODE(Serial.print("raw: "); Serial.print(raw));
  return raw;
}

int32_t BMP280::getPressureRaw()
{
  sRegPress_t   sReg;
  readReg(regOffset(&_sRegs.press), (uint8_t*) &sReg, sizeof(sReg));
  return (((uint32_t) sReg.msb << 12) | ((uint32_t) sReg.lsb << 4) | ((uint32_t) sReg.xlsb));
}

uint8_t BMP280::getReg(uint8_t reg)
{
  uint8_t   temp;
  readReg(reg, (uint8_t*) &temp, sizeof(temp));
  return temp;
}

void BMP280::writeRegBits(uint8_t reg, uint8_t field, uint8_t val)
{
  __DBG_CODE(Serial.print("reg: "); Serial.print(reg, HEX); Serial.print(" flied: "); Serial.print(field, HEX); Serial.print(" val: "); Serial.print(val, HEX));

  uint8_t   temp;
  readReg(reg, (uint8_t*) &temp, sizeof(temp));
  temp &= ~field;
  temp |= val;
  writeReg(reg, (uint8_t*) &temp, sizeof(temp));
}

void BMP280::readReg(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  lastOperateStatus = eStatusErrDeviceNotDetected;
  Wire.beginTransmission(_deviceAddress);
  Wire.write(reg);
  if(Wire.endTransmission() != 0)
    return;
  Wire.requestFrom(_deviceAddress, len);
  for(uint8_t i = 0; i < len; i ++)
    pBuf[i] = Wire.read();
  lastOperateStatus = eStatusOK;
}

void BMP280::writeReg(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  lastOperateStatus = eStatusErrDeviceNotDetected;
  Wire.beginTransmission(_deviceAddress);
  Wire.write(reg);
  for(uint8_t i = 0; i < len; i ++)
    Wire.write(pBuf[i]);
  if(Wire.endTransmission() != 0)
    return;
  lastOperateStatus = eStatusOK;
}
