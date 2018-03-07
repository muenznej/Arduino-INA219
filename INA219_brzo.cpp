/*
INA219.cpp - Class file for the INA219 Zero-Drift, Bi-directional Current/Power Monitor Arduino Library.

Version: 1.0.0
(c) 2014 Korneliusz Jarzebski
www.jarzebski.pl

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <brzo_i2c.h>
#include "INA219_brzo.h"

uint8_t SDA_PIN = 5; //5
uint8_t SCL_PIN = 4; //4
uint16_t SCL_frequency_KHz = 1000;

//uint8_t ADDR = 0x40;
uint8_t ADDR = (0x40);
uint32_t SCL_STRETCH_TIMEOUT = 2000;
uint8_t _buffer[3] = {0x00, 0x00, 0x00};

bool ICACHE_RAM_ATTR INA219_brzo::begin(uint8_t address)
{
    brzo_i2c_setup(SDA_PIN, SCL_PIN, SCL_STRETCH_TIMEOUT);
    ADDR = address;
    return true;
}

bool ICACHE_RAM_ATTR INA219_brzo::configure(ina219_range_t range, ina219_gain_t gain, ina219_busRes_t busRes, ina219_shuntRes_t shuntRes, ina219_mode_t mode)
{
    uint16_t config = 0;

    config |= (range << 13 | gain << 11 | busRes << 7 | shuntRes << 3 | mode);

    switch (range)
    {
    case INA219_RANGE_32V:
        vBusMax = 32.0f;
        break;
    case INA219_RANGE_16V:
        vBusMax = 16.0f;
        break;
    }

    switch (gain)
    {
    case INA219_GAIN_320MV:
        vShuntMax = 0.32f;
        break;
    case INA219_GAIN_160MV:
        vShuntMax = 0.16f;
        break;
    case INA219_GAIN_80MV:
        vShuntMax = 0.08f;
        break;
    case INA219_GAIN_40MV:
        vShuntMax = 0.04f;
        break;
    }

    writeRegister16(INA219_REG_CONFIG, config);

    return true;
}

bool ICACHE_RAM_ATTR INA219_brzo::calibrate(float rShuntValue, float iMaxExpected)
{
    uint16_t calibrationValue;
    rShunt = rShuntValue;

    float iMaxPossible, minimumLSB;

    iMaxPossible = vShuntMax / rShunt;

    minimumLSB = iMaxExpected / 32767;

    currentLSB = (uint16_t)(minimumLSB * 100000000);
    currentLSB /= 100000000;
    currentLSB /= 0.0001;
    currentLSB = ceil(currentLSB);
    currentLSB *= 0.0001;

    powerLSB = currentLSB * 20;

    calibrationValue = (uint16_t)((0.04096) / (currentLSB * rShunt));

    writeRegister16(INA219_REG_CALIBRATION, calibrationValue);

    return true;
}

float ICACHE_RAM_ATTR INA219_brzo::getMaxPossibleCurrent(void)
{
    return (vShuntMax / rShunt);
}

float ICACHE_RAM_ATTR INA219_brzo::getMaxCurrent(void)
{
    float maxCurrent = (currentLSB * 32767);
    float maxPossible = getMaxPossibleCurrent();

    if (maxCurrent > maxPossible)
    {
        return maxPossible;
    }
    else
    {
        return maxCurrent;
    }
}

float ICACHE_RAM_ATTR INA219_brzo::getMaxShuntVoltage(void)
{
    float maxVoltage = getMaxCurrent() * rShunt;

    if (maxVoltage >= vShuntMax)
    {
        return vShuntMax;
    }
    else
    {
        return maxVoltage;
    }
}

float ICACHE_RAM_ATTR INA219_brzo::getMaxPower(void)
{
    return (getMaxCurrent() * vBusMax);
}

float ICACHE_RAM_ATTR INA219_brzo::readBusPower(void)
{
    return (readRegister16(INA219_REG_POWER) * powerLSB);
}

float ICACHE_RAM_ATTR INA219_brzo::readShuntCurrent(void)
{
    return (readRegister16(INA219_REG_CURRENT) * currentLSB);
}

float ICACHE_RAM_ATTR INA219_brzo::readShuntVoltage(void)
{
    float voltage;

    voltage = readRegister16(INA219_REG_SHUNTVOLTAGE);

    return (voltage / 100000);
}

float ICACHE_RAM_ATTR INA219_brzo::readBusVoltage(void)
{
    int16_t voltage;

    voltage = readRegister16(INA219_REG_BUSVOLTAGE);
    voltage >>= 3;

    return (voltage * 0.004);
}

ina219_range_t ICACHE_RAM_ATTR INA219_brzo::getRange(void)
{
    uint16_t value;

    value = readRegister16(INA219_REG_CONFIG);
    value &= 0b0010000000000000;
    value >>= 13;

    return (ina219_range_t)value;
}

ina219_gain_t ICACHE_RAM_ATTR INA219_brzo::getGain(void)
{
    uint16_t value;

    value = readRegister16(INA219_REG_CONFIG);
    value &= 0b0001100000000000;
    value >>= 11;

    return (ina219_gain_t)value;
}

ina219_busRes_t ICACHE_RAM_ATTR INA219_brzo::getBusRes(void)
{
    uint16_t value;

    value = readRegister16(INA219_REG_CONFIG);
    value &= 0b0000011110000000;
    value >>= 7;

    return (ina219_busRes_t)value;
}

ina219_shuntRes_t ICACHE_RAM_ATTR INA219_brzo::getShuntRes(void)
{
    uint16_t value;

    value = readRegister16(INA219_REG_CONFIG);
    value &= 0b0000000001111000;
    value >>= 3;

    return (ina219_shuntRes_t)value;
}

ina219_mode_t ICACHE_RAM_ATTR INA219_brzo::getMode(void)
{
    uint16_t value;

    value = readRegister16(INA219_REG_CONFIG);
    value &= 0b0000000000000111;

    return (ina219_mode_t)value;
}

int16_t ICACHE_RAM_ATTR INA219_brzo::readRegister16(uint8_t reg)
{
    int16_t value;
    _buffer[0] = reg;

    brzo_i2c_start_transaction(ADDR, SCL_frequency_KHz);
    brzo_i2c_write(&_buffer[0], 1, true); // Set Register
    brzo_i2c_read(&_buffer[1], 2, false); // Read 2 Bytes from Register
    uint8_t _ecode = brzo_i2c_end_transaction();

    uint8_t vha = _buffer[1]; // 1011 1111
    uint8_t vla = _buffer[2]; // 1001 0110
    // vha<<8 --> 1011 1111 0000 0000
    // (vha<<8)|vla --> 1011 1111 0000 0000 | 0000 0000 1001 0110
    value = (vha << 8) | vla; // shift to high byte and add low byte
#ifdef DEBUG
    Serial.println("");
    Serial.print("Reading from:");
    Serial.println(_buffer[0], BIN);
    Serial.print(value, BIN);
    Serial.println("");

    if (_ecode != 0) // on error
    {
        Serial.print("Error Code: ");
        Serial.println(_ecode);
    }
#endif
    return value;
}

void ICACHE_RAM_ATTR INA219_brzo::writeRegister16(uint8_t reg, uint16_t val)
{
    /* val = 1011 1111 1001 0110
       val >> 8: 0000 0000 1011 1111
       0xFF = 1111 1111
       (val >> 8) & 0xFF: 1011 1111 // seems to be some implicity typecast to byte
    */
    _buffer[0] = reg;
    _buffer[1] = (val >> 8) & 0xFF; // high BYTE of DWORD
    _buffer[2] = val & 0xFF;        // low BYTE of DWORD

#ifdef DEBUG
    Serial.println("");
    Serial.print("Writing to:");
    Serial.println(_buffer[0], BIN);
    Serial.print(_buffer[1], BIN);
    Serial.print(" ");
    Serial.print(_buffer[2], BIN);
    Serial.println("");
#endif
    brzo_i2c_start_transaction(ADDR, SCL_frequency_KHz);
    brzo_i2c_write(&_buffer[0], 3, true); // Set Register
    uint8_t _ecode = brzo_i2c_end_transaction();
#ifdef DEBUG
    if (_ecode != 0) // on error
    {
        Serial.print("Error Code: ");
        Serial.println(_ecode);
    }
#endif
}