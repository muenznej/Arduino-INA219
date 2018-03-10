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


#include "INA219_brzo.h"

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifdef(ESP8266)
#include <Wire.h>
#else
#include "brzo_i2c.h"
#endif

INA219_brzo::INA219_brzo(uint8_t scl, uint8_t sda)
{
    _scl = scl;
    _sda = sda;
}
bool ICACHE_RAM_ATTR INA219_brzo::begin(uint8_t address, uint16_t speed, uint16_t stretch)
{
    _address = address;
    _speed = speed;
    _stretch = stretch;
#ifdef(ESP8266)
    brzo_i2c_setup(_sda, _scl, _stretch);
#else
    Wire.begin(_sda, _scl);
#endif

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
    uint16_t voltage;

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

#ifdef(ESP8266)
    brzo_i2c_start_transaction(_address, _speed);
    brzo_i2c_write(&_buffer[0], 1, true); // Set Register
    brzo_i2c_read(&_buffer[1], 2, false); // Read 2 Bytes from Register
    uint8_t _ecode = brzo_i2c_end_transaction();
    uint8_t vha = _buffer[1];
    uint8_t vla = _buffer[2];
#else
    Wire.beginTransmission(_address);
#if ARDUINO >= 100
    Wire.write(reg);
#else
    Wire.send(reg);
#endif
    Wire.endTransmission();

    Wire.requestFrom(_address, 2);
    while (!Wire.available())
    {
    };
#if ARDUINO >= 100
    uint8_t vha = Wire.read();
    uint8_t vla = Wire.read();
#else
    uint8_t vha = Wire.receive();
    uint8_t vla = Wire.receive();
#endif
#endif

#ifdef(ESP8266)
    if (_ecode != 0) // on error
    {
        Serial.print("Error Code: ");
        Serial.println(_ecode);
    }
#endif

    value = (vha << 8) | vla;
    return value;
}

void ICACHE_RAM_ATTR INA219_brzo::writeRegister16(uint8_t reg, uint16_t val)
{
#ifdef(ESP8266)
    _buffer[0] = reg;
    _buffer[1] = (val >> 8) & 0xFF; // high BYTE of DWORD
    _buffer[2] = val & 0xFF;        // low BYTE of DWORD

    brzo_i2c_start_transaction(_address, _speed);
    brzo_i2c_write(&_buffer[0], 3, true); // Set Register
    uint8_t _ecode = brzo_i2c_end_transaction();
    if (_ecode != 0) // on error
    {
        Serial.print("Error Code: ");
        Serial.println(_ecode);
    }
#else
    uint8_t vla;
    vla = (uint8_t)val;
    val >>= 8;

    Wire.beginTransmission(_address);
#if ARDUINO >= 100
    Wire.write(reg);
    Wire.write((uint8_t)val);
    Wire.write(vla);
#else
    Wire.send(reg);
    Wire.send((uint8_t)val);
    Wire.send(vla);
#endif
    Wire.endTransmission();
#endif
}
