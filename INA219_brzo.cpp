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

#if defined(ESP8266)
#include <pgmspace.h>
#define _delay_ms(ms) delayMicroseconds((ms) * 1000)
#define _delay_us(ms) delayMicroseconds((ms))
#endif

//#include <util/delay.h>
#ifdef __avr__
#include <util/delay.h>
#endif

uint8_t SDA_PIN = 5; //5
uint8_t SCL_PIN = 4; //4
uint16_t SCL_frequency_KHz = 800;

//uint8_t ADDR = 0x40;
uint8_t ADDR = 0x40;
uint32_t SCL_STRETCH_TIMEOUT = 0;
uint8_t buffer[3];

bool ICACHE_RAM_ATTR INA219_brzo::begin(uint8_t address)
{
    brzo_i2c_setup(SDA_PIN, SCL_PIN, SCL_STRETCH_TIMEOUT);
    ADDR = address;
    return true;
}

bool INA219_brzo::configure(ina219_range_t range, ina219_gain_t gain, ina219_busRes_t busRes, ina219_shuntRes_t shuntRes, ina219_mode_t mode)
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

bool INA219_brzo::calibrate(float rShuntValue, float iMaxExpected)
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

float INA219_brzo::getMaxPossibleCurrent(void)
{
    return (vShuntMax / rShunt);
}

float INA219_brzo::getMaxCurrent(void)
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

float INA219_brzo::getMaxShuntVoltage(void)
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

float INA219_brzo::getMaxPower(void)
{
    return (getMaxCurrent() * vBusMax);
}

float INA219_brzo::readBusPower(void)
{
    return (readRegister16(INA219_REG_POWER) * powerLSB);
}

float INA219_brzo::readShuntCurrent(void)
{
    return (readRegister16(INA219_REG_CURRENT) * currentLSB);
}

float INA219_brzo::readShuntVoltage(void)
{
    float voltage;

    voltage = readRegister16(INA219_REG_SHUNTVOLTAGE);

    return (voltage / 100000);
}

float INA219_brzo::readBusVoltage(void)
{
    int16_t voltage;

    voltage = readRegister16(INA219_REG_BUSVOLTAGE);
    voltage >>= 3;

    return (voltage * 0.004);
}

ina219_range_t INA219_brzo::getRange(void)
{
    uint16_t value;

    value = readRegister16(INA219_REG_CONFIG);
    value &= 0b0010000000000000;
    value >>= 13;

    return (ina219_range_t)value;
}

ina219_gain_t INA219_brzo::getGain(void)
{
    uint16_t value;

    value = readRegister16(INA219_REG_CONFIG);
    value &= 0b0001100000000000;
    value >>= 11;

    return (ina219_gain_t)value;
}

ina219_busRes_t INA219_brzo::getBusRes(void)
{
    uint16_t value;

    value = readRegister16(INA219_REG_CONFIG);
    value &= 0b0000011110000000;
    value >>= 7;

    return (ina219_busRes_t)value;
}

ina219_shuntRes_t INA219_brzo::getShuntRes(void)
{
    uint16_t value;

    value = readRegister16(INA219_REG_CONFIG);
    value &= 0b0000000001111000;
    value >>= 3;

    return (ina219_shuntRes_t)value;
}

ina219_mode_t INA219_brzo::getMode(void)
{
    uint16_t value;

    value = readRegister16(INA219_REG_CONFIG);
    value &= 0b0000000000000111;

    return (ina219_mode_t)value;
}

int16_t ICACHE_RAM_ATTR INA219_brzo::readRegister16(uint8_t reg)
{
    int16_t value;

    buffer[0] = reg;

    brzo_i2c_start_transaction(ADDR, SCL_frequency_KHz);
    brzo_i2c_write(buffer, 1, true); // 1 byte, repeat 
    brzo_i2c_read(buffer, 2, false);
    brzo_i2c_end_transaction();

    uint8_t vha = buffer[0];
    uint8_t vla = buffer[1];
    value = vha << 8 | vla;

    return value;
}

void ICACHE_RAM_ATTR INA219_brzo::writeRegister16(uint8_t reg, uint16_t val)
{
    buffer[0] = reg;

    uint8_t vla = (uint8_t)val;
    val >>= 8;

    uint8_t tmp_val = (uint8_t)val;

    buffer[1] = tmp_val;
    buffer[2] = vla;
    brzo_i2c_start_transaction(ADDR, SCL_frequency_KHz);
    brzo_i2c_write(&buffer[0], 1, true);     // 1 byte, repeat
    brzo_i2c_write(&buffer[1], 2, false); // 1 byte, no repeat
    brzo_i2c_end_transaction();
}

