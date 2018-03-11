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

#include <Wire.h>
#include <INA219.h>

INA219::INA219(uint8_t scl, uint8_t sda)
{
    this->_scl = scl;
    this->_sda = sda;
}
bool INA219::begin(uint8_t address, uint16_t speed, uint16_t stretch)
{
    this->_address = address;
    this->_speed = speed;

    Wire.begin(this->_sda, this->_scl);
    return true;
}

bool INA219::configure(ina219_range_t range, ina219_gain_t gain, ina219_busRes_t busRes, ina219_shuntRes_t shuntRes, ina219_mode_t mode)
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

bool INA219::calibrate(float rShuntValue, float iMaxExpected)
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

float INA219::getMaxPossibleCurrent(void)
{
    return (vShuntMax / rShunt);
}

float INA219::getMaxCurrent(void)
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

float INA219::getMaxShuntVoltage(void)
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

float INA219::getMaxPower(void)
{
    return (getMaxCurrent() * vBusMax);
}

float INA219::readBusPower(void)
{
    return (readRegister16(INA219_REG_POWER) * powerLSB);
}

float INA219::readShuntCurrent(void)
{
    return (readRegister16(INA219_REG_CURRENT) * currentLSB);
}

float INA219::readShuntVoltage(void)
{
    float voltage;

    voltage = readRegister16(INA219_REG_SHUNTVOLTAGE);

    return (voltage / 100000);
}

float INA219::readBusVoltage(void)
{
    uint16_t voltage;

    voltage = readRegister16(INA219_REG_BUSVOLTAGE);
    voltage >>= 3;

    return (voltage * 0.004);
}

ina219_range_t INA219::getRange(void)
{
    uint16_t value;

    value = readRegister16(INA219_REG_CONFIG);
    value &= 0b0010000000000000;
    value >>= 13;

    return (ina219_range_t)value;
}

ina219_gain_t INA219::getGain(void)
{
    uint16_t value;

    value = readRegister16(INA219_REG_CONFIG);
    value &= 0b0001100000000000;
    value >>= 11;

    return (ina219_gain_t)value;
}

ina219_busRes_t INA219::getBusRes(void)
{
    uint16_t value;

    value = readRegister16(INA219_REG_CONFIG);
    value &= 0b0000011110000000;
    value >>= 7;

    return (ina219_busRes_t)value;
}

ina219_shuntRes_t INA219::getShuntRes(void)
{
    uint16_t value;

    value = readRegister16(INA219_REG_CONFIG);
    value &= 0b0000000001111000;
    value >>= 3;

    return (ina219_shuntRes_t)value;
}

ina219_mode_t INA219::getMode(void)
{
    uint16_t value;

    value = readRegister16(INA219_REG_CONFIG);
    value &= 0b0000000000000111;

    return (ina219_mode_t)value;
}

int16_t INA219::readRegister16(uint8_t reg)
{
    int16_t value;
    _buffer[0] = reg;
    uint8_t vha = _buffer[1];
    uint8_t vla = _buffer[2];

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

    value = (vha << 8) | vla;
    return value;
}

void INA219::writeRegister16(uint8_t reg, uint16_t val)
{
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
}

void INA219::checkConfig(void)
{
    Serial.println("I2C:                 ");
    Serial.print("ADR: (0x");
    Serial.print(this->_address, HEX);
    Serial.println(")");
    Serial.print("Pins: ");
    Serial.print(this->_scl);
    Serial.print(" SCL, ");
    Serial.print(this->_sda);
    Serial.println(" SDA");
    Serial.print("SCL: ");
    Serial.print(this->_speed);
    Serial.println("k");

    Serial.print("Mode:                 ");
    switch (this->getMode())
    {
    case INA219_MODE_POWER_DOWN:
        Serial.println("Power-Down");
        break;
    case INA219_MODE_SHUNT_TRIG:
        Serial.println("Shunt Voltage, Triggered");
        break;
    case INA219_MODE_BUS_TRIG:
        Serial.println("Bus Voltage, Triggered");
        break;
    case INA219_MODE_SHUNT_BUS_TRIG:
        Serial.println("Shunt and Bus, Triggered");
        break;
    case INA219_MODE_ADC_OFF:
        Serial.println("ADC Off");
        break;
    case INA219_MODE_SHUNT_CONT:
        Serial.println("Shunt Voltage, Continuous");
        break;
    case INA219_MODE_BUS_CONT:
        Serial.println("Bus Voltage, Continuous");
        break;
    case INA219_MODE_SHUNT_BUS_CONT:
        Serial.println("Shunt and Bus, Continuous");
        break;
    default:
        Serial.println("unknown");
    }

    Serial.print("Range:                ");
    switch (this->getRange())
    {
    case INA219_RANGE_16V:
        Serial.println("16V");
        break;
    case INA219_RANGE_32V:
        Serial.println("32V");
        break;
    default:
        Serial.println("unknown");
    }

    Serial.print("Gain:                 ");
    switch (this->getGain())
    {
    case INA219_GAIN_40MV:
        Serial.println("+/- 40mV");
        break;
    case INA219_GAIN_80MV:
        Serial.println("+/- 80mV");
        break;
    case INA219_GAIN_160MV:
        Serial.println("+/- 160mV");
        break;
    case INA219_GAIN_320MV:
        Serial.println("+/- 320mV");
        break;
    default:
        Serial.println("unknown");
    }

    Serial.print("Bus resolution:       ");
    switch (this->getBusRes())
    {
    case INA219_BUS_RES_9BIT:
        Serial.println("9-bit");
        break;
    case INA219_BUS_RES_10BIT:
        Serial.println("10-bit");
        break;
    case INA219_BUS_RES_11BIT:
        Serial.println("11-bit");
        break;
    case INA219_BUS_RES_12BIT:
        Serial.println("12-bit");
        break;
    default:
        Serial.println("unknown");
    }

    Serial.print("Shunt resolution:     ");
    switch (this->getShuntRes())
    {
    case INA219_SHUNT_RES_9BIT_1S:
        Serial.println("9-bit / 1 sample");
        break;
    case INA219_SHUNT_RES_10BIT_1S:
        Serial.println("10-bit / 1 sample");
        break;
    case INA219_SHUNT_RES_11BIT_1S:
        Serial.println("11-bit / 1 sample");
        break;
    case INA219_SHUNT_RES_12BIT_1S:
        Serial.println("12-bit / 1 sample");
        break;
    case INA219_SHUNT_RES_12BIT_2S:
        Serial.println("12-bit / 2 samples");
        break;
    case INA219_SHUNT_RES_12BIT_4S:
        Serial.println("12-bit / 4 samples");
        break;
    case INA219_SHUNT_RES_12BIT_8S:
        Serial.println("12-bit / 8 samples");
        break;
    case INA219_SHUNT_RES_12BIT_16S:
        Serial.println("12-bit / 16 samples");
        break;
    case INA219_SHUNT_RES_12BIT_32S:
        Serial.println("12-bit / 32 samples");
        break;
    case INA219_SHUNT_RES_12BIT_64S:
        Serial.println("12-bit / 64 samples");
        break;
    case INA219_SHUNT_RES_12BIT_128S:
        Serial.println("12-bit / 128 samples");
        break;
    default:
        Serial.println("unknown");
    }

    Serial.print("Max possible current: ");
    Serial.print(this->getMaxPossibleCurrent());
    Serial.println(" A");

    Serial.print("Max current:          ");
    Serial.print(this->getMaxCurrent());
    Serial.println(" A");

    Serial.print("Max shunt voltage:    ");
    Serial.print(this->getMaxShuntVoltage());
    Serial.println(" V");

    Serial.print("Max power:            ");
    Serial.print(this->getMaxPower());
    Serial.println(" W");
}