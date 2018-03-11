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
#include <INA219_brzo.h>

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
    brzo_i2c_setup(_sda, _scl, _stretch);
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

    brzo_i2c_start_transaction(_address, _speed);
    brzo_i2c_write(&_buffer[0], 1, false); // Set Register
    brzo_i2c_read(&_buffer[1], 2, false);  // Read 2 Bytes from Register
    uint8_t _ecode = brzo_i2c_end_transaction();

    if (_ecode != 0) // on error
    {
        Serial.print("Brzo I2C Read Error on (0x");
        Serial.print(_address, HEX);
        Serial.print("): ");
        //1 : Bus not free, i.e. either SDA or SCL is low
        // 2 : Not ACK ("NACK") by slave during write: Either the slave did not respond to the given slave address; or the slave did not ACK a byte transferred by the master.
        // 4 : Not ACK ("NACK") by slave during read, i.e. slave did not respond to the given slave address
        // 8 : Clock Stretching by slave exceeded maximum clock stretching time. Most probably, there is a bus stall now!
        // 16 : Read was called with 0 bytes to be read by the master. Command not sent to the slave, since this could yield to a bus stall
        // 32 : ACK Polling timeout exceeded
        switch (_ecode)
        {
        case 0:
            Serial.println("All Ok");
            break;
        case 1:
            Serial.println("Bus not free");
            break;
        case 2:
            Serial.println("Not ACK during WRITE");
            break;
        case 4:
            Serial.println("Not ACK during READ");
            break;
        case 8:
            Serial.println("Clock Stretch Exceeded");
            break;
        case 16:
            Serial.println("Read called with 0 Bytes");
            break;
        case 32:
            Serial.println("ACK Polling timeout");
            break;
        default:
            Serial.println("unknown");
        }
        //Serial.println(_ecode);
    }

    uint8_t vha = _buffer[1];
    uint8_t vla = _buffer[2];

    value = (vha << 8) | vla;
    return value;
}

void ICACHE_RAM_ATTR INA219_brzo::writeRegister16(uint8_t reg, uint16_t val)
{
    _buffer[0] = reg;
    _buffer[1] = (val >> 8) & 0xFF; // high BYTE of DWORD
    _buffer[2] = val & 0xFF;        // low BYTE of DWORD

    brzo_i2c_start_transaction(_address, _speed);
    brzo_i2c_write(&_buffer[0], 3, false); // Set Register
    uint8_t _ecode = brzo_i2c_end_transaction();
    if (_ecode != 0) // on error
    {
        Serial.print("Brzo I2C Write Error on (0x");
        Serial.print(_address, HEX);
        Serial.print("): ");
        switch (_ecode)
        {
        case 0:
            Serial.println("All Ok");
            break;
        case 1:
            Serial.println("Bus not free");
            break;
        case 2:
            Serial.println("Not ACK during WRITE");
            break;
        case 4:
            Serial.println("Not ACK during READ");
            break;
        case 8:
            Serial.println("Clock Stretch Exceeded");
            break;
        case 16:
            Serial.println("Read called with 0 Bytes");
            break;
        case 32:
            Serial.println("ACK Polling timeout");
            break;
        default:
            Serial.println("unknown");
        }
        //Serial.println(_ecode);
    }
}

void INA219_brzo::checkConfig(void)
{
    Serial.println("I2C:                 ");
    Serial.print("ADR: (0x");
    Serial.print(_address, HEX);
    Serial.println(")");
    Serial.print("Pins: ");
    Serial.print(_scl);
    Serial.print(" SCL, ");
    Serial.print(_sda);
    Serial.println(" SDA");
    Serial.print("SCL: ");
    Serial.print(_speed);
    Serial.println("k");
    Serial.print("STRETCH: ");
    Serial.print(_stretch);
    Serial.println("ms");

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