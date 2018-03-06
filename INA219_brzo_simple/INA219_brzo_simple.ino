#include <brzo_i2c.h>
#include <INA219_brzo.h>
INA219_brzo ina;

void setup() {
  unsigned long BAUDRATE = 500000;
  Serial.begin(BAUDRATE);
  Serial.println("Initialize INA219");
  Serial.println("-----------------------------------------------");
  ina.begin(0x40);
  ina.configure(INA219_RANGE_16V, INA219_GAIN_40MV, INA219_BUS_RES_9BIT, INA219_SHUNT_RES_9BIT_1S, INA219_MODE_SHUNT_CONT);
  ina.calibrate(0.1, 0.1); // 0.1ohm, 50mA
  delay(2000);
}

float ina_current = 0.;
float ina_bus = 0.;
unsigned short cnt = 0;
unsigned long t1;
unsigned long t2;

#define READ_LIMIT 1000
uint8_t mybuffer[3];

void loop() {
  cnt++;
  if (cnt == 1) {
    t1 = micros();
  }
  ina_current = ina.readShuntCurrent();

  if (cnt == READ_LIMIT) {
    checkConfig();
    t2 = micros();

    Serial.print((t2 - t1));
    Serial.print(" µs; ");
    Serial.print((float)(t2 - t1) / READ_LIMIT, 5);
    Serial.print(" µs/READ; ");
    Serial.print(ina_current, HEX);
    Serial.print(" mA;");
    Serial.print(ina_bus, 10);
    Serial.print(" V");
    Serial.println("");
    Serial.println("");
    cnt = 0;
  }

}

void checkConfig()
{
  Serial.print("Mode:                 ");
  switch (ina.getMode())
  {
    case INA219_MODE_POWER_DOWN:      Serial.println("Power-Down"); break;
    case INA219_MODE_SHUNT_TRIG:      Serial.println("Shunt Voltage, Triggered"); break;
    case INA219_MODE_BUS_TRIG:        Serial.println("Bus Voltage, Triggered"); break;
    case INA219_MODE_SHUNT_BUS_TRIG:  Serial.println("Shunt and Bus, Triggered"); break;
    case INA219_MODE_ADC_OFF:         Serial.println("ADC Off"); break;
    case INA219_MODE_SHUNT_CONT:      Serial.println("Shunt Voltage, Continuous"); break;
    case INA219_MODE_BUS_CONT:        Serial.println("Bus Voltage, Continuous"); break;
    case INA219_MODE_SHUNT_BUS_CONT:  Serial.println("Shunt and Bus, Continuous"); break;
    default: Serial.println("unknown");
  }
  
  Serial.print("Range:                ");
  switch (ina.getRange())
  {
    case INA219_RANGE_16V:            Serial.println("16V"); break;
    case INA219_RANGE_32V:            Serial.println("32V"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Gain:                 ");
  switch (ina.getGain())
  {
    case INA219_GAIN_40MV:            Serial.println("+/- 40mV"); break;
    case INA219_GAIN_80MV:            Serial.println("+/- 80mV"); break;
    case INA219_GAIN_160MV:           Serial.println("+/- 160mV"); break;
    case INA219_GAIN_320MV:           Serial.println("+/- 320mV"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Bus resolution:       ");
  switch (ina.getBusRes())
  {
    case INA219_BUS_RES_9BIT:         Serial.println("9-bit"); break;
    case INA219_BUS_RES_10BIT:        Serial.println("10-bit"); break;
    case INA219_BUS_RES_11BIT:        Serial.println("11-bit"); break;
    case INA219_BUS_RES_12BIT:        Serial.println("12-bit"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Shunt resolution:     ");
  switch (ina.getShuntRes())
  {
    case INA219_SHUNT_RES_9BIT_1S:    Serial.println("9-bit / 1 sample"); break;
    case INA219_SHUNT_RES_10BIT_1S:   Serial.println("10-bit / 1 sample"); break;
    case INA219_SHUNT_RES_11BIT_1S:   Serial.println("11-bit / 1 sample"); break;
    case INA219_SHUNT_RES_12BIT_1S:   Serial.println("12-bit / 1 sample"); break;
    case INA219_SHUNT_RES_12BIT_2S:   Serial.println("12-bit / 2 samples"); break;
    case INA219_SHUNT_RES_12BIT_4S:   Serial.println("12-bit / 4 samples"); break;
    case INA219_SHUNT_RES_12BIT_8S:   Serial.println("12-bit / 8 samples"); break;
    case INA219_SHUNT_RES_12BIT_16S:  Serial.println("12-bit / 16 samples"); break;
    case INA219_SHUNT_RES_12BIT_32S:  Serial.println("12-bit / 32 samples"); break;
    case INA219_SHUNT_RES_12BIT_64S:  Serial.println("12-bit / 64 samples"); break;
    case INA219_SHUNT_RES_12BIT_128S: Serial.println("12-bit / 128 samples"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Max possible current: ");
  Serial.print(ina.getMaxPossibleCurrent());
  Serial.println(" A");

  Serial.print("Max current:          ");
  Serial.print(ina.getMaxCurrent());
  Serial.println(" A");

  Serial.print("Max shunt voltage:    ");
  Serial.print(ina.getMaxShuntVoltage());
  Serial.println(" V");

  Serial.print("Max power:            ");
  Serial.print(ina.getMaxPower());
  Serial.println(" W");
}