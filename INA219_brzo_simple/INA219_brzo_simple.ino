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
  ina.calibrate(0.1, 0.05); // 0.1ohm, 50mA
  delay(2000);
}

float ina_current = 0.;
float ina_bus = 0.;
unsigned short cnt = 0;
unsigned long t1;
unsigned long t2;

#define READ_LIMIT 1000
void loop() {
  // 550µs seem to be the limit , its actually limited by the serial bus as it seems ( ~600µs)
  // This means i should add a buffer with about 10 entries before sending the serial out

  cnt++;
  if (cnt == 1) {
    t1 = micros();
  }
  ina_current = ina.readShuntCurrent();
  //ina_bus = ina.readBusVoltage();
  if (cnt == READ_LIMIT) {
    t2 = micros();

    Serial.print((t2 - t1));
    Serial.print(" µs; ");
    Serial.print((float)(t2 - t1) / READ_LIMIT, 5);
    Serial.print(" µs/READ; ");
    Serial.print(ina_current * 1000.0, 1);
    Serial.print(" mA;");
    Serial.print(ina_bus, 3);
    Serial.print(" V");
    Serial.println("");
    Serial.println("");
    cnt = 0;
  }

}