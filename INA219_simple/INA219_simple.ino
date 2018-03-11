//#define ESP8266

#ifdef ESP8266
#include <INA219_brzo.h>
INA219_brzo ina(4,5); //INA219_brzo ina(D2,D1)
#else
#include <INA219.h>
INA219 ina(4,5); //INA219 ina(D2,D1)
#endif

void setup() {
  delay(500);
  unsigned long BAUDRATE = 500000;
  Serial.begin(BAUDRATE);
  Serial.println("Initialize INA219");
  Serial.println("-----------------------------------------------");
  ina.begin(0x40, 800, 100);
  ina.configure(INA219_RANGE_16V, INA219_GAIN_40MV, INA219_BUS_RES_9BIT, INA219_SHUNT_RES_9BIT_1S, INA219_MODE_SHUNT_BUS_CONT);
  ina.calibrate(0.1, 0.5); // 0.1ohm, 500mA
  delay(500);
  ina.checkConfig();
}

float ina_current = 0.;
float ina_bus = 0.;
unsigned short cnt = 0;
unsigned long t1;
unsigned long t2;

#define READ_LIMIT 1000

void loop() {
  cnt++;
  if (cnt == 1) {
    t1 = micros();
  }
  ina_current = ina.readShuntCurrent();

  if (cnt == READ_LIMIT) {
    t2 = micros();

    Serial.print((t2 - t1));
    Serial.print(" µs; ");
    Serial.print((float)(t2 - t1) / READ_LIMIT, 1);
    Serial.print(" µs/READ; ");
    Serial.print(ina_current * 1000.0, 2);
    Serial.print(" mA;");
    Serial.println("");
    Serial.println("");
    cnt = 0;
  }
}
