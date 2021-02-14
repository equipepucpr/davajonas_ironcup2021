#include <Wire.h> 
#include "wiring_private.h" // pinPeripheral() function
#include <VL53L0X.h> //Polulu Library

//#define debug
#define MAXDIST 400 // max distance in milimeters

TwoWire myWire(&sercom4, 6, 7);

VL53L0X sensor_r;
VL53L0X sensor_l;

void setup() {
#ifdef debug
  SerialUSB.begin(115200);
#endif
  
  Wire.begin();
  myWire.begin();
  pinPeripheral(6,PIO_SERCOM_ALT);
  pinPeripheral(7,PIO_SERCOM_ALT);

  REG_PORT_DIRSET0 |= (PORT_PA04 | PORT_PA10);

  sensor_l.setBus(&myWire);
  sensor_r.setTimeout(50);
  sensor_l.setTimeout(50);
  if (!sensor_r.init() || !sensor_l.init())
  {
    while (1) {
#ifdef debug
      SerialUSB.println("Failed to detect and initialize sensor!");
      delay(1000);
#endif
    }
  }

  sensor_r.startContinuous();
  sensor_l.startContinuous();
}

void loop() {
  uint16_t dist_l, dist_r;
  dist_r = sensor_r.readRangeContinuousMillimeters();
  dist_l = sensor_l.readRangeContinuousMillimeters();

  if (dist_r <= MAXDIST) {
    REG_PORT_OUTSET0 = PORT_PA04;
  } else {
    REG_PORT_OUTCLR0 = PORT_PA04;
  }
  
  if (dist_l <= MAXDIST) {
    REG_PORT_OUTSET0 = PORT_PA10;
  } else {
    REG_PORT_OUTCLR0 = PORT_PA10;
  }

#ifdef debug
  SerialUSB.print("right: ");
  SerialUSB.print(dist_r <= MAXDIST);
  SerialUSB.print(" - ");
  SerialUSB.print(dist_r);
  SerialUSB.print(" mm - left: ");
  SerialUSB.print(dist_l <= MAXDIST);
  SerialUSB.print(" - ");
  SerialUSB.print(dist_l);
  SerialUSB.println(" mm");
  delay(1000);
#endif
}
