#include <Wire.h> 
#include "wiring_private.h" // pinPeripheral() function
#include <VL53L0X.h> //Polulu Library

//#define debug
#define MAXDIST 400 // max distance in milimeters

#define IR_RECEIVE_PIN  10

#define MARK_EXCESS_MICROS    20 // recommended for the cheap VS1838 modules

#include <IRremote.h>

TwoWire myWire(&sercom4, 6, 7);

VL53L0X sensor_r;
VL53L0X sensor_l;

void setup() {
  REG_PORT_DIRSET0 |= PORT_PA07 | PORT_PA05 | PORT_PA11 | PORT_PA17;
  REG_PORT_OUTCLR0 = PORT_PA11;
  REG_PORT_OUTSET0 = PORT_PA17;
  
#ifdef debug
  SerialUSB.begin(115200);
#endif
  
  Wire.begin();
  myWire.begin();
  pinPeripheral(6,PIO_SERCOM_ALT);
  pinPeripheral(7,PIO_SERCOM_ALT);

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

  IrReceiver.begin(IR_RECEIVE_PIN);

  sensor_r.startContinuous();
  sensor_l.startContinuous();
}

void loop() {
  if (IrReceiver.decode()) {
#ifdef debug
    Serial.println("IR Receiver:");
    Serial.print(F("  Address: 0x"));
    Serial.println(IrReceiver.decodedIRData.address, HEX);
    Serial.print(F("  Command: 0x"));
    Serial.println(IrReceiver.decodedIRData.command , HEX);
#endif
    if (IrReceiver.decodedIRData.address == 0x707) {
      switch (IrReceiver.decodedIRData.command) {
        case 0x1:
          REG_PORT_OUTSET0 = PORT_PA17;
          REG_PORT_OUTCLR0 = PORT_PA11;
          break;
        case 0x2:
          REG_PORT_OUTCLR0 = PORT_PA17;
          REG_PORT_OUTSET0 = PORT_PA11;
          break;
      }
    }

    IrReceiver.resume();
  }
  
  uint16_t dist_l, dist_r;
  dist_r = sensor_r.readRangeContinuousMillimeters();
  dist_l = sensor_l.readRangeContinuousMillimeters();

  if (dist_r <= MAXDIST) {
    REG_PORT_OUTSET0 = PORT_PA05;
  } else {
    REG_PORT_OUTCLR0 = PORT_PA05;
  }
  
  if (dist_l <= MAXDIST) {
    REG_PORT_OUTSET0 = PORT_PA07;
  } else {
    REG_PORT_OUTCLR0 = PORT_PA07;
  }

#ifdef debug
  SerialUSB.println("Distance Sensors:");
  SerialUSB.print("  right: ");
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
