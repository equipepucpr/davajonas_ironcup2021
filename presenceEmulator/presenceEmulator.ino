#include <Wire.h> 
#include <VL53L0X.h> //Polulu Library

//#define debug
#define MAXDIST 400 // max distance in milimeters

#define IR_INPUT_PIN  2
#define DO_NOT_USE_FEEDBACK_LED

#include "TinyIRReceiver.cpp.h"

VL53L0X sensor_r;
VL53L0X sensor_l;

volatile uint8_t DIP = 0;

void setup() {
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(2, INPUT);

  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  
#ifdef debug
  Serial.begin(115200);
#endif
  
  Wire.begin();

  sensor_r.setTimeout(50);
  sensor_l.setTimeout(50);

  digitalWrite(8, HIGH);
  digitalWrite(9, LOW);
  delay(50);
  while(!sensor_l.init()) {
#ifdef debug
      Serial.println("Failed to detect and initialize sensor_l!");
      delay(1000);
#endif
  }
  sensor_l.setAddress((uint8_t)25);

  delay(50);
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);
  delay(50);
  while(!sensor_r.init()) {
#ifdef debug
      Serial.println("Failed to detect and initialize sensor_r!");
      delay(1000);
#endif
  }
  initPCIInterruptForTinyReceiver();

  sensor_r.startContinuous();
  sensor_l.startContinuous();
}

void setDIP(uint8_t state, uint8_t value) {
  if (state > 0x1) {
    PORTB |= (1<< PB5);
    if (state == 0x2) {
      DIP = (DIP & 0b0011) | (value << 2);
    } else {
      DIP = (DIP & 0b1100) | value;
    }
    delay(100);
    PORTB &= ~(1<< PB5);
  } else {
#ifdef debug
  Serial.println("DIP:");
  Serial.print(F("  Value: 0b"));
  Serial.print(DIP, BIN);
#endif
    PORTD = (PORTD & 0x0F) | (DIP << 4);
  }
}

void handleReceivedTinyIRData(uint16_t aAddress, uint8_t aCommand, bool isRepeat) {
  static uint8_t state = 0;
  if (isRepeat)
    return;

#ifdef debug
  Serial.println("IR Receiver:");
  Serial.print(F("  Address: 0x"));
  Serial.println(aAddress, HEX);
  Serial.print(F("  Command: 0x"));
  Serial.println(aCommand, HEX);
#endif
  if (aAddress == 0x0) {
    switch (aCommand) {
      case 0x0D:
        if (state == 1) {
          state = 0;
          PORTB &= ~((1 << PB4) | (1<< PB5));
        }
        break;
      case 0x16:
        if (state == 0) {
          state = 1;
          PORTB |= (1 << PB4) | (1<< PB5);
        }
        break;
      case 0x1C:
        switch (state) {
          case 0x0:
            PORTB |= (1<< PB5);
            delay(200);
            PORTB &= ~(1<< PB5);
            state = 0x2;
            break;
          case 0x2:
            PORTB |= (1<< PB5);
            delay(200);
            PORTB &= ~(1<< PB5);
            delay(200);
            PORTB |= (1<< PB5);
            delay(200);
            PORTB &= ~(1<< PB5);
            state = 0x3;
            break;
          case 0x3:
            PORTB |= (1<< PB5);
            delay(200);
            PORTB &= ~(1<< PB5);
            delay(200);
            PORTB |= (1<< PB5);
            delay(200);
            PORTB &= ~(1<< PB5);
            delay(200);
            PORTB |= (1<< PB5);
            delay(200);
            PORTB &= ~(1<< PB5);
            state = 0x0;
            setDIP(state, 0);
            break;
        }
        break;
      case 0x19:
        setDIP(state, 0);
        break;
      case 0x45:
        setDIP(state, 1);
        break;
      case 0x46:
        setDIP(state, 2);
        break;
      case 0x47:
        setDIP(state, 3);
        break;
    }
  }
}

void loop() {
  uint16_t dist_l, dist_r;
  dist_r = sensor_r.readRangeContinuousMillimeters();
  dist_l = sensor_l.readRangeContinuousMillimeters();

  if (dist_r <= MAXDIST) {
    PORTB |= (1 << PB2);
  } else {
    PORTB &= ~(1 << PB2);
  }
  
  if (dist_l <= MAXDIST) {
    PORTB |= (1 << PB3);
  } else {
    PORTB &= ~(1 << PB3);
  }

#ifdef debug
  Serial.println("Distance Sensors:");
  Serial.print("  right: ");
  Serial.print(dist_r <= MAXDIST);
  Serial.print(" - ");
  Serial.print(dist_r);
  Serial.print(" mm - left: ");
  Serial.print(dist_l <= MAXDIST);
  Serial.print(" - ");
  Serial.print(dist_l);
  Serial.println(" mm");
  delay(1000);
#endif
}
