#include <CapacitiveSensor.h>
#include "Arduino.h"
#include "Jumper.h"
#include "Pipe.h"

/*******************************/
/*  Constants and global vars  */
/*******************************/
const byte SEND = A4;
const byte RECEIVE = A5;
const byte ANODES[8] = {13, 12, 11, 10, 9, 8, 7, 6};
const byte CATHODES[8] = {A3, A2, A1, A0, 5, 4, 3, 2};
CapacitiveSensor sensor = CapacitiveSensor(SEND, RECEIVE);
Jumper jumper;
Pipe pipe1(7), pipe2(12);
long vy_timer;
long grav_timer;
long pipe_timer;

/*******************************/
/*          Prototypes         */
/*******************************/
void displayJumper(Jumper &jumper);
void displayPipes(Pipe &pipe1, Pipe &pipe2);
void updatePipes(long &pipe_timer);
void updateKinematics(long &vy_timer, long &grav_timer, Jumper& jumper);
void updateVelocity(long &grav_timer, Jumper& jumper);
void updateHeight(long &vy_timer, Jumper& jumper);
bool playerTapsPlate(int threshold);

void setup() {
  sensor.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1 - just as an example
  Serial.begin(115200); Serial.println("Baud set");
  
  for (int pin = 0; pin < 8; pin++) {
    pinMode(ANODES[pin], OUTPUT);
    digitalWrite(ANODES[pin], HIGH);
    pinMode(CATHODES[pin], OUTPUT);
    digitalWrite(CATHODES[pin], HIGH);
  }
  Serial.println("Pins set");

  vy_timer = millis();
  grav_timer = millis();
  pipe_timer = millis();
  Serial.println("Timers set");
}

void loop() {
  
  if (playerTapsPlate(700)) {
    jumper.jump(1/750.0);
  }

  updatePipes(pipe_timer);

  updateKinematics(vy_timer, grav_timer, jumper);
  displayJumper(jumper);
  //displayPipes(pipe1, pipe2);
}

void updatePipes(long &pipe_timer) {
  long inverseVelocity = (long)abs(1/pipe1.getVel());
  int interval = 5;
  if (millis() - pipe_timer >= inverseVelocity) {
    if (pipe1.getX() > 0) {
      pipe1.setX(pipe1.getX() - 1); 
    } else {
      pipe1.setX(pipe2.getX() + interval - 1);
      pipe1.generateRandomBase();
    }

    if (pipe2.getX() > 0) {
      pipe2.setX(pipe2.getX() - 1);
    } else {
      pipe2.setX(pipe1.getX() + interval);
      pipe2.generateRandomBase();
    }
    pipe_timer = millis();
  }
}

void updateKinematics(long &vy_timer, long &grav_timer, Jumper& jumper) {
  updateVelocity(grav_timer, jumper);
  updateHeight(vy_timer, jumper);
}

void updateVelocity(long &grav_timer, Jumper &jumper) {
  long inverseGravity = (long)abs(1/jumper.getGrav());
  if (millis() - grav_timer >= inverseGravity) {
    if (jumper.getHeight() == 7) {
      jumper.setVel(0.0);
    }
    if (jumper.getHeight() > 0) { // if jumper above ground, decrease vy further
      float newVelocity = jumper.getVel() - jumper.getGrav();
      jumper.setVel(newVelocity);
    } else if (jumper.getHeight() <= 0 && jumper.getVel() < 0) { // if jumper at ground, set vy to 0 -> shouldn't be moving up or down
      jumper.setVel(0.0);
      jumper.setHeight(0);
    }
    grav_timer = millis();
  }
}

void updateHeight(long& vy_timer, Jumper& jumper) {
  // if velocity is 0, don't update height
  // if velocity is non-zero, then change height accordingly
  if (jumper.getVel() != 0.0) {
    long inverseSpeed = (long) abs(1/jumper.getVel());
    if (millis() - vy_timer >= inverseSpeed) { 
      if (jumper.getVel() > 0.0 && jumper.getHeight() < 7) {
        jumper.setHeight(jumper.getHeight() + 1);
      } else if (jumper.getVel() < 0.0 && jumper.getHeight() > 0) {
        jumper.setHeight(jumper.getHeight() - 1);
      }
      vy_timer = millis();
    }
  }
}

bool playerTapsPlate(int threshold) {
  long total = sensor.capacitiveSensor(30);
  if (total > threshold) {
      return true;
  }
  return false;
}

void displayJumper(Jumper &jumper) {
  digitalWrite(CATHODES[1], LOW);
  int row = 7 - jumper.getHeight();
  digitalWrite(ANODES[row], LOW);

  delay(10);

  digitalWrite(ANODES[row], HIGH);
  digitalWrite(CATHODES[1], HIGH);
}

void displayPipes(Pipe &pipe1, Pipe &pipe2) {
  int lowerBound1 = pipe1.getBase();
  int upperBound1 = pipe1.getBase() + pipe1.getOpening();
  
  int lowerBound2 = pipe2.getBase();
  int upperBound2 = pipe2.getBase() + pipe2.getOpening();
  
  for (int row = 0; row <= 7; row++) {
    digitalWrite(ANODES[row], LOW);
    for (int col = 0; col <= 7; col++) {
      if (col == pipe1.getX() && (row > lowerBound1 && row <= upperBound1)) {
        digitalWrite(CATHODES[col], LOW);
      }
      if (col == pipe2.getX() && (row > lowerBound2 && row <= upperBound2)) {
        digitalWrite(CATHODES[col], LOW);
      }
      delay(5);
      digitalWrite(CATHODES[col], HIGH);
    }
    digitalWrite(ANODES[row], HIGH);
  }
}
