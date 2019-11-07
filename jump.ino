#include <CapacitiveSensor.h>
#include "Arduino.h"
#include "Jumper.h"

/*******************************/
/*  Constants and global vars  */
/*******************************/
const byte SEND = A4;
const byte RECEIVE = A5;
const byte ANODES[8] = {13, 12, 11, 10, 9, 8, 7, 6};
const byte CATHODES[8] = {A3, A2, A1, A0, 5, 4, 3, 2};
CapacitiveSensor sensor = CapacitiveSensor(SEND, RECEIVE);
Jumper jumper;
long vy_timer;
long grav_timer;

/*******************************/
/*          Prototypes         */
/*******************************/
void display(Jumper jumper);
void updateKinematics(long &vy_timer, long &grav_timer, Jumper& jumper);
void updateVelocity(long &grav_timer, Jumper& jumper);
void updateHeight(long &vy_timer, Jumper& jumper);
bool playerTapsPlate(int threshold);

void setup() {
  /* Setting Baud Rate and capacitive sensor */
  sensor.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1 - just as an example
  Serial.begin(115200); Serial.println("Baud set");
  
  /* Turning all LEDs off at the very beginning */
  for (int pin = 0; pin < 8; pin++) {
    pinMode(ANODES[pin], OUTPUT);
    digitalWrite(ANODES[pin], HIGH);
    pinMode(CATHODES[pin], OUTPUT);
    digitalWrite(CATHODES[pin], HIGH);
  }
  Serial.println("Pins set");
  
  /* Setting the timers for updating the height (vy) and velocity (grav) of the bird */
  vy_timer = millis();
  grav_timer = millis();
}

void loop() {

  if (playerTapsPlate(700)) {  // parameter refers to the threshold capacitance for a jump to occur
    jumper.jump(1/180.0);      // parameter refers to the thrust of the jump (ie. what will be added to jumper's y-velocity 
  }
  
  /* Adds the effects of gravity on the bird's velocity, and of the bird's velocity on its height */
  updateKinematics(vy_timer, grav_timer, jumper);
  delay(10);
  display(jumper);
}

void updateKinematics(long &vy_timer, long &grav_timer, Jumper& jumper) {
  
  Serial.println(jumper.getVel()*1000); // just for diagnostics, not important for actual functionality
  updateVelocity(grav_timer, jumper);
  updateHeight(vy_timer, jumper);
}

/* In this function, we can think of gravity as a change in velocity, or as 
    something to be added to or subtracted from the bird's current velocity.
    Alternatively, we can think of gravity not in terms of dx meters/second, 
    but rather as 1/dx seconds/meter. In other words, we can use the gravity
    parameter's multiplicative inverse to time how long it should take for the bird's velocity
    to decrement by gravity meters/second. Strong gravity will have a very large
    value, so its inverse will be very small. Our grav_timer will exceed this inverse
    more frequently, meaning that the bird's y-velocity will decrement by gravety 
    meters/second more frequently (and vice versa).*/
void updateVelocity(long &grav_timer, Jumper &jumper) {
  long inverseGravity = (long)abs(1/jumper.getGrav());
  if (millis() - grav_timer >= inverseGravity) {
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

/* This function works similarly to updateVelocity(), with some minor
    differences. Instead of considering the actual value of jumper's velocity,
    we make use of its sign and its multiplicative inverse (1/velocity). The multiplicative
    inverse gives how long it should take for the bird's height to be updated. If
    velocity is high, for example, it's inverse will be low, meaning that the bird's
    height will be updated more frequently (to be clear, we say the bird's height is updated
    when it moves up or down one row of LEDs). More frequent updates will, in turn, give
    the appearance that the bird is moving faster. To determine whether the bird should
    move up or down, we simply use the sign of the bird's velocity. Negative velocities 
    correspond to downwards movements, and vice versa. */
void updateHeight(long& vy_timer, Jumper& jumper) {
  // if velocity is 0, don't update height
  // if velocity is non-zero, then change height accordingly
  if (jumper.getVel() != 0.0) {
    long inverseSpeed = (long) abs(1/jumper.getVel());
    if (millis() - vy_timer >= inverseSpeed) { 
      if (jumper.getVel() > 0.0 /*&& jumper.getHeight() < 7*/) {
        jumper.setHeight(jumper.getHeight() + 1);
      } else if (jumper.getVel() < 0.0 /*&& jumper.getHeight() > 0*/) {
        jumper.setHeight(jumper.getHeight() - 1);
      }
      vy_timer = millis();
    }
  }
}

bool playerTapsPlate(int threshold) {
  long total = sensor.capacitiveSensor(30);
  Serial.println(total);
  if (total > threshold) {
    while (total > threshold) {
      total = sensor.capacitiveSensor(30);
    }
    return true;
  }
  return false;
}

void display(Jumper jumper) {
  digitalWrite(CATHODES[1], LOW);
  int row = 7 - jumper.getHeight();
  digitalWrite(ANODES[row], LOW);

  delay(10);

  digitalWrite(ANODES[row], HIGH);
  digitalWrite(CATHODES[1], HIGH);
}
