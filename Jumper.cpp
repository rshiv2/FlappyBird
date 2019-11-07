#include "Jumper.h"

#if (ARDUINO >= 100) 
  #include "Arduino.h"
 #else 
  #include "WProgram.h"
#endif

Jumper::Jumper() {
  this->height = 7;
  this->vy = 0.0;
  this->gy = 1.0/400; // takes 1/gy seconds for vy to update with regards to gravity
}

int Jumper::getHeight() {
  return this->height;
}

void Jumper::setHeight(int newHeight) {
  if (validHeight(newHeight)) {
    this->height = newHeight;
  }
}
    
float Jumper::getVel() {
  return this->vy;
}

void Jumper::setVel(float newVel) {
  this->vy = newVel;
}

float Jumper::getGrav() {
  return this->gy;
}
    
void Jumper::setGrav(float newGrav) {
  this->gy = newGrav;
}

bool Jumper::validHeight(int check) {
  return (check >= 0 && check <= 7);
}

void Jumper::jump(float thrust) {
  this->vy += thrust;
}
