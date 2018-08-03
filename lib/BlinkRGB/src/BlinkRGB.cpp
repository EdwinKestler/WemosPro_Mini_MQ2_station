/*
  BlinkRGB.cpp - Library for RGB visual signal.
  Created by Edwin Kestler, May 29, 2018.
  Released into the public domain.
*/

#include "Arduino.h"
#include "BlinkRGB.h"

BlinkRGB::BlinkRGB(int pin){
  pinMode(pin, OUTPUT);
  _pin = pin;
}

void BlinkRGB::On(){
  digitalWrite(_pin, HIGH);
}

void BlinkRGB::Off(){
  digitalWrite(_pin, LOW);
}

void BlinkRGB::Flash(){
  digitalWrite(_pin, HIGH);
  delay(250);
  digitalWrite(_pin, LOW);
}

BlinkColor::BlinkColor(int pin0, int pin1,int pin2){
  pinMode(pin0, OUTPUT);
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  _pin0 = pin0;
  _pin1 = pin1;
  _pin2 = pin1;
}

void BlinkColor::COn(){
  digitalWrite(_pin0, HIGH);
  digitalWrite(_pin1, HIGH);
  digitalWrite(_pin2, HIGH);
}

void BlinkColor::COff(){
  digitalWrite(_pin0, LOW);
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, LOW);
}

void BlinkColor::CFlash(){
  digitalWrite(_pin0, HIGH);
  digitalWrite(_pin1, HIGH);
  digitalWrite(_pin2, HIGH);
  delay(250);
  digitalWrite(_pin0, LOW);
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, LOW);
}