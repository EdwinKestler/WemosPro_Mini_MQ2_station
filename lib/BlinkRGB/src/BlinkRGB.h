/*
  BlinkRGB.h - Library for RGB visual signal.
  Created by Edwin Kestler, May 29 , 2018.
  Released into the public domain.
*/

#ifndef BlinkRGB_h
#define BlinkRGB_h

#include "Arduino.h"

class BlinkRGB{
  public:
    BlinkRGB(int pin);
    void On();
    void Off();
    void Flash();
  private:
    int _pin;
};

class BlinkColor{
  public:
    BlinkColor(int pin0, int pin1, int pin2);
    void COn();
    void COff();
    void CFlash();
  private:
    int _pin0;
    int _pin1;
    int _pin2;
};

#endif