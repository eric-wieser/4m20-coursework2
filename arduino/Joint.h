#pragma once

#include <Servo.h>

// class for a single joint, that can read the angle and control the servo
class Joint {
private:
  uint8_t servoPin_;
  uint8_t encoderPin_;
  bool enabled_;
  Servo servo_;

  int limitMin_ = 0;
  int limitMax_ = 3000;

  int period_ = 0;

  inline int clamp(int period) {
    if(period > limitMax_) return limitMax_;
    if(period < limitMin_) return limitMin_;
    return period;
  }

public:
  Joint(uint8_t servoPin, uint8_t adcPin) : servoPin_(servoPin), encoderPin_(adcPin), enabled_(false) {
  }

  void write(int period) {
    if(period == -1) {
      servo_.detach();
      enabled_ = false;
    }
    else {
      //enable the servo if this is our first time
      if(!enabled_) {
        servo_.attach(servoPin_);
        enabled_ = true;
      }

      // clip the value to avoid damage
      period_ = clamp(period);
      servo_.writeMicroseconds(period_);
    }
  }

  void setLimits(int limitMin, int limitMax) {
    limitMin_ = limitMin;
    limitMax_ = limitMax;

    // make sure we're not already violating these limits
    if(enabled_) {
      write(period_);
    }
  }

  int read() const {
    return analogRead(encoderPin_);
  }
};
