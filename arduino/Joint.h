#pragma once

#include <Servo.h>

// class for a single joint, that can read the angle and control the servo
class Joint {
private:
  uint8_t servoPin_;
  uint8_t encoderPin_;
  bool enabled_;
  Servo servo_;
public:
  Joint(uint8_t servoPin, uint8_t adcPin) : servoPin_(servoPin), encoderPin_(adcPin), enabled_(false) {
  }

  void write(int period) {
    if(period == -1) {
      servo_.detach();
      enabled_ = false;
    }
    else {
      if(!enabled_) {
        servo_.attach(servoPin_);
        enabled_ = true;
      }
      servo_.writeMicroseconds(period);
    }
  }

  int read() const {
    return analogRead(encoderPin_);
  }
};
