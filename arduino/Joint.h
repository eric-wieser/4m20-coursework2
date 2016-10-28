#pragma once

#include <Servo.h>

// class for a single joint, that can read the angle and control the servo
class Joint {
private:
  uint8_t encoderPin_;
  Servo servo_;
public:
  Joint(uint8_t servoPin, uint8_t adcPin) : encoderPin_(adcPin) {
    servo_.attach(servoPin, 700, 2300);
  }

  void write(int angle) {
    servo_.write(angle);
  }

  int read() const {
    return analogRead(encoderPin_);
  }
};
