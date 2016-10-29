#pragma once

#include <Servo.h>

// class for a single joint, that can read the angle and control the servo
class Joint {
private:
  uint8_t encoderPin_;
  Servo servo_;
public:
  Joint(uint8_t servoPin, uint8_t adcPin) : encoderPin_(adcPin) {
    servo_.attach(servoPin);
  }

  void write(int period) {
    servo_.writeMicroseconds(period);
  }

  int read() const {
    return analogRead(encoderPin_);
  }
};
