#pragma once

#include <Servo.h>

// class for a single joint, that can read the angle and control the servo
class Joint {
private:
  uint8_t servoPin_;
  uint8_t encoderPin_;

  enum ControlMode { DISABLED, POSITION, FORCE};
  ControlMode mode_;
  Servo servo_;

  int limitMin_ = 0;
  int limitMax_ = 3000;

  int period_ = 0;
  int targetForce_ = 512;

  uint32_t lastUpdate_ = 0;

  inline int clamp(int period) {
    if(period > limitMax_) return limitMax_;
    if(period < limitMin_) return limitMin_;
    return period;
  }

  void write_(int period) {
    // clip the value to avoid damage
    period_ = clamp(period);
    servo_.writeMicroseconds(period_);
  }

public:
  Joint(uint8_t servoPin, uint8_t adcPin) : servoPin_(servoPin), encoderPin_(adcPin), mode_(DISABLED) {
  }

  void write(int period) {
    if(period == -1) {
      servo_.detach();
      mode_ = DISABLED;
    }
    else {
      //enable the servo if this is our first time
      if(mode_ == DISABLED) {
        servo_.attach(servoPin_);
      }
      mode_ = POSITION;
      write_(period);
    }
  }

  void setLimits(int limitMin, int limitMax) {
    limitMin_ = limitMin;
    limitMax_ = limitMax;

    // make sure we're not already violating these limits
    if(mode_ != DISABLED) {
      write(period_);
    }
  }

  int read() const {
    return analogRead(encoderPin_);
  }

  void writeForce(int f) {
    targetForce_ = f;
    if(mode_ == DISABLED) {
      servo_.attach(servoPin_);
      period_ = (limitMax_ + limitMin_) / 2; // start in the middle of the range to maximize our chances
    }
    mode_ = FORCE;
  }

  void update(uint32_t ms) {
    if(mode_ != FORCE) return;

    // only update every 50ms, to avoid oscillation
    if(ms < lastUpdate_ + 2) return;

    // simple proportional force controller
    int err = targetForce_ - read();
    write_(period_ + err / 2);

    lastUpdate_ = ms;
  }
};
