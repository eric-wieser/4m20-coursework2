#pragma once

#include <Servo.h>

// class for a single joint, that can read the angle and control the servo
class Joint {
private:
  uint8_t servoPin_;
  uint8_t encoderPin_;

  enum ControlMode { DISABLED, POSITION, FORCE, POSITION_FEEDBACK};
  ControlMode mode_;
  Servo servo_;

  int16_t limitMin_ = 0;
  int16_t limitMax_ = 3000;
  int16_t adcZero_ = 0;
  float servoPerAdc_ = 0.0;

  int16_t period_ = 0;
  int16_t targetForce_ = 512;
  int16_t targetPeriod_ = 0;

  uint32_t lastUpdate_ = 0;

  inline int16_t clamp(int16_t period) {
    if(period > limitMax_) return limitMax_;
    if(period < limitMin_) return limitMin_;
    return period;
  }

  void write_(int16_t period) {
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

  int16_t getPeriod() {
    return period_;
  }

  void setLimits(int16_t limitMin, int16_t limitMax) {
    limitMin_ = limitMin;
    limitMax_ = limitMax;

    // make sure we're not already violating these limits
    if(mode_ != DISABLED) {
      write(period_);
    }
  }

  void setAdcParams(int16_t zero, float servoPer) {
    adcZero_ = zero;
    servoPerAdc_ = servoPer;
  }

  int16_t read() const {
    return analogRead(encoderPin_);
  }

  float readError() const {
    return (read() - adcZero_) * servoPerAdc_;
  }

  void writeForce(int16_t f) {
    targetForce_ = f;
    if(mode_ == DISABLED) {
      servo_.attach(servoPin_);
      period_ = (limitMax_ + limitMin_) / 2; // start in the middle of the range to maximize our chances
    }
    mode_ = FORCE;
  }

  void writeFeedback(int16_t targetPeriod) {
    // go to the position determined by a given pulse width, using feedback
    targetPeriod_ = targetPeriod;
    if(mode_ == DISABLED) {
      servo_.attach(servoPin_);
      period_ = targetPeriod;
    }
    mode_ = POSITION_FEEDBACK;
  }

  void update(uint32_t ms) {
    if(mode_ == FORCE) {
      const uint32_t UPDATE_DT = 2;
      const float K = 0.5;

      // only update every 50ms, to avoid oscillation
      if(ms < lastUpdate_ + UPDATE_DT) return;

      // simple proportional force controller
      int err = targetForce_ - read();
      write_(period_ + K*err);

      lastUpdate_ = ms;
    }

    else if(mode_ == POSITION_FEEDBACK) {
      const uint32_t UPDATE_DT = 2;//originally=2
      const float K = .1;//originally=.1
      const float K_relax = .002;//originally=.01

      // only update every 50ms, to avoid oscillation
      if(ms < lastUpdate_ + UPDATE_DT) return;

      // simple proportional force controller
      float err = targetPeriod_ - (period_ - readError());
      float relax = targetPeriod_ - period_;
      write_(period_ + K*err + K_relax * relax);
      Serial.print("debug");
      Serial.print(err);
      Serial.print("  ");
      Serial.print(relax);
      Serial.print('\0');

      lastUpdate_ = ms;
    }
  }
};
