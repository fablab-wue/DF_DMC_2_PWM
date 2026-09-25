#pragma once

// Servo PWM at 306.35 Hz. Dragonframe steps are raw PWM counts from 30113.

#include "config.h"

#include <path_table.h>

#include <cstdint>

namespace dfpwm {

class ServoBank {
 public:
  void begin(int motorCount);
  int motorCount() const { return motorCount_; }

  void moveToSteps(int axis0, int32_t steps);
  void slewTo(int axis0, int32_t steps, uint16_t speed);
  void startBlur(int axis0, int32_t endSteps, uint32_t delayMs, uint32_t accelMs, uint32_t cruiseMs);
  bool blurEnabled(int axis0) const;
  void setMaxSpeed(int axis0, int32_t stepsPerSec);
  void stopMotion();
  void configure(int axis0, uint8_t flags);
  void setLimits(int axis0, bool lowerEn, int32_t lower, bool upperEn, int32_t upper);
  int32_t positionSteps(int axis0) const;
  uint32_t movingMask() const { return movingMask_; }
  bool moving() const { return movingMask_ != 0; }
  bool pathActive() const { return pathActive_; }
  int currentFrame() const { return currentFrame_; }
  void setCurrentFrame(int frame) { currentFrame_ = frame; }

  void attachPath(dfdmc::PathTable* path);
  void moveToFramePose(int dfFrame);
  void setPathSliceUs(uint32_t us);
  void pathGoRange(int dfStart, int dfEnd);
  void update();

 private:
  int32_t clampSteps(int axis0, int32_t steps) const;
  void writeAxis(int axis0);
  void applyFrame(int dfFrame);
  void slewUpdate();
  int32_t slewRate(int axis0) const;

  dfdmc::PathTable* path_ = nullptr;
  int motorCount_ = 0;
  int32_t steps_[kPwmPins]{};
  bool enabled_[kPwmPins]{};
  uint8_t config_[kPwmPins]{};
  bool blurOn_[kPwmPins]{};
  int32_t blurStart_[kPwmPins]{};
  int32_t blurEnd_[kPwmPins]{};
  float blurV_[kPwmPins]{};
  uint32_t blurDelayMs_[kPwmPins]{};
  uint32_t blurAccelMs_[kPwmPins]{};
  uint32_t blurCruiseMs_[kPwmPins]{};
  uint32_t blurT0_ = 0;
  bool blurClock_ = false;
  bool lowerEn_[kPwmPins]{};
  bool upperEn_[kPwmPins]{};
  int32_t lower_[kPwmPins]{};
  int32_t upper_[kPwmPins]{};
  int32_t maxStepsPerSec_[kPwmPins]{};
  int32_t slewTarget_[kPwmPins]{};
  uint16_t slewSpeed_[kPwmPins]{};
  bool slewOn_[kPwmPins]{};
  uint32_t slewLastMs_ = 0;
  bool pathActive_ = false;
  uint32_t movingMask_ = 0;
  uint32_t pathSliceUs_ = 41667;
  uint32_t nextFrameMs_ = 0;
  int currentFrame_ = 1;
  int playEndFrame_ = 1;
  int playDir_ = 1;
};

}  // namespace dfpwm
