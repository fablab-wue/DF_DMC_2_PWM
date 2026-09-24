#include "servo_bank.h"

#include <dmc_protocol.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>

namespace dfpwm {

namespace {

constexpr uint16_t kServoWrap = 65535;

uint16_t usToCompare(int32_t us) {
  if (us < 0) {
    us = 0;
  }
  const uint32_t cc = (static_cast<uint32_t>(us) * static_cast<uint32_t>(kServoWrap + 1) + 5000u) / 10000u;
  if (cc > kServoWrap) {
    return kServoWrap;
  }
  return static_cast<uint16_t>(cc);
}

}  // namespace

void ServoBank::begin(int motorCount) {
  if (motorCount < 0) {
    motorCount = 0;
  }
  if (motorCount > kPwmPins) {
    motorCount = kPwmPins;
  }
  motorCount_ = motorCount;
  pathActive_ = false;
  movingMask_ = 0;
  if (motorCount_ == 0) {
    return;
  }

  const float div = static_cast<float>(clock_get_hz(clk_sys)) /
                    (static_cast<float>(kServoHz) * static_cast<float>(kServoWrap + 1));
  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv(&cfg, div);
  pwm_config_set_wrap(&cfg, kServoWrap);
  bool sliceInited[8] = {};
  for (int s = 0; s < motorCount_; ++s) {
    const uint pin = static_cast<uint>(s);
    const uint slice = pwm_gpio_to_slice_num(pin);
    if (slice < 8 && !sliceInited[slice]) {
      pwm_init(slice, &cfg, true);
      sliceInited[slice] = true;
    }
    gpio_set_function(pin, GPIO_FUNC_PWM);
    gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_8MA);
    steps_[s] = 0;
    enabled_[s] = false;
    lowerEn_[s] = false;
    upperEn_[s] = false;
    lower_[s] = kServoMinSteps;
    upper_[s] = kServoMaxSteps;
    pwm_set_gpio_level(pin, 0);
  }
}

int32_t ServoBank::clampSteps(int axis0, int32_t steps) const {
  int32_t lo = kServoMinSteps;
  int32_t hi = kServoMaxSteps;
  if (axis0 >= 0 && axis0 < motorCount_) {
    if (lowerEn_[axis0] && lower_[axis0] > lo) {
      lo = lower_[axis0];
    }
    if (upperEn_[axis0] && upper_[axis0] < hi) {
      hi = upper_[axis0];
    }
  }
  if (lo > hi) {
    lo = kServoMinSteps;
    hi = kServoMaxSteps;
  }
  if (steps < lo) {
    return lo;
  }
  if (steps > hi) {
    return hi;
  }
  return steps;
}

void ServoBank::writeAxis(int axis0) {
  if (axis0 < 0 || axis0 >= motorCount_) {
    return;
  }
  const uint pin = static_cast<uint>(axis0);
  if (!enabled_[axis0]) {
    pwm_set_gpio_level(pin, 0);
    return;
  }
  const int32_t us = kServoCenterUs + steps_[axis0];
  pwm_set_gpio_level(pin, usToCompare(us));
}

void ServoBank::moveToSteps(int axis0, int32_t steps) {
  if (axis0 < 0 || axis0 >= motorCount_) {
    return;
  }
  steps_[axis0] = clampSteps(axis0, steps);
  writeAxis(axis0);
}

void ServoBank::stopMotion() {
  pathActive_ = false;
  movingMask_ = 0;
}

void ServoBank::configure(int axis0, uint8_t flags) {
  if (axis0 < 0 || axis0 >= motorCount_) {
    return;
  }
  enabled_[axis0] = (flags & dfdmc::kDmcMotorConfigEnabled) != 0;
  writeAxis(axis0);
}

void ServoBank::setLimits(int axis0, bool lowerEn, int32_t lower, bool upperEn, int32_t upper) {
  if (axis0 < 0 || axis0 >= motorCount_) {
    return;
  }
  lowerEn_[axis0] = lowerEn;
  upperEn_[axis0] = upperEn;
  lower_[axis0] = lower;
  upper_[axis0] = upper;
  steps_[axis0] = clampSteps(axis0, steps_[axis0]);
  writeAxis(axis0);
}

int32_t ServoBank::positionSteps(int axis0) const {
  if (axis0 < 0 || axis0 >= motorCount_) {
    return 0;
  }
  return steps_[axis0];
}

void ServoBank::attachPath(dfdmc::PathTable* path) { path_ = path; }

void ServoBank::applyFrame(int dfFrame) {
  if (path_ == nullptr) {
    return;
  }
  int local = 0;
  if (!path_->localFrame(dfFrame, &local)) {
    return;
  }
  currentFrame_ = dfFrame;
  for (int a = 0; a < motorCount_; ++a) {
    moveToSteps(a, path_->positionSteps(a, local));
  }
}

void ServoBank::moveToFramePose(int dfFrame) { applyFrame(dfFrame); }

void ServoBank::setPathSliceUs(uint32_t us) {
  if (us < 1000) {
    us = 1000;
  }
  pathSliceUs_ = us;
}

void ServoBank::pathGoRange(int dfStart, int dfEnd) {
  playEndFrame_ = dfEnd;
  playDir_ = dfStart <= dfEnd ? 1 : -1;
  currentFrame_ = dfStart;
  pathActive_ = true;
  movingMask_ = motorCount_ == 0 ? 0 : (motorCount_ >= 32 ? 0xFFFFFFFFu : ((1u << motorCount_) - 1u));
  const uint32_t sliceMs = pathSliceUs_ / 1000u;
  nextFrameMs_ = millis() + (sliceMs == 0 ? 1 : sliceMs);
}

void ServoBank::update() {
  if (!pathActive_) {
    return;
  }
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - nextFrameMs_) < 0) {
    return;
  }
  const uint32_t sliceMs = pathSliceUs_ / 1000u;
  nextFrameMs_ += sliceMs == 0 ? 1 : sliceMs;
  const int next = currentFrame_ + playDir_;
  const bool done = playDir_ > 0 ? next > playEndFrame_ : next < playEndFrame_;
  if (done) {
    pathActive_ = false;
    movingMask_ = 0;
    currentFrame_ = playEndFrame_;
    return;
  }
  applyFrame(next);
}

}  // namespace dfpwm
