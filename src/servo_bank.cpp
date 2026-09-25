#include "servo_bank.h"

#include <cmath>

#include <dmc_protocol.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>

namespace dfpwm {

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

  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv(&cfg, kServoClkDiv);
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
    maxStepsPerSec_[s] = 40000;
    config_[s] = 0;
    blurOn_[s] = false;
    slewOn_[s] = false;
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
  int32_t level = steps_[axis0] + kServoPwmZero;
  if (level < kServoPwmMin) {
    level = kServoPwmMin;
  }
  if (level > kServoPwmMax) {
    level = kServoPwmMax;
  }
  pwm_set_gpio_level(pin, static_cast<uint16_t>(level));
}

void ServoBank::moveToSteps(int axis0, int32_t steps) {
  if (axis0 < 0 || axis0 >= motorCount_) {
    return;
  }
  slewOn_[axis0] = false;
  steps_[axis0] = clampSteps(axis0, steps);
  writeAxis(axis0);
  if (!pathActive_) {
    movingMask_ &= ~(1u << axis0);
  }
}

void ServoBank::setMaxSpeed(int axis0, int32_t stepsPerSec) {
  if (axis0 < 0 || axis0 >= motorCount_) {
    return;
  }
  if (stepsPerSec < 0) {
    stepsPerSec = -stepsPerSec;
  }
  if (stepsPerSec < 1) {
    stepsPerSec = 1;
  }
  maxStepsPerSec_[axis0] = stepsPerSec;
}

int32_t ServoBank::slewRate(int axis0) const {
  const int32_t maxRate = maxStepsPerSec_[axis0] < 1 ? 1 : maxStepsPerSec_[axis0];
  uint32_t speed = slewSpeed_[axis0];
  if (speed < 1) {
    speed = 1;
  }
  if (speed > 10000) {
    speed = 10000;
  }
  int32_t rate = static_cast<int32_t>((static_cast<int64_t>(maxRate) * speed) / 10000);
  if (rate < 1) {
    rate = 1;
  }
  return rate;
}

void ServoBank::slewTo(int axis0, int32_t steps, uint16_t speed) {
  if (axis0 < 0 || axis0 >= motorCount_) {
    return;
  }
  pathActive_ = false;
  slewTarget_[axis0] = clampSteps(axis0, steps);
  slewSpeed_[axis0] = speed == 0 ? 1 : speed;
  slewOn_[axis0] = steps_[axis0] != slewTarget_[axis0];
  slewLastMs_ = millis();
  if (slewOn_[axis0]) {
    movingMask_ |= (1u << axis0);
  } else {
    movingMask_ &= ~(1u << axis0);
  }
}

bool ServoBank::blurEnabled(int axis0) const {
  return axis0 >= 0 && axis0 < motorCount_ && (config_[axis0] & dfdmc::kDmcMotorConfigBlur) != 0;
}

void ServoBank::startBlur(int axis0, int32_t endSteps, uint32_t delayMs, uint32_t accelMs, uint32_t cruiseMs) {
  if (axis0 < 0 || axis0 >= motorCount_ || accelMs < 1) {
    return;
  }
  pathActive_ = false;
  slewOn_[axis0] = false;
  if (!blurClock_) {
    blurT0_ = millis();
    blurClock_ = true;
  }
  blurStart_[axis0] = steps_[axis0];
  blurEnd_[axis0] = clampSteps(axis0, endSteps);
  blurDelayMs_[axis0] = delayMs;
  blurAccelMs_[axis0] = accelMs;
  blurCruiseMs_[axis0] = cruiseMs;
  const double dist = static_cast<double>(blurEnd_[axis0] - blurStart_[axis0]);
  const double rampSec = static_cast<double>(accelMs) / 1000.0;
  const double moveSec = rampSec + static_cast<double>(cruiseMs) / 1000.0;
  double v = moveSec > 0.0 ? fabs(dist) / moveSec : 0.0;
  const double vmax = maxStepsPerSec_[axis0] < 1 ? 1.0 : static_cast<double>(maxStepsPerSec_[axis0]);
  if (v > vmax) {
    v = vmax;
  }
  blurV_[axis0] = static_cast<float>(v);
  blurOn_[axis0] = true;
  movingMask_ |= (1u << axis0);
}

static int32_t blurPose(int32_t start, int32_t end, float v, uint32_t delayMs, uint32_t accelMs, uint32_t cruiseMs,
                        uint32_t elapsedMs) {
  const double sign = end >= start ? 1.0 : -1.0;
  const double t = static_cast<double>(elapsedMs) / 1000.0;
  const double delay = static_cast<double>(delayMs) / 1000.0;
  const double accel = static_cast<double>(accelMs) / 1000.0;
  const double cruise = static_cast<double>(cruiseMs) / 1000.0;
  const double vel = v;
  double s = 0.0;
  if (t <= delay || accel <= 0.0) {
    s = 0.0;
  } else {
    const double u = t - delay;
    if (u < accel) {
      s = 0.5 * vel / accel * u * u;
    } else if (u < accel + cruise) {
      s = 0.5 * vel * accel + vel * (u - accel);
    } else if (u < accel + cruise + accel) {
      const double w = u - accel - cruise;
      s = 0.5 * vel * accel + vel * cruise + (vel * w - 0.5 * vel / accel * w * w);
    } else {
      s = vel * (cruise + accel);
    }
  }
  int32_t pos = start + static_cast<int32_t>(lround(sign * s));
  if (sign > 0.0 && pos > end) {
    pos = end;
  }
  if (sign < 0.0 && pos < end) {
    pos = end;
  }
  return pos;
}

void ServoBank::slewUpdate() {
  if (pathActive_) {
    return;
  }
  const uint32_t now = millis();
  bool anyBlur = false;
  for (int a = 0; a < motorCount_; ++a) {
    if (!blurOn_[a]) {
      continue;
    }
    anyBlur = true;
    const uint32_t elapsed = now - blurT0_;
    const uint32_t totalMs = blurDelayMs_[a] + blurAccelMs_[a] + blurCruiseMs_[a] + blurAccelMs_[a];
    steps_[a] = clampSteps(a, blurPose(blurStart_[a], blurEnd_[a], blurV_[a], blurDelayMs_[a], blurAccelMs_[a],
                                       blurCruiseMs_[a], elapsed));
    writeAxis(a);
    if (elapsed >= totalMs) {
      blurOn_[a] = false;
      movingMask_ &= ~(1u << a);
    }
  }
  if (!anyBlur) {
    blurClock_ = false;
  }
  uint32_t dt = now - slewLastMs_;
  if (dt == 0) {
    return;
  }
  if (dt > 100) {
    dt = 100;
  }
  slewLastMs_ = now;
  for (int a = 0; a < motorCount_; ++a) {
    if (!slewOn_[a]) {
      continue;
    }
    const int32_t target = slewTarget_[a];
    int32_t step = static_cast<int32_t>((static_cast<int64_t>(slewRate(a)) * dt) / 1000);
    if (step < 1) {
      step = 1;
    }
    if (steps_[a] < target) {
      steps_[a] = steps_[a] > target - step ? target : steps_[a] + step;
    } else if (steps_[a] > target) {
      steps_[a] = steps_[a] < target + step ? target : steps_[a] - step;
    }
    steps_[a] = clampSteps(a, steps_[a]);
    writeAxis(a);
    if (steps_[a] == target) {
      slewOn_[a] = false;
      movingMask_ &= ~(1u << a);
    }
  }
}

void ServoBank::stopMotion() {
  pathActive_ = false;
  movingMask_ = 0;
  blurClock_ = false;
  for (int a = 0; a < motorCount_; ++a) {
    slewOn_[a] = false;
    blurOn_[a] = false;
  }
}

void ServoBank::configure(int axis0, uint8_t flags) {
  if (axis0 < 0 || axis0 >= motorCount_) {
    return;
  }
  config_[axis0] = flags;
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
  for (int a = 0; a < motorCount_; ++a) {
    slewOn_[a] = false;
  }
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
    slewUpdate();
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
