#include "bridge.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace dfpwm {

using namespace dfdmc;

BoardMode DmcBridge::readDip() const {
  bool on[3] = {};
  for (int i = 0; i < 4; ++i) {
    pinMode(kSwPins[i], INPUT_PULLUP);
  }
  for (int i = 0; i < 3; ++i) {
    on[i] = digitalRead(kSwPins[i]) == LOW;
  }
  const bool exp = digitalRead(kSwPins[3]) == LOW;
  const int code = (on[0] ? 4 : 0) | (on[1] ? 2 : 0) | (on[2] ? 1 : 0);
  const int motors = code == 7 ? 16 : code * 2;
  return BoardMode{motors, kPwmPins - motors, exp};
}

void DmcBridge::setup() {
  Serial.begin(115200);
  Serial.setTimeout(0);
  mode_ = readDip();
  for (int i = 0; i < mode_.dmxPwmCount; ++i) {
    dmxPwmPins_[i] = static_cast<uint8_t>(mode_.motorCount + i);
  }
  const GioMap gioMap{kGioOutPins, 2, &kGioInPin, 1, kCameraPin, kBuzzerPin, 255};
  gio_.begin(gioMap);
  servos_.begin(mode_.motorCount);
  servos_.attachPath(&path_);
  dmx_.begin(kDmxTxPin, mode_.dmxPwmCount > 0 ? dmxPwmPins_ : nullptr, static_cast<uint8_t>(mode_.dmxPwmCount),
             kDmxPwmHz, mode_.exponential);
  statusLed_.begin();
  dfConnected_ = false;
  bootHelloSent_ = false;
}

void DmcBridge::loop() {
  maybeSendBootHello();
  while (Serial.available()) {
    const uint8_t byte = static_cast<uint8_t>(Serial.read());
    DmcFrame frame;
    if (dmcParser_.feed(byte, &frame)) {
      statusLed_.markPacket();
      handleDmcFrame(frame);
    }
  }
  dmx_.update();
  gio_.tick();
  servos_.update();
  maybeSendPositionReport();
  maybeUnsolicitedGio();
  maybeRestoreBloop();
  maybeFinishPath();
  maybeUpdateShoot();
  pumpPendingPlay();
  statusLed_.update(dfConnected_);
}

bool DmcBridge::motorIndexValid(uint8_t motor) const {
  return motor >= 1 && motor <= static_cast<uint8_t>(servos_.motorCount());
}

void DmcBridge::sendDmcFrame(uint32_t id, uint16_t type, const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> packet;
  packet.reserve(kDmcHeaderSize + payload.size() + kDmcCsumSize);
  packet.push_back('D');
  packet.push_back('F');
  appendDwordLE(packet, id);
  appendWordLE(packet, type);
  appendWordLE(packet, static_cast<uint16_t>(payload.size()));
  packet.insert(packet.end(), payload.begin(), payload.end());
  const uint16_t rawChecksum = computeChecksum(packet.data(), packet.size());
  appendWordLE(packet, encodeChecksum(rawChecksum));
  Serial.write(packet.data(), packet.size());
}

void DmcBridge::sendDmcAck(uint32_t id, uint16_t type, uint32_t status) {
  std::vector<uint8_t> payload;
  appendDwordLE(payload, status);
  sendDmcFrame(id, static_cast<uint16_t>(type | kDmcMsgFlagAck), payload);
}

void DmcBridge::sendDmcHello(uint32_t id) {
  char name[48];
  const int wrote = snprintf(name, sizeof(name), "jDF-PWM V1 %dS+%dL+2O+1I+CT+DMX", mode_.motorCount, mode_.dmxPwmCount);
  std::vector<uint8_t> payload(32, 0);
  if (wrote > 0) {
    const size_t n = static_cast<size_t>(wrote) > 32 ? 32 : static_cast<size_t>(wrote);
    std::memcpy(payload.data(), name, n);
  }
  appendByte(payload, kHelloVersionMajor);
  appendByte(payload, kHelloVersionMinor);
  appendByte(payload, kHelloVersionRev);
  appendByte(payload, static_cast<uint8_t>(mode_.motorCount));
  appendWordLE(payload, static_cast<uint16_t>(kDmxChannels));
  appendByte(payload, 2);
  appendByte(payload, 1);
  appendByte(payload, 0);
  appendDwordLE(payload, static_cast<uint32_t>(kMaxUploadFrames));
  appendDwordLE(payload, kDmcCapRealTime | kDmcCapGoMotion | kDmcCapGoMotion2 | kDmcCapRealTimeCamera);
  appendWordLE(payload, kHelloProtocolVersion);
  sendDmcFrame(id, kDmcMsgHi, payload);
}

void DmcBridge::sendMotorStatus(uint32_t id) {
  std::vector<uint8_t> payload;
  appendDwordLE(payload, servos_.movingMask());
  appendByte(payload, dmx_.ramping() ? 1 : 0);
  sendDmcFrame(id, kDmcMsgMotorStatus, payload);
}

void DmcBridge::sendMotorPositions(uint32_t id) {
  std::vector<uint8_t> payload;
  appendDwordLE(payload, 0);
  for (int a = 0; a < servos_.motorCount(); ++a) {
    appendDwordLE(payload, static_cast<uint32_t>(servos_.positionSteps(a)));
  }
  sendDmcFrame(id, kDmcMsgMotorGetPosition, payload);
}

void DmcBridge::sendGioIn(uint32_t id) {
  std::vector<uint8_t> payload;
  appendDwordLE(payload, gio_.inputs());
  sendDmcFrame(id, kDmcMsgGioIn, payload);
}

void DmcBridge::maybeSendBootHello() {
  const bool cdcConnected = static_cast<bool>(Serial);
  if (cdcConnected && !bootHelloSent_) {
    sendDmcHello(0);
    bootHelloSent_ = true;
  } else if (!cdcConnected) {
    bootHelloSent_ = false;
  }
}

void DmcBridge::maybeSendPositionReport() {
  if (!dfConnected_ || !servos_.moving()) {
    return;
  }
  const uint32_t now = millis();
  if (now - lastPositionTxMs_ < kPositionReportMs) {
    return;
  }
  lastPositionTxMs_ = now;
  sendMotorPositions(0);
}

void DmcBridge::maybeUnsolicitedGio() {
  if (dfConnected_ && gio_.pollInputChange()) {
    sendGioIn(0);
  }
}

void DmcBridge::maybeRestoreBloop() {
  if (!bloopDmxOn_) {
    return;
  }
  if (static_cast<int32_t>(millis() - bloopDmxUntilMs_) < 0) {
    return;
  }
  dmx_.apply(bloopDmxChannel_, &bloopSavedLevel_, 1, false);
  bloopDmxOn_ = false;
}

void DmcBridge::maybeFinishPath() {
  const bool active = servos_.pathActive();
  if (active) {
    postrollWaiting_ = false;
  } else if (wasPathActive_ && dfConnected_ && !shootRun_) {
    if (pendingPostrollMs_ > 0) {
      postrollUntilMs_ = millis() + pendingPostrollMs_;
      pendingPostrollMs_ = 0;
      postrollWaiting_ = true;
    } else if (!postrollWaiting_) {
      sendDmcFrame(0, kDmcMsgRtEnd, {});
    }
  }
  if (postrollWaiting_ && static_cast<int32_t>(millis() - postrollUntilMs_) >= 0) {
    postrollWaiting_ = false;
    if (dfConnected_ && !shootRun_) {
      sendDmcFrame(0, kDmcMsgRtEnd, {});
    }
  }
  wasPathActive_ = active;
}

void DmcBridge::clearShoot() {
  if (shootShutter_) {
    gio_.setCameraShutter(false);
    shootShutter_ = false;
  }
  shootArmed_ = false;
  shootRun_ = false;
}

void DmcBridge::maybeUpdateShoot() {
  if (!shootRun_) {
    return;
  }
  const uint32_t elapsed = millis() - shootT0_;
  if (!shootShutter_ && elapsed >= shootShutterOpenMs_) {
    gio_.setCameraShutter(true);
    shootShutter_ = true;
  }
  if (shootShutter_ && elapsed >= shootShutterCloseMs_) {
    gio_.setCameraShutter(false);
    shootShutter_ = false;
  }
  if (elapsed >= shootDoneMs_ && !servos_.moving()) {
    shootRun_ = false;
  }
}

static int32_t sampleSteps(const dfdmc::PathTable& path, int axis, double frameTime) {
  const int lo = path.startFrame() < path.endFrame() ? path.startFrame() : path.endFrame();
  const int hi = path.startFrame() > path.endFrame() ? path.startFrame() : path.endFrame();
  if (frameTime < lo) {
    frameTime = lo;
  }
  if (frameTime > hi) {
    frameTime = hi;
  }
  const int f0 = static_cast<int>(floor(frameTime));
  int f1 = f0 + 1;
  if (f1 > hi) {
    f1 = hi;
  }
  int local0 = 0;
  if (!path.localFrame(f0, &local0)) {
    return 0;
  }
  const int32_t p0 = path.positionSteps(axis, local0);
  if (f1 == f0) {
    return p0;
  }
  int local1 = 0;
  if (!path.localFrame(f1, &local1)) {
    return p0;
  }
  const double u = frameTime - static_cast<double>(f0);
  return p0 + static_cast<int32_t>(lround((path.positionSteps(axis, local1) - p0) * u));
}

void DmcBridge::handleShootFrame(const DmcFrame& frame) {
  int32_t dfFrame = 0;
  uint8_t direction = 1;
  uint32_t exposureMs = 0;
  uint16_t blurX10 = 0;
  if (frame.payload.size() < 11 || !readSignedDwordLE(frame.payload, 0, &dfFrame) ||
      !readByte(frame.payload, 4, &direction) || !readDwordLE(frame.payload, 5, &exposureMs) ||
      !readWordLE(frame.payload, 9, &blurX10)) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
    return;
  }
  if (servos_.moving() || shootRun_) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrMoving);
    return;
  }
  if (exposureMs < 1 || exposureMs > 60000) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
    return;
  }
  if (blurX10 == 0) {
    blurX10 = 1000;
  }
  const int n = servos_.motorCount();
  bool overrideAxis[kPwmPins] = {};
  int32_t posA[kPwmPins] = {};
  int32_t posB[kPwmPins] = {};
  size_t off = 11;
  while (off + 9 <= frame.payload.size()) {
    uint8_t motor = 0;
    int32_t a = 0;
    int32_t b = 0;
    if (!readByte(frame.payload, off, &motor) || !readSignedDwordLE(frame.payload, off + 1, &a) ||
        !readSignedDwordLE(frame.payload, off + 5, &b)) {
      break;
    }
    if (motorIndexValid(motor)) {
      overrideAxis[motor - 1] = true;
      posA[motor - 1] = a;
      posB[motor - 1] = b;
    }
    off += 9;
  }
  const int dirSign = direction ? 1 : -1;
  const double dt = static_cast<double>(blurX10) * 0.0005;
  const double te = static_cast<double>(exposureMs) / 1000.0;
  for (int axis = 0; axis < n; ++axis) {
    int32_t pose = servos_.positionSteps(axis);
    int local = 0;
    if (path_.localFrame(dfFrame, &local)) {
      pose = path_.positionSteps(axis, local);
    }
    int32_t openPose = pose;
    int32_t closePose = pose;
    if (servos_.blurEnabled(axis) && overrideAxis[axis]) {
      const double delta = (0.5 - fabs(dt)) * static_cast<double>(posB[axis] - posA[axis]);
      openPose = posA[axis] + static_cast<int32_t>(lround(delta));
      closePose = posB[axis] - static_cast<int32_t>(lround(delta));
    } else if (servos_.blurEnabled(axis) && !path_.empty()) {
      openPose = sampleSteps(path_, axis, static_cast<double>(dfFrame) - dirSign * dt);
      closePose = sampleSteps(path_, axis, static_cast<double>(dfFrame) + dirSign * dt);
    }
    const int32_t span = closePose - openPose;
    const int sign = span >= 0 ? 1 : -1;
    const double v = te > 0.0 ? fabs(static_cast<double>(span)) / te : 0.0;
    const int32_t accel = static_cast<int32_t>(lround(0.5 * v));
    const int32_t pre = openPose - sign * accel;
    shootEndSteps_[axis] = closePose + sign * accel;
    shootBlur_[axis] = span != 0;
    servos_.slewTo(axis, shootBlur_[axis] ? pre : pose, 10000);
  }
  shootDelayMs_ = 0;
  shootAccelMs_ = 1000;
  shootCruiseMs_ = exposureMs;
  shootShutterOpenMs_ = 1000;
  shootShutterCloseMs_ = 1000 + exposureMs;
  shootDoneMs_ = 2000 + exposureMs;
  shootShutter_ = false;
  shootRun_ = false;
  shootArmed_ = true;
  sendDmcAck(frame.id, frame.type, kDmcAckOk);
}

void DmcBridge::handleShootFrame2(const DmcFrame& frame) {
  int32_t dfFrame = 0;
  uint32_t exposureMs = 0;
  uint16_t openWord = 0;
  uint16_t closeWord = 0;
  if (frame.payload.size() < 12 || !readSignedDwordLE(frame.payload, 0, &dfFrame) ||
      !readDwordLE(frame.payload, 4, &exposureMs) || !readWordLE(frame.payload, 8, &openWord) ||
      !readWordLE(frame.payload, 10, &closeWord)) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
    return;
  }
  if (servos_.moving() || shootRun_) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrMoving);
    return;
  }
  if (exposureMs == 0) {
    exposureMs = 1000;
  }
  if (exposureMs > 60000) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
    return;
  }
  const int16_t shutterOpen = static_cast<int16_t>(openWord);
  const int16_t shutterClose = static_cast<int16_t>(closeWord);
  if (shutterClose <= shutterOpen) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
    return;
  }

  const int n = servos_.motorCount();
  bool overrideAxis[kPwmPins] = {};
  int32_t posA[kPwmPins] = {};
  int32_t posB[kPwmPins] = {};
  size_t off = 12;
  while (off + 9 <= frame.payload.size()) {
    uint8_t motor = 0;
    int32_t a = 0;
    int32_t b = 0;
    if (!readByte(frame.payload, off, &motor) || !readSignedDwordLE(frame.payload, off + 1, &a) ||
        !readSignedDwordLE(frame.payload, off + 5, &b)) {
      break;
    }
    if (motorIndexValid(motor)) {
      overrideAxis[motor - 1] = true;
      posA[motor - 1] = a;
      posB[motor - 1] = b;
    }
    off += 9;
  }

  const double te = static_cast<double>(exposureMs) / 1000.0;
  const double degrees = static_cast<double>(shutterClose - shutterOpen);
  const double secondsPerDegree = te / degrees;
  const double moveT = 360.0 * secondsPerDegree;
  const uint32_t moveMs = static_cast<uint32_t>(lround(moveT * 1000.0));
  const uint32_t accelMs = static_cast<uint32_t>(lround(moveT * 0.125 * 1000.0));
  const uint32_t cruiseMs = moveMs > 2 * accelMs ? moveMs - 2 * accelMs : 0;
  uint32_t delayMs = 0;
  uint32_t shutterOpenMs = 0;
  if (shutterOpen < 0) {
    delayMs = static_cast<uint32_t>(lround(-shutterOpen * secondsPerDegree * 1000.0));
    shutterOpenMs = 0;
  } else {
    shutterOpenMs = static_cast<uint32_t>(lround(shutterOpen * secondsPerDegree * 1000.0));
  }
  const uint32_t shutterCloseMs = shutterOpenMs + exposureMs;
  uint32_t postMs = 0;
  if (shutterClose > 360) {
    postMs = static_cast<uint32_t>(lround((shutterClose - 360) * secondsPerDegree * 1000.0));
  }
  const uint32_t doneMs = delayMs + moveMs + postMs;
  if (doneMs > 120000 || accelMs < 1) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
    return;
  }

  for (int axis = 0; axis < n; ++axis) {
    int32_t pose = servos_.positionSteps(axis);
    int local = 0;
    if (path_.localFrame(dfFrame, &local)) {
      pose = path_.positionSteps(axis, local);
    }
    int32_t startPose = pose;
    int32_t endPose = pose;
    if (servos_.blurEnabled(axis) && overrideAxis[axis]) {
      startPose = posA[axis];
      endPose = posB[axis];
    } else if (servos_.blurEnabled(axis) && !path_.empty()) {
      startPose = sampleSteps(path_, axis, static_cast<double>(dfFrame) - 0.5);
      endPose = sampleSteps(path_, axis, static_cast<double>(dfFrame) + 0.5);
    }
    shootBlur_[axis] = startPose != endPose;
    shootEndSteps_[axis] = endPose;
    servos_.slewTo(axis, shootBlur_[axis] ? startPose : pose, 10000);
  }
  shootDelayMs_ = delayMs;
  shootAccelMs_ = accelMs;
  shootCruiseMs_ = cruiseMs;
  shootShutterOpenMs_ = shutterOpenMs;
  shootShutterCloseMs_ = shutterCloseMs;
  shootDoneMs_ = doneMs;
  shootShutter_ = false;
  shootRun_ = false;
  shootArmed_ = true;
  sendDmcAck(frame.id, frame.type, kDmcAckOk);
}

void DmcBridge::fireBloop(unsigned ms) {
  if (ms == 0) {
    ms = 100;
  }
  gio_.pulseBuzzer(ms);
}

void DmcBridge::applyFrameTrigger(int dfFrame) {
  int local = 0;
  if (path_.triggerMask() == 0 || !path_.localFrame(dfFrame, &local)) {
    return;
  }
  gio_.setOutputs(path_.triggerAtLocal(local));
}

void DmcBridge::pumpPendingPlay() {
  if (!pendingPlay_) {
    return;
  }
  if (millis() < pendingPlayAtMs_) {
    return;
  }
  pendingPlay_ = false;
  if (pendingBloopMs_ > 0) {
    fireBloop(pendingBloopMs_);
    if (pendingBloopDmx_ >= 1 && pendingBloopDmx_ <= kDmxChannels) {
      if (bloopDmxOn_) {
        dmx_.apply(bloopDmxChannel_, &bloopSavedLevel_, 1, false);
      }
      bloopDmxChannel_ = pendingBloopDmx_;
      bloopSavedLevel_ = dmx_.levelAt(bloopDmxChannel_);
      const uint8_t full = 255;
      dmx_.apply(bloopDmxChannel_, &full, 1, false);
      bloopDmxUntilMs_ = millis() + pendingBloopMs_;
      bloopDmxOn_ = true;
    }
  }
  applyFrameTrigger(pendingStart_);
  servos_.pathGoRange(pendingStart_, pendingEnd_);
}

uint32_t DmcBridge::psFromFpsX1000(uint32_t fpsX1000) const {
  if (fpsX1000 < 1) {
    fpsX1000 = 24000;
  }
  uint32_t us = (1000000000UL + (fpsX1000 / 2)) / fpsX1000;
  if (us < 1000) {
    us = 1000;
  }
  return us;
}

void DmcBridge::handleDmcFrame(const DmcFrame& frame) {
  if (!frame.valid) {
    sendDmcAck(frame.id, frame.type, kDmcAckErrChecksum);
    return;
  }
  dfConnected_ = true;

  if (frame.type == kDmcMsgHi) {
    dfConnected_ = true;
    sendDmcHello(frame.id);
    return;
  }

  switch (frame.type) {
    case kDmcMsgGioOut: {
      uint32_t bits = 0;
      if (!readDwordLE(frame.payload, 0, &bits)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      gio_.setOutputs(bits);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgGioIn:
      sendGioIn(frame.id);
      break;
    case kDmcMsgGioCam: {
      uint32_t cam = 0;
      if (!readDwordLE(frame.payload, 0, &cam)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      gio_.setCameraShutter((cam & kDmcGioCamShutter) != 0);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgDmx: {
      uint8_t ramp = 0;
      uint16_t channel = 1;
      uint16_t count = 0;
      const uint8_t* levels = nullptr;
      bool parsed = false;
      if (frame.payload.size() >= 7) {
        uint32_t channel32 = 0;
        uint16_t counted = 0;
        uint8_t rampByte = 0;
        if (readDwordLE(frame.payload, 0, &channel32) && readWordLE(frame.payload, 4, &counted) &&
            readByte(frame.payload, 6, &rampByte) && counted > 0 && 7u + counted == frame.payload.size() &&
            channel32 >= 1 && channel32 <= static_cast<uint32_t>(kDmxChannels)) {
          channel = static_cast<uint16_t>(channel32);
          count = counted;
          ramp = rampByte;
          levels = frame.payload.data() + 7;
          parsed = true;
        }
      }
      if (!parsed) {
        if (frame.payload.size() < 4 || !readByte(frame.payload, 0, &ramp) || !readWordLE(frame.payload, 1, &channel)) {
          sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
          break;
        }
        count = static_cast<uint16_t>(frame.payload.size() - 3);
        levels = frame.payload.data() + 3;
      }
      if (channel < 1 || channel > kDmxChannels || count == 0) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      dmx_.apply(channel, levels, count, ramp != 0);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgMotorStatus:
      sendMotorStatus(frame.id);
      break;
    case kDmcMsgMotorGetPosition:
      sendMotorPositions(frame.id);
      break;
    default:
      handleMotorOrRt(frame);
      break;
  }
}

void DmcBridge::handleMotorOrRt(const DmcFrame& frame) {
  switch (frame.type) {
    case kDmcMsgMotorMove: {
      uint8_t motor = 0;
      int32_t position = 0;
      if (frame.payload.size() != 5 || !readByte(frame.payload, 0, &motor) ||
          !readSignedDwordLE(frame.payload, 1, &position)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      servos_.slewTo(motor - 1, position, 10000);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgMotorStop: {
      uint8_t motor = 0;
      if (!readByte(frame.payload, 0, &motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      servos_.stopMotion();
      clearShoot();
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      sendMotorPositions(frame.id);
      break;
    }
    case kDmcMsgMotorStopAll:
    case kDmcMsgMotorHardStop:
      servos_.stopMotion();
      clearShoot();
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      sendMotorPositions(frame.id);
      break;
    case kDmcMsgMotorResetPosition: {
      uint8_t motor = 0;
      int32_t position = 0;
      if (frame.payload.size() != 5 || !readByte(frame.payload, 0, &motor) ||
          !readSignedDwordLE(frame.payload, 1, &position)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      (void)position;
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      if (servos_.pathActive()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrMoving);
        break;
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      sendMotorPositions(frame.id);
      break;
    }
    case kDmcMsgMotorJog: {
      uint8_t motor = 0;
      uint16_t speed = 0;
      int32_t destination = 0;
      if (frame.payload.size() != 7 || !readByte(frame.payload, 0, &motor) || !readWordLE(frame.payload, 1, &speed) ||
          !readSignedDwordLE(frame.payload, 3, &destination)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      servos_.slewTo(motor - 1, destination, speed == 0 ? 1 : speed);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgMotorConfigure: {
      uint8_t motor = 0;
      uint8_t flags = 0;
      if (frame.payload.size() < 2 || !readByte(frame.payload, 0, &motor) || !readByte(frame.payload, 1, &flags)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      servos_.configure(motor - 1, flags);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgMotorSetSpeed: {
      uint8_t motor = 0;
      int32_t maxVelocity = 0;
      int32_t maxAccel = 0;
      if (frame.payload.size() != 9 || !readByte(frame.payload, 0, &motor) ||
          !readSignedDwordLE(frame.payload, 1, &maxVelocity) || !readSignedDwordLE(frame.payload, 5, &maxAccel)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      (void)maxAccel;
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      servos_.setMaxSpeed(motor - 1, maxVelocity);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgMotorSetLimits: {
      uint8_t motor = 0;
      uint8_t lowerEnable = 0;
      uint8_t upperEnable = 0;
      uint8_t hwSet = 0;
      int32_t lower = 0;
      int32_t upper = 0;
      if (frame.payload.size() < 12 || !readByte(frame.payload, 0, &motor) ||
          !readByte(frame.payload, 1, &lowerEnable) || !readSignedDwordLE(frame.payload, 2, &lower) ||
          !readByte(frame.payload, 6, &upperEnable) || !readSignedDwordLE(frame.payload, 7, &upper) ||
          !readByte(frame.payload, 11, &hwSet)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      (void)hwSet;
      if (!motorIndexValid(motor)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      servos_.setLimits(motor - 1, lowerEnable != 0, lower, upperEnable != 0, upper);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgRtUploadBegin: {
      int32_t startFrame = 1;
      int32_t endFrame = 1;
      if (!readSignedDwordLE(frame.payload, 0, &startFrame) || !readSignedDwordLE(frame.payload, 4, &endFrame)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      path_.beginUpload(startFrame, endFrame, servos_.motorCount());
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgRtUploadAxis: {
      uint8_t motor = 0;
      uint32_t index = 0;
      if (frame.payload.size() < 5 || !readByte(frame.payload, 0, &motor) || !readDwordLE(frame.payload, 1, &index)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      const bool finalFill = (index & kDmcDmxFlagFinalSet) != 0;
      index &= ~kDmcDmxFlagFinalSet;
      const int n = static_cast<int>((frame.payload.size() - 5) / 4);
      std::vector<int32_t> values(static_cast<size_t>(n));
      for (int i = 0; i < n; ++i) {
        readSignedDwordLE(frame.payload, 5 + static_cast<size_t>(i) * 4, &values[static_cast<size_t>(i)]);
      }
      if (!path_.storeAxis(motor, index, values.data(), n, finalFill)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgRtUploadDmx: {
      uint16_t channel = 0;
      uint32_t index = 0;
      if (frame.payload.size() < 6 || !readWordLE(frame.payload, 0, &channel) || !readDwordLE(frame.payload, 2, &index)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      (void)index;
      if (channel < 1 || channel > kDmxChannels) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrRange);
        break;
      }
      if (frame.payload.size() == 7) {
        dmx_.apply(channel, frame.payload.data() + 6, 1, false);
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgRtUploadTriggers: {
      uint32_t mask = 0;
      if (!readDwordLE(frame.payload, 0, &mask)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      size_t off = 4;
      while (off + 8 <= frame.payload.size()) {
        uint32_t idx = 0;
        uint32_t val = 0;
        readDwordLE(frame.payload, off, &idx);
        readDwordLE(frame.payload, off + 4, &val);
        path_.storeTrigger(mask, idx, val);
        off += 8;
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgRtUploadEnd:
      if (!path_.finishUpload()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    case kDmcMsgRtPositionFrame: {
      int32_t frameNo = 0;
      if (!readSignedDwordLE(frame.payload, 0, &frameNo)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (path_.empty()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      servos_.moveToFramePose(frameNo);
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      lastPositionTxMs_ = millis();
      sendMotorPositions(frame.id);
      break;
    }
    case kDmcMsgRtRunMove: {
      uint32_t fpsX1000 = 24000;
      int32_t startFrame = 1;
      int32_t endFrame = 1;
      uint32_t prerollMs = 0;
      uint32_t postrollMs = 0;
      uint8_t syncDmx = 0;
      uint32_t bloopLoc = 0;
      uint16_t bloopDmx = 0;
      uint16_t bloopTime = 0;
      if (frame.payload.size() < 20 || !readDwordLE(frame.payload, 0, &fpsX1000) ||
          !readSignedDwordLE(frame.payload, 4, &startFrame) || !readSignedDwordLE(frame.payload, 8, &endFrame)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (frame.payload.size() >= 24) {
        readDwordLE(frame.payload, 12, &prerollMs);
      }
      if (frame.payload.size() >= 28) {
        readDwordLE(frame.payload, 16, &postrollMs);
      }
      if (frame.payload.size() >= 29) {
        readByte(frame.payload, 20, &syncDmx);
      }
      (void)syncDmx;
      if (frame.payload.size() >= 33) {
        readDwordLE(frame.payload, 21, &bloopLoc);
      }
      if (frame.payload.size() >= 35) {
        readWordLE(frame.payload, 25, &bloopDmx);
      }
      if (frame.payload.size() >= 37) {
        readWordLE(frame.payload, 27, &bloopTime);
      }
      if (path_.empty()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      servos_.setPathSliceUs(psFromFpsX1000(fpsX1000));
      servos_.moveToFramePose(startFrame);
      pendingStart_ = startFrame;
      pendingEnd_ = endFrame;
      pendingBloopMs_ = (bloopLoc != 0 || bloopTime != 0) ? (bloopTime == 0 ? 100 : bloopTime) : 0;
      pendingBloopDmx_ = bloopDmx;
      pendingPostrollMs_ = postrollMs;
      postrollWaiting_ = false;
      pendingPlayAtMs_ = millis() + prerollMs;
      pendingPlay_ = true;
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    case kDmcMsgRtShootFrame:
      handleShootFrame(frame);
      break;
    case kDmcMsgRtShootFrame2:
      handleShootFrame2(frame);
      break;
    case kDmcMsgRtGo:
      if (shootArmed_) {
        if (servos_.moving() || shootRun_) {
          sendDmcAck(frame.id, frame.type, kDmcAckErrNotInPosition);
          break;
        }
        shootT0_ = millis();
        for (int axis = 0; axis < servos_.motorCount(); ++axis) {
          if (shootBlur_[axis]) {
            servos_.startBlur(axis, shootEndSteps_[axis], shootDelayMs_, shootAccelMs_, shootCruiseMs_);
          }
        }
        shootArmed_ = false;
        shootRun_ = true;
        shootShutter_ = false;
        sendDmcAck(frame.id, frame.type, kDmcAckOk);
        break;
      }
      if (path_.empty()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      pendingStart_ = path_.startFrame();
      pendingEnd_ = path_.endFrame();
      pendingBloopMs_ = 0;
      pendingPlayAtMs_ = millis();
      pendingPlay_ = true;
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    case kDmcMsgRtJogAll: {
      uint32_t fpsX1000 = 24000;
      int32_t dest = 1;
      if (frame.payload.size() < 8 || !readDwordLE(frame.payload, 0, &fpsX1000) ||
          !readSignedDwordLE(frame.payload, 4, &dest)) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      if (path_.empty()) {
        sendDmcAck(frame.id, frame.type, kDmcAckErrGeneral);
        break;
      }
      int from = servos_.currentFrame();
      if (from < path_.startFrame()) {
        from = path_.startFrame();
      }
      servos_.setPathSliceUs(psFromFpsX1000(fpsX1000));
      pendingStart_ = from;
      pendingEnd_ = dest;
      pendingBloopMs_ = 0;
      pendingPlayAtMs_ = millis();
      pendingPlay_ = true;
      sendDmcAck(frame.id, frame.type, kDmcAckOk);
      break;
    }
    default:
      sendDmcAck(frame.id, frame.type, kDmcAckErrUnsupported);
      break;
  }
}

}  // namespace dfpwm
