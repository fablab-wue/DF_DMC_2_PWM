#pragma once

#include "config.h"
#include "servo_bank.h"
#include "status_led.h"

#include <dmc_protocol.h>
#include <dmx_engine.h>
#include <gio_io.h>
#include <path_table.h>
#include <rt_support.h>

namespace dfpwm {

struct BoardMode {
  int motorCount = 0;
  int dmxPwmCount = 16;
  bool exponential = false;
};

class DmcBridge {
 public:
  void setup();
  void loop();

 private:
  BoardMode readDip() const;
  bool motorIndexValid(uint8_t motor) const;
  void sendDmcFrame(uint32_t id, uint16_t type, const std::vector<uint8_t>& payload);
  void sendDmcAck(uint32_t id, uint16_t type, uint32_t status);
  void sendDmcHello(uint32_t id);
  void sendMotorStatus(uint32_t id);
  void sendMotorPositions(uint32_t id, int32_t frameTime = 0);
  void sendGioIn(uint32_t id);
  void maybeSendBootHello();
  void maybeSendPositionReport();
  void maybeUnsolicitedGio();
  void maybeFinishPath();
  void maybeUpdateShoot();
  void endShoot(bool notify);
  void noteCdc(bool up);
  void moveToSample(double frameTime);
  uint32_t poseFault(double frameTime, bool extrapolate) const;
  bool rejectRunLimits(const dfdmc::RtRunMove& move, const dfdmc::RtPlaySpan& span, uint32_t id);
  bool inRun(int frame) const;
  void handleShootFrame(const dfdmc::DmcFrame& frame);
  void handleShootFrame2(const dfdmc::DmcFrame& frame);
  void applyFrameTrigger(int dfFrame);
  void applyProgramDmx(int dfFrame);
  void followPathOutputs();
  void pumpPendingPlay();
  uint32_t psFromFpsX1000(uint32_t fpsX1000) const;
  void handleDmcFrame(const dfdmc::DmcFrame& frame);
  void handleMotorOrRt(const dfdmc::DmcFrame& frame);

  dfdmc::DmcParser dmcParser_;
  dfdmc::DmcGio gio_;
  dfdmc::DmxEngine dmx_;
  dfdmc::PathTable path_;
  ServoBank servos_;
  StatusLed statusLed_;
  BoardMode mode_{};
  uint8_t dmxPwmPins_[kPwmPins]{};
  uint32_t lastPositionTxMs_ = 0;
  uint32_t pendingPlayAtMs_ = 0;
  bool dfConnected_ = false;
  bool bootHelloSent_ = false;
  bool pendingPlay_ = false;
  bool armedPlay_ = false;
  bool syncDmx_ = false;
  int reportedFrame_ = -1;
  bool wasPathActive_ = false;
  int pendingStart_ = 1;
  int pendingEnd_ = 1;
  int runStart_ = 1;
  int runEnd_ = 1;
  dfdmc::BloopOut bloop_;
  dfdmc::LiveShutter shutter_;
  uint32_t lastStopAllMs_ = 0;
  bool cdcWasUp_ = false;
  bool shootNeedsEnd_ = false;
  bool shootArmed_ = false;
  bool shootRun_ = false;
  bool shootShutter_ = false;
  uint32_t shootT0_ = 0;
  uint32_t shootDelayMs_ = 0;
  uint32_t shootAccelMs_ = 0;
  uint32_t shootCruiseMs_ = 0;
  uint32_t shootShutterOpenMs_ = 0;
  uint32_t shootShutterCloseMs_ = 0;
  uint32_t shootDoneMs_ = 0;
  int32_t shootEndSteps_[kPwmPins]{};
  bool shootBlur_[kPwmPins]{};
};

}  // namespace dfpwm
