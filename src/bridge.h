#pragma once

#include "config.h"
#include "servo_bank.h"
#include "status_led.h"

#include <dmc_protocol.h>
#include <dmx_engine.h>
#include <gio_io.h>
#include <path_table.h>

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
  void sendMotorPositions(uint32_t id);
  void sendGioIn(uint32_t id);
  void maybeSendBootHello();
  void maybeSendPositionReport();
  void maybeUnsolicitedGio();
  void maybeFinishPath();
  void fireBloop(unsigned ms);
  void applyFrameTrigger(int dfFrame);
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
  bool wasPathActive_ = false;
  int pendingStart_ = 1;
  int pendingEnd_ = 1;
  unsigned pendingBloopMs_ = 0;
};

}  // namespace dfpwm
