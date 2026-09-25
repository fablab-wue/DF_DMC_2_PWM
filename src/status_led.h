#pragma once

#include "config.h"

namespace dfpwm {

class StatusLed {
 public:
  void begin();
  void markPacket();
  void update(bool dfConnected);

 private:
  uint32_t bootUntilMs_ = 0;
  uint32_t packetUntilMs_ = 0;
  uint32_t readyAnchorMs_ = 0;
};

}  // namespace dfpwm
