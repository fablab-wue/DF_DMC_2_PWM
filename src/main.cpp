#include "bridge.h"

static_assert(dfdmc::kMaxAxes == 16, "DFDMC_MAX_AXES");

namespace {
dfpwm::DmcBridge bridge;
}

void setup() { bridge.setup(); }

void loop() { bridge.loop(); }
