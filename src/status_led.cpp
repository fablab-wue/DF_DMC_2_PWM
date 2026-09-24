#include "status_led.h"

namespace dfpwm {

void StatusLed::begin() {
  pinMode(kStatusLedPin, OUTPUT);
  digitalWrite(kStatusLedPin, LOW);
  bootUntilMs_ = millis() + kLedBootOffMs;
  packetUntilMs_ = 0;
}

void StatusLed::markPacket() {
  const uint32_t now = millis();
  if (now < bootUntilMs_) {
    return;
  }
  packetUntilMs_ = now + kLedPacketPulseMs;
  digitalWrite(kStatusLedPin, HIGH);
}

void StatusLed::update(bool dfConnected) {
  const uint32_t now = millis();
  if (now < bootUntilMs_) {
    digitalWrite(kStatusLedPin, LOW);
    return;
  }
  if (now < packetUntilMs_) {
    digitalWrite(kStatusLedPin, HIGH);
    return;
  }
  if (!dfConnected) {
    digitalWrite(kStatusLedPin, ((now / kLedWaitHalfMs) & 1u) ? HIGH : LOW);
    return;
  }
  digitalWrite(kStatusLedPin, (now % kLedReadyPeriodMs) < kLedReadyPulseMs ? HIGH : LOW);
}

}  // namespace dfpwm
