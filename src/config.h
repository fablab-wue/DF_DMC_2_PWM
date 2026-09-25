#pragma once

// Raspberry Pi Pico pin map. USB CDC is binary DMC only — do not print debug text on Serial.

#include <Arduino.h>
#include <cstdint>

namespace dfpwm {

constexpr int kPwmPins = 16;
constexpr int kMaxUploadFrames = 2048;
constexpr int kDmxChannels = 512;

// 133 MHz / 6.625 / 65536 = 306.35 Hz. One count is 49.812 ns.
// 20000 counts = 0.996 ms, so ±20000 around 30113 is 504 µs..2497 µs.
constexpr float kServoClkDiv = 6.625f;
constexpr uint16_t kServoWrap = 65535;
constexpr int32_t kServoPwmZero = 30113;
constexpr int32_t kServoPwmMin = 10113;
constexpr int32_t kServoPwmMax = 50113;
constexpr int32_t kServoMinSteps = -20000;
constexpr int32_t kServoMaxSteps = 20000;
constexpr uint32_t kDmxPwmHz = 18000;

constexpr uint8_t kSwPins[4] = {18, 19, 20, 21};
constexpr uint8_t kDmxTxPin = 16;
constexpr uint8_t kCameraPin = 17;
constexpr uint8_t kBuzzerPin = 22;
constexpr uint8_t kStatusLedPin = 25;
constexpr uint8_t kGioOutPins[2] = {26, 27};
constexpr uint8_t kGioInPin = 28;

constexpr uint8_t kHelloVersionMajor = 1;
constexpr uint8_t kHelloVersionMinor = 0;
constexpr uint8_t kHelloVersionRev = 0;
constexpr uint16_t kHelloProtocolVersion = 2;

constexpr uint32_t kPositionReportMs = 100;
constexpr uint32_t kLedBootOffMs = 500;
constexpr uint32_t kLedWaitHalfMs = 125;
constexpr uint32_t kLedReadyPeriodMs = 1000;
constexpr uint32_t kLedReadyPulseMs = 15;
constexpr uint32_t kLedPacketPulseMs = 40;

}  // namespace dfpwm
