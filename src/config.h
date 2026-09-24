#pragma once

// Raspberry Pi Pico pin map. USB CDC is binary DMC only — do not print debug text on Serial.

#include <Arduino.h>
#include <cstdint>

namespace dfpwm {

constexpr int kPwmPins = 16;
constexpr int kMaxUploadFrames = 2048;
constexpr int kDmxChannels = 512;

constexpr int32_t kServoCenterUs = 1500;
constexpr int32_t kServoMinSteps = -1000;
constexpr int32_t kServoMaxSteps = 1000;
constexpr uint32_t kServoHz = 100;
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
constexpr char kDmcHelloName[] = "DF PWM V1 (dmc-lite)";

constexpr uint32_t kPositionReportMs = 100;
constexpr uint32_t kLedBootOffMs = 500;
constexpr uint32_t kLedWaitHalfMs = 125;
constexpr uint32_t kLedReadyPeriodMs = 1000;
constexpr uint32_t kLedReadyPulseMs = 15;
constexpr uint32_t kLedPacketPulseMs = 40;

}  // namespace dfpwm
