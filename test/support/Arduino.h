/**
 * MIT License
 *
 * @brief Minimal Arduino/ESP32 stand-ins for PotIO host-side example syntax checks.
 *
 * @file Arduino.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-08-07
 * @copyright Copyright © 2026 Little Man Builds
 */
#pragma once

#include <cstdint>
#include <cstddef>

class PotIOSerialMock
{
public:
    void begin(unsigned long baud) { (void)baud; }
    explicit operator bool() const { return true; }
    int available() const { return 0; }
    int read() { return -1; }
    void print(const char *v) { (void)v; }
    void print(char v) { (void)v; }
    void print(int v) { (void)v; }
    void print(unsigned int v) { (void)v; }
    void print(long v) { (void)v; }
    void print(unsigned long v) { (void)v; }
    void print(float v, int digits = 2) { (void)v; (void)digits; }
    void println() {}
    void println(const char *v) { (void)v; }
    void println(int v) { (void)v; }
    void println(unsigned int v) { (void)v; }
    void println(long v) { (void)v; }
    void println(unsigned long v) { (void)v; }
    void println(float v, int digits = 2) { (void)v; (void)digits; }
};

extern PotIOSerialMock Serial;

inline void delay(unsigned long ms) { (void)ms; }
inline std::uint32_t millis() { return 0u; }
inline int analogRead(std::uint8_t pin) { (void)pin; return 2048; }
inline std::uint32_t analogReadMilliVolts(std::uint8_t pin) { (void)pin; return 1650u; }
