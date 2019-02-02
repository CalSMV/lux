#ifndef __LUX_CONSTANTS_H
#define __LUX_CONSTANTS_H

#include <chrono>

constexpr short MAX_SHORT = 32767;
constexpr double MAX_SHORT_D = 32767;
inline double percent(const short pVal) {
  return static_cast<double>(pVal) / MAX_SHORT_D;
}

using DURATION_TYPE = std::chrono::duration<uint64_t, std::milli>;
constexpr DURATION_TYPE ZERO_TIME(0);

constexpr DURATION_TYPE LOOP_TIME(100000);

constexpr DURATION_TYPE BRAKE_TIMEOUT(1000);
constexpr int BRAKE_FAULT_LEVEL = 2;
constexpr DURATION_TYPE ACCEL_TIMEOUT = ZERO_TIME;
constexpr int ACCEL_FAULT_LEVEL = 1;
constexpr DURATION_TYPE BMS_TIMEOUT(500);
constexpr int BMS_FAULT_LEVEL = 2;
constexpr DURATION_TYPE HALL_TIMEOUT(500);
constexpr int HALL_FAULT_LEVEL = 2;

constexpr int BREAK_TEMP = 0; //Placeholder
constexpr int SHUTDOWN_TEMP = 0; //Placeholder
constexpr int MIN_VOLTAGE = 0; //Placeholder

enum PIN {
};

enum MOSFET {
  SWITCH_A = 0,
  SWITCH_B = 1,
  SWITCH_C = 2,
  PWM = 3,
  ON = 1,
  OFF = 0,
  REVERSE = -1,
};


#endif//__LUX_CONSTANTS_H