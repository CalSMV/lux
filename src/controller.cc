#include "controller.hh"

#include <algorithm>
#include <optional>

#include "accelerator_reader.hh"
#include "brake_reader.hh"
#include "hall_reader.hh"
#include "observer.hh"
#include "lookup_table.hh"
#include "battery_reader.hh"
#include "constants.h"

//TODO: Make optional?
double setPoint_;
double getSetpoint() {
  return setPoint_;
}

int BMSFaultLevel(std::tuple<int, int> BMSStat) {
  int temp = std::get<0>(BMSStat);
  int volt = std::get<1>(BMSStat);

  if(temp >= SHUTDOWN_TEMP || volt < MIN_VOLTAGE) {
    return 2;
  } else if (temp >= BREAK_TEMP) {
    return 1;
  } else {
    return 0;
  }
}

int TimeoutFaultLevel(std::chrono::system_clock::time_point reference) {
  int timeout_fault = 0;
  if(BRAKE_TIMEOUT != ZERO_TIME) {
    if(std::chrono::duration_cast<DURATION_TYPE>(getBrakeTimestamp() - reference) >= BRAKE_TIMEOUT) {
      timeout_fault = std::max(timeout_fault, BRAKE_FAULT_LEVEL);
    }
  }
  if(ACCEL_TIMEOUT != ZERO_TIME) {
    if(std::chrono::duration_cast<DURATION_TYPE>(getAccelTimestamp() - reference) >= ACCEL_TIMEOUT) {
      timeout_fault = std::max(timeout_fault, ACCEL_FAULT_LEVEL);
    }
  }
  if(BMS_TIMEOUT != ZERO_TIME) {
    if(std::chrono::duration_cast<DURATION_TYPE>(getBMSTimestamp() - reference) >= BMS_TIMEOUT) {
      timeout_fault = std::max(timeout_fault, BMS_FAULT_LEVEL);
    }
  }
  if(HALL_TIMEOUT != ZERO_TIME) {
    if(std::chrono::duration_cast<DURATION_TYPE>(getHallTimestamp() - reference) >= HALL_TIMEOUT) {
      timeout_fault = std::max(timeout_fault, HALL_FAULT_LEVEL);
    }
  }
}

int controllerUpdate(std::chrono::system_clock::time_point prevTime, std::chrono::system_clock::time_point currTime) {
  uint8_t accel = getAccelValue();
  bool brake = getBrakeValue();
  uint8_t position = getPosition();

  int faultLevel = std::max(BMSFaultLevel(getBMSValues()), TimeoutFaultLevel(std::chrono::system_clock::now()));
  if(faultLevel) {
    setPoint_ = 0;
    return faultLevel;
  } else {
    if(brake) {
      setPoint_ = 0;
    } else {
      setPoint_ = accel * percent(getBMSThrottle());
    }
  }
}