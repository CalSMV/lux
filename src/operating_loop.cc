#include "operating_loop.hh"

#include <thread>
#include <tuple>

#include "accelerator_reader.hh"
#include "battery_reader.hh"
#include "brake_reader.hh"
#include "can_interface.hh"
#include "controller.hh"
#include "observer.hh"
#include "hall_reader.hh"
#include "fet_writer.hh"
#include "lookup_table.hh"
#include "constants.hh"

bool switchingLoop(std::chrono::system_clock::time_point prevTime, std::chrono::system_clock::time_point currTime) {

  short dutyCycle = getAccelValue();

  DURATION_TYPE uptime = std::chrono::duration_cast<DURATION_TYPE>(PWML_TIME * percent(dutyCycle));
  auto onTime = currTime + uptime;
  auto endTime = currTime + PWML_TIME;
  setMOSFET(MOSFET::PWM, 1);
  std::this_thread::sleep_for(uptime);
  setMOSFET(MOSFET::PWM, 0);
  std::this_thread::sleep_until(endTime);

  return 0;
}

bool operatingLoop(std::chrono::system_clock::time_point prevTime, std::chrono::system_clock::time_point currTime) {
  CANUpdate();
  BMSUpdate();
  brakeUpdate();
  accelUpdate();
  hallUpdate();

  std::tuple<int, int, int> phaseValues = lookupValue(getHallValues());

  setMOSFET(MOSFET::SWITCH_A, std::get<0>(phaseValues));
  setMOSFET(MOSFET::SWITCH_B, std::get<1>(phaseValues));
  setMOSFET(MOSFET::SWITCH_C, std::get<2>(phaseValues));

  if(std::chrono::duration_cast<DURATION_TYPE>(std::chrono::system_clock::now() - currTime) < LOOP_TIME) {
    //We need some sort of threading solution...
    std::this_thread::sleep_for(std::chrono::duration_cast<DURATION_TYPE>(std::chrono::system_clock::now() - currTime));
  }
}