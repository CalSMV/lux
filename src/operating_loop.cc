#include "operating_loop.h"

#include <thread>
#include <tuple>

#include "accelerator_reader.h"
#include "battery_reader.h"
#include "brake_reader.h"
#include "can_interface.h"
#include "controller.h"
#include "observer.h"
#include "hall_reader.h"
#include "fet_writer.h"
#include "lookup_table.h"
#include "constants.h"

bool operatingLoop(std::chrono::system_clock::time_point prevTime, std::chrono::system_clock::time_point currTime) {
  CANUpdate();
  BMSUpdate();
  brakeUpdate();
  accelUpdate();
  hallUpdate();

  observerUpdate(prevTime, currTime, getHallValues()); //TODO fix
  if(controllerUpdate(prevTime, currTime) == 2) {
    return false;
  }

  //Cycle PWM

  std::tuple<int, int, int> phaseValues = lookupValue(getPosition(), getSetpoint());

  setMOSFET(MOSFET::SWITCH_A, std::get<0>(phaseValues));
  setMOSFET(MOSFET::SWITCH_B, std::get<1>(phaseValues));
  setMOSFET(MOSFET::SWITCH_C, std::get<2>(phaseValues));

  if(std::chrono::duration_cast<DURATION_TYPE>(std::chrono::system_clock::now() - currTime) < LOOP_TIME) {
    //We need some sort of threading solution...
    //std::this_thread::sleep_for(std::chrono::duration_cast<DURATION_TYPE>(std::chrono::system_clock::now() - currTime));
  }
}