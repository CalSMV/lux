#include <iostream>
#include <list>
#include <ctime>
#include <chrono>
#include <cstdlib>
#include <thread>

#include "operating_loop.hh"


void loopPWM(std::chrono::time_point<std::chrono::system_clock> prevTime) {
  auto currTime = std::chrono::system_clock::now();

  //No error handling, no errors we could have, really
  while(true) {
    prevTime = currTime;
    currTime = std::chrono::system_clock::now();

    switchingLoop(prevTime, currTime);
  }
}
void loopElse(std::chrono::time_point<std::chrono::system_clock> prevTime) {
  auto currTime = std::chrono::system_clock::now();
  bool cont = true;
  while(cont) {
    //Check integrity of STM32F7: <1ms?
    prevTime = currTime;
    currTime = std::chrono::system_clock::now();

    cont = operatingLoop(prevTime, currTime);
  }
}
int main(int argc, char** argv){
    auto zeroTime = std::chrono::system_clock::now();
    //TODO: Initialize STM32F7
    //TODO: Verify STM32F7 port/hardware integrity
    //TODO: ? Initialize CAN Bus

    std::thread threadMain(loopElse, zeroTime);
    std::thread threadPWM(loopElse, zeroTime);

    threadMain.join();

    //TODO: Close down STM32 nicely
    return 0;
}
