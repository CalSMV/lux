#include <iostream>
#include <list>
#include <ctime>
#include <chrono>
#include <cstdlib>

#include "operating_loop.hh"


int main(int argc, char** argv){
    auto zeroTime = std::chrono::system_clock::now();
    auto currTime = std::chrono::system_clock::now();
    auto prevTime = std::chrono::system_clock::now();
    //TODO: Initialize STM32F7
    //TODO: Verify STM32F7 port/hardware integrity
    //TODO: ? Initialize CAN Bus

    bool cont = true;
    while(cont) {
      //Check integrity of STM32F7: <1ms?
      prevTime = currTime;
      currTime = std::chrono::system_clock::now();

      cont = operatingLoop(prevTime, currTime);
    }

    //TODO: Close down STM32 nicely
    return 0;
}
