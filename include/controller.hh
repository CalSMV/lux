#ifndef __LUX_CONTROLLER_H
#define __LUX_CONTROLLER_H

#include <tuple>
#include <chrono>

int BMSFaultLevel(std::tuple<int, int> BMSStat);
int TimeoutFaultLevel(std::chrono::system_clock::time_point reference);

//Needs: position, BMSStat, brake, accelerator
int controllerUpdate(std::chrono::system_clock::time_point prevTime, std::chrono::system_clock::time_point currTime);

int getSetpoint();

#endif//__LUX_CONTROLLER_H