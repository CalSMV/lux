#ifndef __LUX_CONTROLLER_H
#define __LUX_CONTROLLER_H

#include <tuple>
#include <chrono>

int BMSFaultLevel(std::tuple<int, int> BMSStat);
int TimeoutFaultLevel(std::chrono::system_clock::time_point reference);

//Needs: position, BMSStat, brake, accelerator
//Returns: Fault level, if any
int controllerUpdate(std::chrono::system_clock::time_point prevTime, std::chrono::system_clock::time_point currTime);

double getSetpoint();

#endif//__LUX_CONTROLLER_H