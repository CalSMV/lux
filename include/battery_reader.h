#ifndef __LUX_BATTERY_READER_H
#define __LUX_BATTERY_READER_H

#include <tuple>
#include <chrono>

void BMSUpdate();

std::tuple<int, int> getBMSValues();
short getBMSThrottle();

std::chrono::system_clock::time_point getBMSTimestamp();

#endif//__LUX_BATTERY_READER_H