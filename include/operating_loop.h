#ifndef __LUX_OPERATING_LOOP_H
#define __LUX_OPERATING_LOOP_H

#include <chrono>
#include <cstdint>

using DURATION_TYPE = std::chrono::duration<uint64_t, std::milli>;
constexpr DURATION_TYPE LOOP_TIME(100000);

bool operatingLoop(std::chrono::system_clock::time_point prevTime, std::chrono::system_clock::time_point nextTime);

#endif//__LUX_OPERATING_LOOP_H