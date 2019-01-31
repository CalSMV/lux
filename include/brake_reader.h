#ifndef __LUX_BRAKE_READER_H
#define __LUX_BRAKE_READER_H

#include <chrono>

static bool brakeValue_;

void brakeUpdate();

bool getBrakeValue();

std::chrono::system_clock::time_point getBrakeTimestamp();

#endif//__LUX_BRAKE_READER_H