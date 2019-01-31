#ifndef __LUX_ACCELERATOR_READER_H
#define __LUX_ACCELERATOR_READER_H

#include <chrono>

void accelUpdate();

uint8_t getAccelValue();

std::chrono::system_clock::time_point getAccelTimestamp();

#endif//__LUX_ACCELERATOR_READER_H