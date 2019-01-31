#ifndef __LUX_HALL_READER_H
#define __LUX_HALL_READER_H

#include <chrono>

void hallUpdate();

std::tuple<short, short, short> getHallValues();

std::chrono::system_clock::time_point getHallTimestamp();

#endif//__LUX_HALL_READER_H