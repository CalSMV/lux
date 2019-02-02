#ifndef __LUX_LOOKUP_TABLE_H
#define __LUX_LOOKUP_TABLE_H

static constexpr char gateData_[8] = {0, 49, 14, 52, 28, 13, 19, 0};

std::tuple<int, int, int> lookupValue(uint8_t position, uint8_t setPoint);

#endif//__LUX_LOOKUP_TABLE_H