#include "lookup_table.hh"

inline int getIntValue(const std::tuple<bool, bool, bool>& hallValues) {
  return   (std::get<2>(hallValues) << 2)
         + (std::get<1>(hallValues) << 1)
         + (std::get<0>(hallValues) << 0);
}

std::tuple<int, int, int> lookupValue(std::tuple<bool, bool, bool> hallValues) {
  unsigned char gateValues = gateData_[getIntValue(hallValues)];
  return std::make_tuple<int, int, int>(
    (((gateValues >> 0) & 0x01) - (((gateValues >> 1) & 0x01)) * 2),
    (((gateValues >> 2) & 0x01) - (((gateValues >> 3) & 0x01)) * 2),
    (((gateValues >> 4) & 0x01) - (((gateValues >> 5) & 0x01)) * 2)
  );
}