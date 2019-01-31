#ifndef __LUX_OBSERVER_H
#define __LUX_OBSERVER_H

void observerUpdate(std::chrono::system_clock::time_point prevTime, std::chrono::system_clock::time_point currTime,
                      std::tuple<bool, bool, bool> hallValues);

uint8_t getPosition();

#endif//__LUX_OBSERVER_H