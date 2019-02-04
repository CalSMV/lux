#ifndef __LUX_FET_WRITER_H
#define __LUX_FET_WRITER_H

#include "constants.hh"
#include <mutex>

void setMOSFET(MOSFET mosfet_ID, int value);

void setPWM(DURATION_TYPE loopTime, short dutyCycle);

#endif//__LUX_FET_WRITER_H