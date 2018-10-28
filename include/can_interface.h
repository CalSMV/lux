#ifndef __CAN_INTERFACE_H
#define __CAN_INTERFACE_H

#include <ctime>

/* Tasks for this segment:
 * 1. Read input from hardware ports into software (HAL)
 * 2. Recieve CAN information (? CAN Library)
 * 3. Decode CAN messages into useable signals (Figure out what we want our signals to look like)
 */

struct CAN_msg {
    unsigned int can_id;
    time_t time;
    uint8_t msg[8];
};

CAN_msg buffer[]; //Indexed by CAN id

CAN_msg readBuffer(unsigned int can_id);

#endif//__CAN_INTERFACE_H
