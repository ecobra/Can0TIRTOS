/*
 * BMS_CAN.h
 *
 *  Created on: Apr 30, 2014
 *      Author: TRex
 */

#ifndef BMS_CAN_H_
#define BMS_CAN_H_

#include <stdint.h>

void InitCan (void);
void CANIntHandler(void);
int mainCan(void);

typedef struct
{
    //
    //! The CAN message identifier used for 11 or 29 bit identifiers.
    //
    uint32_t MsgID;

    //
    //! This value is the number of bytes of data in the message object.
    //
    uint8_t MsgLen;
    uint16_t TimeStamp;

    //
    //! This is a pointer to the message object's data.
    //
    uint8_t MsgData[8];
}
tCANMsg;

#define MaxCanBuffer 255
typedef struct
{
	tCANMsg Buffer[MaxCanBuffer];
    uint8_t BufferSize;
}
tCANMsgBuffer;

extern volatile tCANMsgBuffer CANMsgBuffer;
//void SendCanMessage (uint32_t CanID,  )



#endif /* BMS_CAN_H_ */
