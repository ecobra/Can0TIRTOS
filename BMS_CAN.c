/*
 * BMS_CAN.c
 *
 *  Created on: Apr 30, 2014
 *      Author: TRex
 */

//*****************************************************************************
//
// multi_tx.c - Peripheral example demonstrating multiple CAN message
//              transmission.
//
// Copyright (c) 2010-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.1.0.12573 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/can.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "BMS_CAN.h"

volatile tCANMsgBuffer CANMsgBuffer;

//*****************************************************************************
//
//! \addtogroup can_examples_list
//! <h1>Multiple CAN TX (multi_tx)</h1>
//!
//! This example shows how to set up the CAN to send multiple messages.  The
//! CAN peripheral is configured to send messages with 4 different CAN IDs.
//! Two of the messages (with different CAN IDs) are sent using a shared
//! message object.  This shows how to reuse a message object for multiple
//! messages.  The other two messages are sent using their own message objects.
//! All four messages are transmitted once per second.  The content of each
//! message is a test pattern.  A CAN interrupt handler is used to confirm
//! message transmission and count the number of messages that have been sent.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - CAN0 peripheral
//! - GPIO Port B peripheral (for CAN0 pins)
//! - CAN0RX - PB4
//! - CAN0TX - PB5
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of CAN.
//! - GPIO port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - INT_CAN0 - CANIntHandler
//
//*****************************************************************************

//*****************************************************************************
//
// A counter that keeps track of the number of times the TX interrupt has
// occurred, which should match the number of TX messages that were sent.
//
//*****************************************************************************
volatile uint32_t g_ui32IntCount = 0, g_ui32MsgCount = 0;

//*****************************************************************************
//
// Counters that are used to count the number of messages on each of the
// three message objects that are used in this example.
//
//*****************************************************************************
volatile uint32_t g_ui32Msg1Count = 0;
volatile uint32_t g_ui32Msg2Count = 0;
volatile uint32_t g_ui32Msg3Count = 0;

//*****************************************************************************
//
// A flag to indicate that CAN controller message object 3 has sent a message.
//
//*****************************************************************************
volatile bool g_bMsgObj3Sent = 0;

volatile bool g_bRXFlag1 = 0;
volatile bool g_bRXFlag2 = 0;
volatile bool g_bRXFlag3 = 0;

//*****************************************************************************
//
// A flag to indicate that some transmission error occurred.
//
//*****************************************************************************
volatile bool g_bErrFlag = 0;

//*****************************************************************************
//
// CAN message objects that will hold the separate CAN messages.  These could
// also be allocated on the stack but be careful because these structures
// each take about 20 bytes.
//
//*****************************************************************************
tCANMsgObject g_sCANMsgObject1;
tCANMsgObject g_sCANMsgObject2;
tCANMsgObject g_sCANMsgObject3;
tCANMsgObject g_sCANMsgObject4;

tCANMsgObject sCANMessage;

uint8_t pui8MsgData[8];
//*****************************************************************************
//
// Message buffers that hold the contents of the 4 different messages that
// are being transmitted.  Each one is a different length.
//
//*****************************************************************************
uint8_t g_pui8Msg1[4] = { 0x40, 0x00, 0x01, 0x22 };
uint8_t g_pui8Msg2[4] = { 0x41, 0x00, 0x00, 0x96 };
uint8_t g_pui8Msg3[4] = { 0x42, 0x00, 0x0B, 0xB8 };
uint8_t g_pui8MsgOn[4] = { 0x44, 0x00, 0x01, 0x04 };
uint8_t g_pui8MsgOff[4] = { 0x44, 0x00, 0x00, 0x04 };

//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************
void
InitConsole(void)
{
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
    //
    // Enable the UART interrupt.
    //
    IntEnable(INT_UART3);
    UARTIntEnable(UART3_BASE, UART_INT_RX | UART_INT_RT);
}

//*****************************************************************************
//
// This function prints some information about the CAN message to the
// serial port for information purposes only.
//
//*****************************************************************************
void
PrintCANMessageInfo(tCANMsgObject *psCANMsg, uint32_t ui32MsgObj)
{
	//todo
	return;
    unsigned int uIdx;

//    UARTprintf("Sending msg: obj=%d ID=0x%04X msg=0x", ui32MsgObj,
//               psCANMsg->ui32MsgID);
    for(uIdx = 0; uIdx < psCANMsg->ui32MsgLen; uIdx++)
    {
//        UARTprintf("%02X ", psCANMsg->pui8MsgData[uIdx]);
    }
//    UARTprintf("\n");
}
void
PrintCANMessageRXInfo(tCANMsgObject *psCANMsg, uint32_t ui32MsgObj)
{
	//todo
	return;
    unsigned int uIdx;

//    UARTprintf("Receiving msg: obj=%d ID=0x%04X msg=0x", ui32MsgObj,
//               psCANMsg->ui32MsgID);
    for(uIdx = 0; uIdx < psCANMsg->ui32MsgLen; uIdx++)
    {
//        UARTprintf("%02X ", psCANMsg->pui8MsgData[uIdx]);
    }
//    UARTprintf("\n");
}
//*****************************************************************************
//
// This function provides a 1 second delay using a simple polling method.
//
//*****************************************************************************
void
SimpleDelay(void)
{
    //
    // Delay cycles for 1 second
    //
    SysCtlDelay(16000000 / 3);
}

//*****************************************************************************
//
// This function is the interrupt handler for the CAN peripheral.  It checks
// for the cause of the interrupt, and maintains a count of all messages that
// have been transmitted.
//
//*****************************************************************************
void
CANIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.  If the
        // CAN peripheral is not connected to a CAN bus with other CAN devices
        // present, then errors will occur and will be indicated in the
        // controller status.
        //
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        //
        // Set a flag to indicate some errors may have occurred.
        //
        g_bErrFlag = 1;
    }

    //
    // Check if the cause is message object 1, which is used for sending
    // message 1.
    //
    else if(ui32Status == 1)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 1);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ui32Msg1Count++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 2, which is used for sending
    // message 2.
    //
    else if(ui32Status == 2)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 2, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 2);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ui32Msg2Count++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 3, which is used for sending
    // messages 3 and 4.
    //
    else if(ui32Status == 3)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 3, and a message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 3);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ui32Msg3Count++;

        //
        // Set the flag indicating that a message was sent using message
        // object 3.  The program main loop uses this to know when to send
        // another message using message object 3.
        //
        g_bMsgObj3Sent = 1;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlag = 0;
    }
    //
    // Check if the cause is message object 1.
    //
    else if(ui32Status == 4)
    {
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message reception is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 4);

        //
        // Increment a counter to keep track of how many messages have been
        // received.  In a real application this could be used to set flags to
        // indicate when a message is received.
        //
        g_ui32MsgCount++;

        //
        // Set flag to indicate received message is pending for this message
        // object.
        //
        g_bRXFlag1 = 1;

        //
        // Since a message was received, clear any error flags.
        //
        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 2.
    //
    else if(ui32Status == 5)
    {
        CANIntClear(CAN0_BASE, 5);
        g_ui32MsgCount++;
        g_bRXFlag2 = 1;
        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 3.
    //
    else if(ui32Status == 6)
    {
        CANIntClear(CAN0_BASE, 6);
        g_ui32MsgCount++;
        g_bRXFlag3 = 1;
        g_bErrFlag = 0;
    }
    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }
}

void InitCan (void)
{
    //
    // Initialize the CAN controller
    //
    CANInit(CAN0_BASE);

    //
    // Set up the bit rate for the CAN bus.  This function sets up the CAN
    // bus timing for a nominal configuration.  You can achieve more control
    // over the CAN bus timing by using the function CANBitTimingSet() instead
    // of this one, if needed.
    // In this example, the CAN bus is set to 500 kHz.  In the function below,
    // the call to SysCtlClockGet() is used to determine the clock rate that
    // is used for clocking the CAN peripheral.  This can be replaced with a
    // fixed value if you know the value of the system clock, saving the extra
    // function call.  For some parts, the CAN peripheral is clocked by a fixed
    // 8 MHz regardless of the system clock in which case the call to
    // SysCtlClockGet() should be replaced with 8000000.  Consult the data
    // sheet for more information about CAN peripheral clocking.
    //
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 250000);

    //
    // Enable interrupts on the CAN peripheral.  This example uses static
    // allocation of interrupt handlers which means the name of the handler
    // is in the vector table of startup code.  If you want to use dynamic
    // allocation of the vector table, then you must also call CANIntRegister()
    // here.
    //
    // CANIntRegister(CAN0_BASE, CANIntHandler); // if using dynamic vectors
    //
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    //
    // Enable the CAN interrupt on the processor (NVIC).
    //
    IntEnable(INT_CAN0);

    //
    // Enable the CAN for operation.
    //
    CANEnable(CAN0_BASE);



    //
    // Initialize a message object to receive CAN messages with ID 0x1001.
    // The expected ID must be set along with the mask to indicate that all
    // bits in the ID must match.
    //
    sCANMessage.ui32MsgID = 0x1001;
    sCANMessage.ui32MsgIDMask = 0xfffff;
    sCANMessage.ui32Flags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER |
                             MSG_OBJ_EXTENDED_ID);
    sCANMessage.ui32MsgLen = 8;
    //
    // Now load the message object into the CAN peripheral message object 1.
    // Once loaded the CAN will receive any messages with this CAN ID into
    // this message object, and an interrupt will occur.
    //
    CANMessageSet(CAN0_BASE, 4, &sCANMessage, MSG_OBJ_TYPE_RX);


    //
    // Initialize message object 1 to be able to send CAN message 1.  This
    // message object is not shared so it only needs to be initialized one
    // time, and can be used for repeatedly sending the same message ID.
    //
    g_sCANMsgObject1.ui32MsgID = 0x271;
    g_sCANMsgObject1.ui32MsgIDMask = 0;
    g_sCANMsgObject1.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    g_sCANMsgObject1.ui32MsgLen = sizeof(g_pui8Msg1);
    g_sCANMsgObject1.pui8MsgData = g_pui8Msg1;

}

//*****************************************************************************
//
// Configure the CAN and enter a loop to transmit periodic CAN messages.
//
//*****************************************************************************
int mainCan(void)
{
	static uint8_t CanState = 0;
	uint8_t i = 0;


    //
    // Enter loop to send messages.  Four messages will be sent once per
    // second.  The contents of each message will be changed each time.
    //
    {
        if(g_bRXFlag1)
        {
            //
            // Reuse the same message object that was used earlier to configure
            // the CAN for receiving messages.  A buffer for storing the
            // received data must also be provided, so set the buffer pointer
            // within the message object.  This same buffer is used for all
            // messages in this example, but your application could set a
            // different buffer each time a message is read in order to store
            // different messages in different buffers.
            //
            sCANMessage.pui8MsgData = pui8MsgData;

            //
            // Read the message from the CAN.  Message object number 1 is used
            // (which is not the same thing as CAN ID).  The interrupt clearing
            // flag is not set because this interrupt was already cleared in
            // the interrupt handler.
            //
            CANMessageGet(CAN0_BASE, 4, &sCANMessage, 0);

            // todo something with received message here

            //
            // Clear the pending message flag so that the interrupt handler can
            // set it again when the next message arrives.
            //
            g_bRXFlag1 = 0;

            //
            // Print information about the message just received.
            //
            PrintCANMessageRXInfo(&sCANMessage, 4);
        }
		//
		// Send message 1 using CAN controller message object 1.  This is
		// the only message sent using this message object.  The
		// CANMessageSet() function will cause the message to be sent right
		// away.
		//
		g_sCANMsgObject1.ui32MsgID = 0x271;
		g_sCANMsgObject1.ui32MsgIDMask = 0;
		g_sCANMsgObject1.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
		g_sCANMsgObject1.ui32MsgLen = sizeof(g_pui8Msg1);
		g_sCANMsgObject1.pui8MsgData = g_pui8Msg1;
		PrintCANMessageInfo(&g_sCANMsgObject1, 1);
		CANMessageSet(CAN0_BASE, 1, &g_sCANMsgObject1, MSG_OBJ_TYPE_TX);
        //
        // Check the error flag to see if errors occurred
        //
        if(g_bErrFlag)
        {
        	//todo
//            UARTprintf(" error - cable connected?\n");
        }
        else
        {
            //
            // If no errors then print the count of message sent
            //
        	//todo
//            UARTprintf(" total count = %u\n", g_ui32Msg1Count + g_ui32Msg2Count + g_ui32Msg3Count);
        }

    }

    //
    // Return no errors
    //
    return(0);
}



