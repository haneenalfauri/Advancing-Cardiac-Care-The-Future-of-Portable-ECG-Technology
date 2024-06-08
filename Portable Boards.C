/* 

 * Copyright (c) 2019, Texas Instruments Incorporated

 * All rights reserved.

 *

 * Redistribution and use in source and binary forms, with or without

 * modification, are permitted provided that the following conditions

 * are met:

 *

 * *  Redistributions of source code must retain the above copyright

 *    notice, this list of conditions and the following disclaimer.

 *

 * *  Redistributions in binary form must reproduce the above copyright

 *    notice, this list of conditions and the following disclaimer in the

 *    documentation and/or other materials provided with the distribution.

 *

 * *  Neither the name of Texas Instruments Incorporated nor the names of

 *    its contributors may be used to endorse or promote products derived

 *    from this software without specific prior written permission.

 *

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"

 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,

 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR

 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR

 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,

 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,

 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;

 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,

 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR

 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,

 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

/***** Includes *****/

/* Standard C Libraries */

#include <stdlib.h>

#include <stdio.h>

/* TI Drivers */

#include <ti/drivers/rf/RF.h>

#include <ti/drivers/PIN.h>

#include <ti/drivers/pin/PINCC26XX.h>

/* Driverlib Header files */

#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Board Header files */

#include "Board.h"

/* Application Header files */

#include "RFQueue.h"

#include "smartrf_settings/smartrf_settings.h"

/***** Defines *****/

/* Packet TX/RX Configuration */

#define PAYLOAD_LENGTH      20//30

#define RX_LENGTH 4

/* Set packet interval to 500000us or 5000ms */

#define PACKET_INTERVAL     500000

/* Set Receive timeout to 1 hour = 3600s 0.5f */

//#define RX_TIMEOUT          (uint32_t)(4000000*600);

/* NOTE: Only two data entries supported at the moment */

#define NUM_DATA_ENTRIES    2

/* The Data Entries data field will contain:

 * 1 Header byte (RF_cmdPropRxAdv.rxConf.bIncludeHdr = 0x1)

 * Max 30 payload bytes

 * 1 status byte (RF_cmdPropRxAdv.rxConf.bAppendStatus = 0x1) */

#define NUM_APPENDED_BYTES  1

/* Log radio events in the callback */

//#define LOG_RADIO_EVENTS

/***** Prototypes *****/

//static void echoCallbackRX(RF_Handle h_rx, RF_CmdHandle ch_rx, RF_EventMask e_rx);

//static void echoCallbackTX(RF_Handle h_tx, RF_CmdHandle ch_tx, RF_EventMask e_tx);

/***** Variable declarations *****/

static RF_Object rfObject;

static RF_Handle rfHandle;

/* Pin driver handle */

static PIN_Handle ledPinHandle;

static PIN_State ledPinState;

static uint8_t addressList[] = {0xAA, 0xBB};



/* Buffer which contains all Data Entries for receiving data.

 * Pragmas are needed to make sure this buffer is aligned to a 4 byte boundary

 * (requirement from the RF core)

 */

#if defined(__TI_COMPILER_VERSION__)

#pragma DATA_ALIGN(rxDataEntryBuffer, 4)

static uint8_t

rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,

                                                  RX_LENGTH,

                                                  NUM_APPENDED_BYTES)];

#elif defined(__IAR_SYSTEMS_ICC__)

#pragma data_alignment = 4

static uint8_t

rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,

                                                  RX_LENGTH,

                                                  NUM_APPENDED_BYTES)];

#elif defined(__GNUC__)

static uint8_t

rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,

                                                  RX_LENGTH,

                                                  NUM_APPENDED_BYTES)]

                                                  __attribute__((aligned(4)));

#else

#error This compiler is not supported

#endif //defined(__TI_COMPILER_VERSION__)

/* Receive Statistics */

//static rfc_PropRxAdvOutput_t rxStatistics;

/* Receive dataQueue for RF Core to fill in data */

static dataQueue_t dataQueue;

static rfc_dataEntryGeneral_t* currentDataEntry;

static uint8_t packetLength;

static uint8_t* packetDataPointer;

static uint8_t rxPacket[RX_LENGTH];

static uint8_t txPacket[PAYLOAD_LENGTH];

static uint16_t seqNumber;

static rfc_propRxOutput_t rxStatistics;

static volatile bool Ok_Signal = false;

#ifdef LOG_RADIO_EVENTS

static volatile RF_EventMask eventLog[32];

static volatile uint8_t evIndex = 0;

#endif // LOG_RADIO_EVENTS

/*

 * Application LED pin configuration table:

 *   - All LEDs board LEDs are off.

 */

PIN_Config pinTable[] =

{

#if defined(Board_CC1350_LAUNCHXL)

 Board_DIO30_SWPWR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,

#endif

 Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,

 Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,

 PIN_TERMINATE

};

// start transmission to device with address 0x01 then toggle to 0x02 then to 0x03 Roundrobin

uint8_t Sensor_ID = 0x01;

static void echoCallbackTx(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)

{

    printf("Entered callback TX \n");

    if(e & RF_EventLastCmdDone)

    {

        /* Successful TX */

        /* Toggle LED1, clear LED2 to indicate TX */

        PIN_setOutputValue(ledPinHandle, Board_PIN_LED1,

                           !PIN_getOutputValue(Board_PIN_LED1));

        PIN_setOutputValue(ledPinHandle, Board_PIN_LED2, 0);

        /* Transmission was successful, enter RX mode to wait for reply */

        printf("Successful Tx \n");

        //RF_EventMask  rfStatustx = RF_flushCmd(rfHandle, RF_CMDHANDLE_FLUSH_ALL , 0);

        Ok_Signal=false;

    }

    printf("End echo TX \n");

}

    static void echoCallbackRx(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)

    {

        printf("Entered callback RX \n");

    if(e &  RF_EventRxEntryDone )

    {

        /* Successful RX */

        //bRxSuccess = true;

        //printf("Successful Rx \n");

        /* Get current unhandled data entry */

        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &(currentDataEntry->data):

         * - Length is the first byte with the current configuration

         * - Data starts from the second byte

         */

        packetLength      = *(uint8_t *)(&(currentDataEntry->data));

        packetDataPointer = (uint8_t *)(&(currentDataEntry->data) + 1); //Move the pointer one bytes forward to read byte 1

        //packetDataPointer+=1; //Move the pointer two bytes forward

        //uint8_t thirdByte = *(packetDataPointer);

        uint8_t  fstByte =*(packetDataPointer);

        packetDataPointer+=1;

        uint8_t  scndByte =*(packetDataPointer);

        packetDataPointer+=1;

        uint8_t  thirdByte =*(packetDataPointer);

        if (thirdByte==Sensor_ID){

            printf("Received Ok signal for my ID number= %02X\n",thirdByte);

            Ok_Signal=true;

            //RF_EventMask  rfStatusrx = RF_flushCmd(rfHandle, RF_CMDHANDLE_FLUSH_ALL , 0);

            //RF_EventMask events = RF_pendCmd(rfHandle, (RF_Op*)&RF_cmdFs,RF_EventRxEntryDone);

            /* Set LED2, clear LED1 to indicate RX */

            PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 0);

            PIN_setOutputValue(ledPinHandle, Board_PIN_LED2, 1);

        RFQueue_nextEntry();

    }

        else{

            printf("Received Wrong Board ID number= %02X   = %02X   = %08lx \n", fstByte, scndByte, thirdByte);

            RFQueue_nextEntry();

        }

    }

    printf("end echo RX \n");

    RF_EventMask rfStatus = RF_flushCmd(rfHandle, RF_CMDHANDLE_FLUSH_ALL , 0);

}

/***** Function definitions *****/

void *mainThread(void *arg0)

{

    RF_Params rfParams;

    RF_Params_init(&rfParams);

    /* Open LED pins */

    ledPinHandle = PIN_open(&ledPinState, pinTable);

    if (ledPinHandle == NULL)

    {

        while(1);

    }

    if(RFQueue_defineQueue(&dataQueue,

                           rxDataEntryBuffer,

                           sizeof(rxDataEntryBuffer),

                           NUM_DATA_ENTRIES,

                           RX_LENGTH + NUM_APPENDED_BYTES))

    {

        /* Failed to allocate space for all data entries */

        printf ("Failed to allocate space for all data entries");

        PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);

        PIN_setOutputValue(ledPinHandle, Board_PIN_LED2, 1);

        while(1);

    }

    /* Modify CMD_PROP_TX and CMD_PROP_RX commands for application needs */

    RF_cmdPropTxAdv.pktLen = PAYLOAD_LENGTH+1 ;

    RF_cmdPropTxAdv.pPkt = txPacket;

    RF_cmdPropTxAdv.startTrigger.triggerType = TRIG_ABSTIME;

    RF_cmdPropTxAdv.startTrigger.pastTrig = 1;

    RF_cmdPropTxAdv.startTime = 0;




   //RF_cmdPropTxAdv.startTime = 0;

   // RF_cmdPropTxAdv.pNextOp = (rfc_radioOp_t *)&RF_cmdPropRxAdv;

    //RF_cmdPropRxAdv.pNextOp = (rfc_radioOp_t *)&RF_cmdPropTxAdv;

    /* Set the Data Entity queue for received data */

    RF_cmdPropRxAdv.pQueue = &dataQueue;

    /* Discard ignored packets from Rx queue */

    RF_cmdPropRxAdv.rxConf.bAutoFlushIgnored = 1;

    /* Discard packets with CRC error from Rx queue */

    RF_cmdPropRxAdv.rxConf.bAutoFlushCrcErr = 1;

    /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */

    RF_cmdPropRxAdv.maxPktLen = RX_LENGTH;

    RF_cmdPropRxAdv.pktConf.bRepeatOk = 1;// don't stop at first packet

    RF_cmdPropRxAdv.pktConf.bRepeatNok = 1; //don't stop at first crc error

    RF_cmdPropRxAdv.pAddr = (uint8_t*)&addressList;

    //RF_cmdPropRxAdv.pOutput = (uint8_t *)&rxStatistics;

    /* Receive operation will end RX_TIMEOUT ms after command starts */

    RF_cmdPropRxAdv.startTrigger.triggerType = TRIG_NOW;
    RF_cmdPropRxAdv.startTime = 0;


    //RF_cmdPropRxAdv.pNextOp = (rfc_radioOp_t *)&RF_cmdPropTxAdv; Sensor might recieve different board number so keep recieving

    /* Request access to the radio */

#if defined(DeviceFamily_CC26X0R2)

    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);

#else

    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

#endif// DeviceFamily_CC26X0R2

    /* Set the frequency */

    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    /* Get current time */

    //uint32_t curtime = RF_getCurrentTime();

    while(1)

    {

        Ok_Signal=false;



        printf("Ok signal %d \n",Ok_Signal); //Prints 1 for true, 0 for false

        RF_cmdPropRxAdv.startTime =TRIG_NOW;

       // RF_cmdPropRxAdv.endTime = RX_TIMEOUT; // Set the RX timeout

        printf("Enter Rx mode \n");

        /* Enter receive mode and wait for reply */

        RF_EventMask terminationReasonRx = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRxAdv, RF_PriorityNormal,  &echoCallbackRx, RF_EventRxEntryDone);

       printf("Permit signal %d \n",Ok_Signal); //Prints 1 for true, 0 for false
        //Ok_Signal=true;

        if (Ok_Signal)

        {

        /* Create packet with incrementing sequence number and random payload */

            txPacket[0]= (uint8_t) PAYLOAD_LENGTH;

         txPacket[1]=0xBB;

        txPacket[2] = (uint8_t)(seqNumber++);

        txPacket[3] = Sensor_ID;

        txPacket[4] =  (uint8_t)rand() % 256;

        // Print the value of txPacket[2]

       // printf("txPacket[2] = 0x%02X\n", txPacket[2]);

        uint8_t i;

        for (i=5 ; i<PAYLOAD_LENGTH; i++)

        {

            txPacket[i] = (uint8_t)rand() % 256;

        }

        /* Set absolute TX time to utilize automatic power management */

        RF_cmdPropTxAdv.startTime =TRIG_NOW;

   //   printf("Before tx command\n");

        RF_EventMask terminationReasonTx =

                RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTxAdv, RF_PriorityNormal, &echoCallbackTx, RF_EventTxEntryDone);



        switch(terminationReasonTx)
             {
                 case RF_EventLastCmdDone:
                     // A stand-alone radio operation command or the last radio
                     // operation command in a chain finished.
                     printf("RF_EventLastCmdDon \n");
                     break;
                 case RF_EventCmdCancelled:
                     // Command cancelled before it was started; it can be caused
                 // by RF_cancelCmd() or RF_flushCmd().
                     break;
                 case RF_EventCmdAborted:
                     // Abrupt command termination caused by RF_cancelCmd() or
                     // RF_flushCmd().
                     break;
                 case RF_EventCmdStopped:
                     // Graceful command termination caused by RF_cancelCmd() or
                     // RF_flushCmd().
                     break;
                 default:
                     // Uncaught error event
                     printf("Uncaught error event \n");
                     while(1);
             }

             uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropTxAdv)->status;
             switch(cmdStatus)
             {
                 case PROP_DONE_OK:
                     // Packet transmitted successfully
                     printf("Packet transmitted successfully \n");
                     break;
                 case PROP_DONE_STOPPED:
                     // received CMD_STOP while transmitting packet and finished
                     // transmitting packet
                     break;
                 case PROP_DONE_ABORT:
                     // Received CMD_ABORT while transmitting packet
                     break;
                 case PROP_ERROR_PAR:
                     // Observed illegal parameter
                     break;
                 case PROP_ERROR_NO_SETUP:
                     // Command sent without setting up the radio in a supported
                     // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
                     break;
                 case PROP_ERROR_NO_FS:
                     // Command sent without the synthesizer being programmed
                     break;
                 case PROP_ERROR_TXUNF:
                     // TX underflow observed during operation
                     printf("TX underflow observed during operation \n");
                     break;
                 default:
                     // Uncaught error event - these could come from the
                     // pool of states defined in rf_mailbox.h
                     printf("Uncaught error event 2\n");
                     while(1);
             }




        }


}

}
