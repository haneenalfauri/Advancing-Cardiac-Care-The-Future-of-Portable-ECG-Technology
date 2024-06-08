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

/* Sensor Controller Driver and Initialization */
#include "scif.h"
#define BV(x)    (1 << (x)) // It stands for Bit Value where you pass it a bit and it gives you the byte value with that bit set.

/* For usleep() */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <semaphore.h>
/* Driver Header files */
#include <ti/drivers/GPIO.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>

/* Standard C Libraries */
#include <stdlib.h>
#include <unistd.h>

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26XXDMA.h>
#include <ti/drivers/SD.h>
/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Board Header files */
#include "Board.h"
#include "smartrf_settings/smartrf_settings.h"



////////////////////////////////////////////
#include "RFQueue.h"








/***** Defines *****/

/* Do power measurement */
//#define POWER_MEASUREMENT

/* Packet TX Configuration */
#define NUM_READ 1000
#define PAYLOAD_LENGTH      2 * NUM_READ +4


#ifdef POWER_MEASUREMENT
#define PACKET_INTERVAL     5  /* For power measurement set packet interval to 5s */
#else
#define PACKET_INTERVAL     10000  /* Set packet interval to 500000us or 500ms */
#endif





/* Total Number of Samples in Packet*/
#define totalSamples        1 * NUM_READ

//#define mAvgBufferArraySize 300

/***** Prototypes *****/

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

static uint8_t packet[PAYLOAD_LENGTH+2];
static uint16_t seqNumber;
static uint8_t version1 = 0;
static uint8_t version2 = 2;

static uint16_t ADCReadings[totalSamples];
static unsigned int cnt = 0;
//static uint16_t movingAvgArray[totalSamples];
//static uint16_t movingAvgBufferArray[mAvgBufferArraySize];
//static uint16_t numSamples = 0;


extern sem_t data_ready_sem;
extern sem_t write_ready_sem;
extern sem_t send_ready_sem;


// Pin Table
PIN_Config pinTable[] = {
    SENSOR_ADC_RST_N    | PIN_GPIO_OUTPUT_EN    | PIN_GPIO_HIGH,
    PIN_TERMINATE
};
PIN_Handle pinHandle;
PIN_State pinState;


/////////////////////////////////////////////////////
/***** Defines *****/
/* Packet RX/TX Configuration */
/* Max length byte the radio will accept */
#define rf_PAYLOAD_LENGTH         30

/* NOTE: Only two data entries supported at the moment */
#define NUM_DATA_ENTRIES       2

/* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     2

/* Log radio events in the callback */
//#define LOG_RADIO_EVENTS

/***** Prototypes *****/
//static void echoCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;


/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is aligned to a 4 byte boundary
 * (requirement from the RF core)
 */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(rxDataEntryBuffer, 4)
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  rf_PAYLOAD_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 4
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  rf_PAYLOAD_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__GNUC__)
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  rf_PAYLOAD_LENGTH,
                                                  NUM_APPENDED_BYTES)]
                                                  __attribute__((aligned(4)));
#else
#error This compiler is not supported
#endif //defined(__TI_COMPILER_VERSION__)


/* Receive Statistics */
//static rfc_propRxOutput_t rxStatistics;

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
//static uint8_t packetLength;
static uint8_t* packetDataPointer;


#ifdef LOG_RADIO_EVENTS
static volatile RF_EventMask eventLog[32];
static volatile uint8_t evIndex = 0;
#endif // LOG_RADIO_EVENTS

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.

PIN_Config pinTable[] =
{
#if defined(Board_CC1350_LAUNCHXL)
 Board_DIO30_SWPWR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
 Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
 Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
 PIN_TERMINATE
};

*/










//////////////////////////////////////////////////////







/***** Function definitions *****/

void processTaskAlert(void)
{


} // processTaskAlert

void scCtrlReadyCallback(void)
{

} // scCtrlReadyCallback

void scTaskAlertCallback(void)
{
   // printf("%d\n",scifTaskData.ads8689.output.buffer[0]);

    // Clear the ALERT interrupt Source
    scifClearAlertIntSource();
    //unsigned int index = 6;
    //memcpy(&ADCReadings[index + cnt * 10], (uint16_t *)scifTaskData.ads8689.output.buffer, 20);
    memcpy(&ADCReadings[cnt * 1], (uint16_t *)scifTaskData.ads8689.output.buffer, 2);
    cnt++;
    if (cnt == NUM_READ){
        cnt = 0;
        sem_post(&data_ready_sem);
    }
    //memcpy(&ADCReadings, (uint16_t *)scifTaskData.ads8689.output.buffer, 100);
    //printf("%d\n", ADCReadings[49]);
   // printf("%d\n", ADCReadings[25]);
    // Post to other task
    //sem_post(&data_ready_sem);
    // Get data from the Sensor Controller
    scifAckAlertEvents();

       // processTaskAlert();

} // scTaskAlertCallback


static void* echoCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{  printf("echoCallback\n");
#ifdef LOG_RADIO_EVENTS
    eventLog[evIndex++ & 0x1F] = e;
#endif// LOG_RADIO_EVENTS

    if (e & RF_EventRxEntryDone)
    {
        /* Successful RX */
        /* Toggle LED2, clear LED1 to indicate RX */
        //PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 0);
        //PIN_setOutputValue(ledPinHandle, Board_PIN_LED2,
                          // !PIN_getOutputValue(Board_PIN_LED2));

        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &currentDataEntry->data:
         * - Length is the first byte with the current configuration
         * - Data starts from the third byte */
        //packetLength      = *(uint8_t *)(&(currentDataEntry->data));
        packetDataPointer = (uint8_t *)(&(currentDataEntry->data) + 3);


        //extraxt the first byte
        uint8_t firstByte = *(packetDataPointer);
        if (firstByte==0x01){
            printf("Value of board number: %02X\n",firstByte);
            printf("correct board number \n, ");

            RFQueue_nextEntry();
            sem_post(&send_ready_sem);



        }


        else {
            printf("Value of board number recieved: %02X\n",firstByte);
            printf("Wrong board number \n");

        }

    }

    else // any uncaught event
    {
        /* Error Condition: set LED1, clear LED2 */
        //PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);
        //PIN_setOutputValue(ledPinHandle, Board_PIN_LED2, 0);
    }


}

void *Wait_for_controller_signal(void *arg0){
printf("Wait_for_controller_signal\n");
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if (ledPinHandle == NULL)
    {
        while(1);
    }

    if( RFQueue_defineQueue(&dataQueue,
                            rxDataEntryBuffer,
                            sizeof(rxDataEntryBuffer),
                            NUM_DATA_ENTRIES,
                            rf_PAYLOAD_LENGTH + NUM_APPENDED_BYTES))
    {

        printf("Failed to allocate space for all data entries \n");
        /* Failed to allocate space for all data entries */
        //PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);
        //PIN_setOutputValue(ledPinHandle, Board_PIN_LED2, 1);
        while(1);
    }

    /* Modify CMD_PROP_TX and CMD_PROP_RX commands for application needs */
    /* Set the Data Entity queue for received data */
    RF_cmdPropRxAdv.pQueue = &dataQueue;
    /* Discard ignored packets from Rx queue */
    RF_cmdPropRxAdv.rxConf.bAutoFlushIgnored = 1;
    /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRxAdv.rxConf.bAutoFlushCrcErr = 1;
    /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRxAdv.maxPktLen = rf_PAYLOAD_LENGTH;
    /* End RX operation when a packet is received correctly and move on to the
     * next command in the chain */
    RF_cmdPropRxAdv.pktConf.bRepeatOk = 0; // if 0 End RX operation on successful packet reception
    RF_cmdPropRx.pktConf.bRepeatNok = 0; // Move on to the next command on failed reception
    RF_cmdPropRxAdv.startTrigger.triggerType = TRIG_NOW; // Start immediately
    //RF_cmdPropRxAdv.pNextOp = (rfc_radioOp_t *)&RF_cmdPropTxAdv;

    //RF_cmdPropRxAdv.condition.rule = COND_STOP_ON_FALSE;
    //RF_cmdPropRxAdv.pOutput = (uint8_t *)&rxStatistics;



    /* Request access to the radio */
#if defined(DeviceFamily_CC26X0R2)
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
#else
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
#endif// DeviceFamily_CC26X0R2


    //RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityHighest , NULL, 0);

    while(1)
    {

        printf("Next run RX command \n");
        RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRxAdv, RF_PriorityHighest, echoCallback, RF_EventRxEntryDone);



        switch(terminationReason)
        {
            case RF_EventLastCmdDone:
                // A stand-alone radio operation command or the last radio
                // operation command in a chain finished.
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
                while(1);
        }

        uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropRx)->status;
        switch(cmdStatus)
        {
            case PROP_DONE_OK:
                // Packet received with CRC OK
                break;
            case PROP_DONE_RXERR:
                // Packet received with CRC error
                break;
            case PROP_DONE_RXTIMEOUT:
                // Observed end trigger while in sync search
                break;
            case PROP_DONE_BREAK:
                // Observed end trigger while receiving packet when the command is
                // configured with endType set to 1
                break;
            case PROP_DONE_ENDED:
                // Received packet after having observed the end trigger; if the
                // command i333s configured with endType set to 0, the end trigger
                // will not terminate an ongoing reception
                break;
            case PROP_DONE_STOPPED:
                // received CMD_STOP after command started and, if sync found,
                // packet is received
                break;
            case PROP_DONE_ABORT:
                // Received CMD_ABORT after command started
                break;
            case PROP_ERROR_RXBUF:
                // No RX buffer large enough for the received data available at
                // the start of a packet
                break;
            case PROP_ERROR_RXFULL:
                // Out of RX buffer space during reception in a partial read
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
            case PROP_ERROR_RXOVF:
                // RX overflow observed during operation
                break;
            default:
                // Uncaught error event - these could come from the
                // pool of states defined in rf_mailbox.h
                while(1);
        }
    }
}




















void *mainThread(void *arg0)
{
    printf("main thread\n");
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    while(1)
    {

        printf("main thread while loop\n");
    //printf("before wait\n");
        sem_wait(&write_ready_sem);
        printf("before sem_wait(&send_ready_sem) \n");
        sem_wait(&send_ready_sem);
        printf("After sem_wait(&send_ready_sem) \n");


        RF_cmdPropTxAdv.pktLen = PAYLOAD_LENGTH + 2;
        RF_cmdPropTxAdv.pPkt = packet;
        RF_cmdPropTxAdv.startTrigger.triggerType = TRIG_NOW;

        /* Request access to the radio */
    #if defined(DeviceFamily_CC26X0R2)
        rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
    #else
        rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
    #endif// DeviceFamily_CC26X0R2

        /* Set the frequency */
        RF_runCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);



    //printf("before if\n");
        //if (numSamples == totalSamples) {

            packet[1] = (uint8_t)((PAYLOAD_LENGTH ) >> 8);
            packet[0] = (uint8_t)((PAYLOAD_LENGTH ));

            /* Create packet with incrementing sequence number and random payload */
            packet[2] = (uint8_t)(version1);
            packet[3] = (uint8_t)(version2);
            packet[4] = (uint8_t)(seqNumber >> 8);
            packet[5] = (uint8_t)(seqNumber++);



            uint16_t i;
      //      printf("ADCs\n");
            /* Fill first half of packet payload with sampled data */
            for (i = 5; i < totalSamples + 5; i++)
            {
                int q = i*2 - 4;
                packet[q] = (uint8_t) (ADCReadings[i-5] >> 8);
                packet[q+1] = (uint8_t) (ADCReadings[i-5]);
                //printf("%d\n", ADCReadings[i-4]);
                //printf("\n");
            }

            printf("packet filled next is send command\n");
            /* Send packet */
            RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTxAdv,
                                                       RF_PriorityNormal, NULL, 0);
        //    printf("did send\n");
            switch(terminationReason)
            {
                case RF_EventLastCmdDone:
                    // A stand-alone radio operation command or the last radio
                    // operation command in a chain finished.
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
                    while(1);
            }

            uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropTxAdv)->status;
            switch(cmdStatus)
            {
                case PROP_DONE_OK:
                    // Packet transmitted successfully
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
                    break;
                default:
                    // Uncaught error event - these could come from the
                    // pool of states defined in rf_mailbox.h
                    while(1);
            }

//    #ifndef POWER_MEASUREMENT
//            PIN_setOutputValue(ledPinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
//    #endif
            /* Power down the radio */
            RF_yield(rfHandle);

    #ifdef POWER_MEASUREMENT
            /* Sleep for PACKET_INTERVAL s */
            sleep(PACKET_INTERVAL);
    #else
            /* Sleep for PACKET_INTERVAL us */
            usleep(PACKET_INTERVAL);
    #endif

            //numSamples = 0;

        //}

    }

}
/* Sensor Controller & ADC Thread  */
void *adc_sc_thread(void *arg0){


  printf("adc_sc_thread\n");
    SPI_Params spiParams;
    SPI_Handle spiHandle;
    SPI_Transaction setupTransaction;
    uint8_t cmd_buff[4]={0,0,0,0}, data_buff[4];
    struct timespec sleep_time;
    uint32_t rtc_Hz;
    uint8_t i;
    pinHandle = PIN_open(&pinState, pinTable);

    // Open the SPI Driver
    SPI_init();
    SPI_Params_init(&spiParams);
    spiParams.bitRate = 1000000; // Was 10 MHz
    spiParams.dataSize = 8; // Frame size in bits
    spiParams.mode = SPI_MASTER;
    spiParams.frameFormat = SPI_POL0_PHA0;
    spiHandle = SPI_open(Board_SPI0, &spiParams);

    // Reset the ADS868x
    sleep_time.tv_sec = 0;
    sleep_time.tv_nsec = 1000000; // About 1 ms, satisfies both delays.
    PIN_setOutputValue(pinHandle, SENSOR_ADC_RST_N, 0);
    nanosleep(&sleep_time,NULL); // Need 2048+ of tCLKIN
    PIN_setOutputValue(pinHandle, SENSOR_ADC_RST_N, 1);
    nanosleep(&sleep_time,NULL); // PoR delay is 250 us

    // Program the ADS868x
    // Set up SPI Transaction
    setupTransaction.count = 4;
    setupTransaction.rxBuf = data_buff;
    setupTransaction.txBuf= cmd_buff;

    // Set the output range to 1.25*VREF(~5.12V)
    //cmd_buff[0] = 0xD0; cmd_buff[1] = 0x14; cmd_buff[2] = 0x00; cmd_buff[3] = 0x0B;
    //SPI_transfer(spiHandle, &setupTransaction);
    // Set the output range to +-1.25*VREF(~5.12V)
    //cmd_buff[0] = 0xD0; cmd_buff[1] = 0x14; cmd_buff[2] = 0x00; cmd_buff[3] = 0x03;
    //SPI_transfer(spiHandle, &setupTransaction);
    // Set the output range to +-0.625*VREF(~2.56V)
    // <Input Cmd> <Register Add> <Cmd> - Pg 42.
    cmd_buff[0] = 0xD0; cmd_buff[1] = 0x14; cmd_buff[2] = 0x00; cmd_buff[3] = 0x04;
    SPI_transfer(spiHandle, &setupTransaction);
    cmd_buff[0] = 0x48; cmd_buff[1] = 0x14; cmd_buff[2] = 0x00; cmd_buff[3] = 0x00;
    SPI_transfer(spiHandle, &setupTransaction);
    cmd_buff[0] = 0x00; cmd_buff[1] = 0x00; cmd_buff[2] = 0x00; cmd_buff[3] = 0x00;
    SPI_transfer(spiHandle, &setupTransaction);

    //Drop initial bad few readings
   // for(i=0; i < 10; i++){
     //   SPI_transfer(spiHandle, &setupTransaction);
    //}

    // Initialize the Sensor Controller Task
    scifOsalInit();
    scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
    scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
    scifInit(&scifDriverSetup);
    // Set period
    rtc_Hz = (uint32_t)((65535)/1000); // 2^16 / target_sample_rate
    scifStartRtcTicksNow(rtc_Hz);
    scifStartTasksNbl(BV(SCIF_ADS8689_TASK_ID));

    //unsigned int cnt = 0;
    // Main program execution loop
    while(1){
       printf("before data ready\n");
        sem_wait(&data_ready_sem); // from sensor controller
        sem_post(&write_ready_sem); // signal the usd or rf_tx tasks to run
      printf("after data ready\n");


    }
}
