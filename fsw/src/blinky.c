/*******************************************************************************
**
**      GSC-18128-1, "Core Flight Executive Version 6.7"
**
**      Copyright (c) 2006-2019 United States Government as represented by
**      the Administrator of the National Aeronautics and Space Administration.
**      All Rights Reserved.
**
**      Licensed under the Apache License, Version 2.0 (the "License");
**      you may not use this file except in compliance with the License.
**      You may obtain a copy of the License at
**
**        http://www.apache.org/licenses/LICENSE-2.0
**
**      Unless required by applicable law or agreed to in writing, software
**      distributed under the License is distributed on an "AS IS" BASIS,
**      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
**      See the License for the specific language governing permissions and
**      limitations under the License.
**
** File: blinky.c
**
** Purpose:
**   This file contains the source code for the blinky app.
**
*******************************************************************************/

/*
** Include Files:
*/
#include "blinky_events.h"
#include "blinky_version.h"
#include "blinky.h"

#include <string.h>
#include <bsp.h> /* for device driver prototypes */
#include <bsp/gpio.h>

/*
** global data
*/
BLINKY_Data_t BLINKY_Data;

uint32 BLINKY_GpioLedMap[] = {
        BBB_LED_USR0,
	BBB_LED_USR1,
	BBB_LED_USR2,
	BBB_LED_USR3,
        BBB_P8_10,
	BBB_P8_12,
	BBB_P8_14,
	BBB_P8_16
     };

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
/* BLINKY_Main() -- Application entry point and main process loop         */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
void BLINKY_Main(void)
{
    int32            status;
    CFE_SB_Buffer_t *SBBufPtr;

    /*
    ** Create the first Performance Log entry
    */
    CFE_ES_PerfLogEntry(BLINKY_PERF_ID);

    /*
    ** Perform application specific initialization
    ** If the Initialization fails, set the RunStatus to
    ** CFE_ES_RunStatus_APP_ERROR and the App will not enter the RunLoop
    */
    status = BLINKY_Init();
    if (status != CFE_SUCCESS)
    {
        BLINKY_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    status = genuC_driver_open();
    if (status != CFE_SUCCESS)
    {
        BLINKY_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    /*
    ** BLINKY Runloop
    */
    while (CFE_ES_RunLoop(&BLINKY_Data.RunStatus) == true)
    {
        /*
        ** Performance Log Exit Stamp
        */
        CFE_ES_PerfLogExit(BLINKY_PERF_ID);

        /* Pend on receipt of command packet */
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, BLINKY_Data.CommandPipe, CFE_SB_PEND_FOREVER);

        /*
        ** Performance Log Entry Stamp
        */
        CFE_ES_PerfLogEntry(BLINKY_PERF_ID);

        if (status == CFE_SUCCESS)
        {
            BLINKY_ProcessCommandPacket(SBBufPtr);
        }
        else
        {
            CFE_EVS_SendEvent(BLINKY_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "BLINKY APP: SB Pipe Read Error, App Will Exit");

            BLINKY_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }

    /*
    ** Performance Log Exit Stamp
    */
    CFE_ES_PerfLogExit(BLINKY_PERF_ID);

    CFE_ES_ExitApp(BLINKY_Data.RunStatus);

} /* End of BLINKY_Main() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  */
/*                                                                            */
/* BLINKY_Init() --  initialization                                       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 BLINKY_Init(void)
{
    int32 status;

    BLINKY_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    /*
    ** Initialize app command execution counters
    */
    BLINKY_Data.CmdCounter = 0;
    BLINKY_Data.ErrCounter = 0;

    /*
    ** Initialize app configuration data
    */
    BLINKY_Data.PipeDepth = BLINKY_PIPE_DEPTH;

    strncpy(BLINKY_Data.PipeName, "BLINKY_CMD_PIPE", sizeof(BLINKY_Data.PipeName));
    BLINKY_Data.PipeName[sizeof(BLINKY_Data.PipeName) - 1] = 0;

    /*
    ** Initialize event filter table...
    */
    BLINKY_Data.EventFilters[0].EventID = BLINKY_STARTUP_INF_EID;
    BLINKY_Data.EventFilters[0].Mask    = 0x0000;
    BLINKY_Data.EventFilters[1].EventID = BLINKY_COMMAND_ERR_EID;
    BLINKY_Data.EventFilters[1].Mask    = 0x0000;
    BLINKY_Data.EventFilters[2].EventID = BLINKY_COMMANDNOP_INF_EID;
    BLINKY_Data.EventFilters[2].Mask    = 0x0000;
    BLINKY_Data.EventFilters[3].EventID = BLINKY_COMMANDRST_INF_EID;
    BLINKY_Data.EventFilters[3].Mask    = 0x0000;
    BLINKY_Data.EventFilters[4].EventID = BLINKY_INVALID_MSGID_ERR_EID;
    BLINKY_Data.EventFilters[4].Mask    = 0x0000;
    BLINKY_Data.EventFilters[5].EventID = BLINKY_LEN_ERR_EID;
    BLINKY_Data.EventFilters[5].Mask    = 0x0000;
    BLINKY_Data.EventFilters[6].EventID = BLINKY_PIPE_ERR_EID;
    BLINKY_Data.EventFilters[6].Mask    = 0x0000;

    /*
    ** Register the events
    */
    status = CFE_EVS_Register(BLINKY_Data.EventFilters, BLINKY_EVENT_COUNTS, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Blinky: Error Registering Events, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    /*
    ** Initialize housekeeping packet (clear user data area).
    */
    CFE_MSG_Init(CFE_MSG_PTR(BLINKY_Data.HkTlm.TelemetryHeader), CFE_SB_ValueToMsgId(BLINKY_HK_TLM_MID),
                 sizeof(BLINKY_Data.HkTlm));

    /*
    ** Create Software Bus message pipe.
    */
    status = CFE_SB_CreatePipe(&BLINKY_Data.CommandPipe, BLINKY_Data.PipeDepth, BLINKY_Data.PipeName);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Blinky: Error creating pipe, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    /*
    ** Subscribe to Housekeeping request commands
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(BLINKY_SEND_HK_MID), BLINKY_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Blinky: Error Subscribing to HK request, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    /*
    ** Subscribe to ground command packets
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(BLINKY_CMD_MID), BLINKY_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Blinky: Error Subscribing to Command, RC = 0x%08lX\n", (unsigned long)status);

        return (status);
    }


    CFE_EVS_SendEvent(BLINKY_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION, "BLINKY App Initialized.%s",
                      BLINKY_VERSION_STRING);

    printf("Calling rtems_gpio_initialize\n");
    rtems_gpio_initialize ();

    return (CFE_SUCCESS);

} /* End of BLINKY_Init() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  BLINKY_ProcessCommandPacket                                    */
/*                                                                            */
/*  Purpose:                                                                  */
/*     This routine will process any packet that is received on the BLINKY    */
/*     command pipe.                                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void BLINKY_ProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;

    CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

    switch (CFE_SB_MsgIdToValue(MsgId))
    {
        case BLINKY_CMD_MID:
            BLINKY_ProcessGroundCommand(SBBufPtr);
            break;

        case BLINKY_SEND_HK_MID:
            BLINKY_ReportHousekeeping((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;

        default:
            CFE_EVS_SendEvent(BLINKY_INVALID_MSGID_ERR_EID, CFE_EVS_EventType_ERROR,
                              "BLINKY: invalid command packet,MID = 0x%x", (unsigned int)CFE_SB_MsgIdToValue(MsgId));
            break;
    }

    return;

} /* End BLINKY_ProcessCommandPacket */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* BLINKY_ProcessGroundCommand() -- BLINKY ground commands                */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void BLINKY_ProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_MSG_FcnCode_t CommandCode = 0;

    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    /*
    ** Process "known" BLINKY app ground commands
    */
    switch (CommandCode)
    {
        case BLINKY_NOOP_CC:
            if (BLINKY_VerifyCmdLength(&SBBufPtr->Msg, sizeof(BLINKY_NoopCmd_t)))
            {
                BLINKY_Noop((BLINKY_NoopCmd_t *)SBBufPtr);
            }

            break;

        case BLINKY_RESET_COUNTERS_CC:
            if (BLINKY_VerifyCmdLength(&SBBufPtr->Msg, sizeof(BLINKY_ResetCountersCmd_t)))
            {
                BLINKY_ResetCounters((BLINKY_ResetCountersCmd_t *)SBBufPtr);
            }

            break;

        case BLINKY_LED_ON_CC:
            if (BLINKY_VerifyCmdLength(&SBBufPtr->Msg, sizeof(BLINKY_LedStateCmd_t)))
            {
                BLINKY_LedOn((BLINKY_LedStateCmd_t *)SBBufPtr);
            }

	    break;

        case BLINKY_LED_OFF_CC:
            if (BLINKY_VerifyCmdLength(&SBBufPtr->Msg, sizeof(BLINKY_LedStateCmd_t)))
            {
                BLINKY_LedOff((BLINKY_LedStateCmd_t *)SBBufPtr);
            }

            break;

        /* default case already found during FC vs length test */
        default:
            CFE_EVS_SendEvent(BLINKY_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Invalid ground command code: CC = %d", CommandCode);
            break;
    }

    return;

} /* End of BLINKY_ProcessGroundCommand() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  BLINKY_ReportHousekeeping                                          */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function is triggered in response to a task telemetry request */
/*         from the housekeeping task. This function will gather the Apps     */
/*         telemetry, packetize it and send it to the housekeeping task via   */
/*         the software bus                                                   */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 BLINKY_ReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg)
{
    int i;

    /*
    ** Get command execution counters...
    */
    BLINKY_Data.HkTlm.Payload.CommandErrorCounter = BLINKY_Data.ErrCounter;
    BLINKY_Data.HkTlm.Payload.CommandCounter      = BLINKY_Data.CmdCounter;

    /* Copy the Led states.. */
    for (i = 0; i < BLINKY_NUM_LEDS; i++)
       BLINKY_Data.HkTlm.Payload.LedState[i]      = BLINKY_Data.LedState[i];

     /*
    ** Make the RF telemetry packet...
    */
    BLINKY_Data.RFTlm.AppID[0] = (uint8_t) ((BLINKY_HK_TLM_MID >> 8) & 0xff);
    BLINKY_Data.RFTlm.AppID[1] = (uint8_t) (BLINKY_HK_TLM_MID & 0xff);
    BLINKY_Data.RFTlm.Payload = BLINKY_Data.HkTlm.Payload;

    /*
    ** Send housekeeping telemetry packet...
    */
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(BLINKY_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(BLINKY_Data.HkTlm.TelemetryHeader), true);

    send_tlm_data();

    return CFE_SUCCESS;

} /* End of BLINKY_ReportHousekeeping() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* BLINKY_Noop --  NOOP commands                                        */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 BLINKY_Noop(const BLINKY_NoopCmd_t *Msg)
{

    BLINKY_Data.CmdCounter++;

    CFE_EVS_SendEvent(BLINKY_COMMANDNOP_INF_EID, CFE_EVS_EventType_INFORMATION, "BLINKYs: NOOP command %s",
                      BLINKY_VERSION);

    return CFE_SUCCESS;

} /* End of BLINKY_Noop */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  BLINKY_ResetCounters                                               */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function resets all the global counter variables that are     */
/*         part of the task telemetry.                                        */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 BLINKY_ResetCounters(const BLINKY_ResetCountersCmd_t *Msg)
{
    BLINKY_Data.CmdCounter = 0;
    BLINKY_Data.ErrCounter = 0;

    CFE_EVS_SendEvent(BLINKY_COMMANDRST_INF_EID, CFE_EVS_EventType_INFORMATION, "BLINKYs: RESET command");

    return CFE_SUCCESS;

} /* End of BLINKY_ResetCounters() */

/** * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  BLINKY_LedOn                                                       */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function processes the LED On ground command                  */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 BLINKY_LedOn(const BLINKY_LedStateCmd_t *Msg)
{
    rtems_status_code sc;

    if (Msg->LedNumber < BLINKY_NUM_LEDS)
    {
       BLINKY_Data.CmdCounter++;
       CFE_EVS_SendEvent(BLINKY_COMMANDLEDON_INF_EID, CFE_EVS_EventType_INFORMATION,
		         "BLINKY: LED %d ON command\n",
                         Msg->LedNumber);
       BLINKY_Data.LedState[Msg->LedNumber] = BLINKY_LED_ON;

       sc = rtems_gpio_request_pin(
                       BLINKY_GpioLedMap[Msg->LedNumber],
		       DIGITAL_OUTPUT, false, false, NULL);
       if (sc != RTEMS_SUCCESSFUL )
       {
          printf("Failed to request LED GPIO %d\n",Msg->LedNumber);
       }
       else
       {
          rtems_gpio_set(BLINKY_GpioLedMap[Msg->LedNumber]);
       }
    }
    else
    {
       BLINKY_Data.ErrCounter++;
       CFE_EVS_SendEvent(BLINKY_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                         "Invalid LED Number in Ground Command. LED = %d", Msg->LedNumber);
    }
    return CFE_SUCCESS;

} /* End of BLINKY_LedOn */

/** * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  BLINKY_LedOff                                                      */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function processes the LED Off ground command                 */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 BLINKY_LedOff(const BLINKY_LedStateCmd_t *Msg)
{
    OS_printf("BLINKY Turn LED OFF\n");

    if (Msg->LedNumber < BLINKY_NUM_LEDS)
    {
       BLINKY_Data.CmdCounter++;
       CFE_EVS_SendEvent(BLINKY_COMMANDLEDOFF_INF_EID, CFE_EVS_EventType_INFORMATION,
		         "BLINKY: LED %d OFF command\n",
                         Msg->LedNumber);
       BLINKY_Data.LedState[Msg->LedNumber] = BLINKY_LED_OFF;
       rtems_gpio_clear(BLINKY_GpioLedMap[Msg->LedNumber]);
       rtems_gpio_release_pin(BLINKY_GpioLedMap[Msg->LedNumber]);
    }
    else
    {
       BLINKY_Data.ErrCounter++;
       CFE_EVS_SendEvent(BLINKY_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                         "Invalid LED Number in Ground Command. LED = %d", Msg->LedNumber);
    }
    return CFE_SUCCESS;

} /* End of BLINKY_LedOff */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* BLINKY_VerifyCmdLength() -- Verify command packet length                   */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
bool BLINKY_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
{
    bool              result       = true;
    size_t            ActualLength = 0;
    CFE_SB_MsgId_t    MsgId        = CFE_SB_INVALID_MSG_ID;
    CFE_MSG_FcnCode_t FcnCode      = 0;

    CFE_MSG_GetSize(MsgPtr, &ActualLength);

    /*
    ** Verify the command packet length.
    */
    if (ExpectedLength != ActualLength)
    {
        CFE_MSG_GetMsgId(MsgPtr, &MsgId);
        CFE_MSG_GetFcnCode(MsgPtr, &FcnCode);

        CFE_EVS_SendEvent(BLINKY_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "Invalid Msg length: ID = 0x%X,  CC = %u, Len = %u, Expected = %u",
                          (unsigned int)CFE_SB_MsgIdToValue(MsgId), (unsigned int)FcnCode, (unsigned int)ActualLength,
                          (unsigned int)ExpectedLength);

        result = false;

        BLINKY_Data.ErrCounter++;
    }

    return (result);

} /* End of BLINKY_VerifyCmdLength() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* Functions to interact with the gen-uC                                     */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
#ifdef GENUC
  static int uC_ioctl(i2c_dev *dev, ioctl_command_t command, void *arg);

  int32 send_tlm_data(){
    int rv;

    uint8_t *val;
    val = NULL;
    val = malloc(RF_PAYLOAD_BYTES * sizeof(uint8_t));

    val[0] = BLINKY_Data.RFTlm.AppID[0];
    val[1] = BLINKY_Data.RFTlm.AppID[1];
    val[2] = BLINKY_Data.RFTlm.Payload.CommandErrorCounter;
    val[3] = BLINKY_Data.RFTlm.Payload.CommandCounter;
    val[4] = 0;
    val[5] = 0;

    for(int i=0;i<8;i++){
      val[i+6] = BLINKY_Data.RFTlm.Payload.LedState[i];
    }

    for(int i=14;i<RF_PAYLOAD_BYTES;i++){
      val[i] = 0;
    }

    // Send the telemetry payload
    rv = uC_set_bytes(UC_ADDRESS, &val, RF_PAYLOAD_BYTES);
    if(rv >= 0){
      return CFE_SUCCESS;
    }else{
      return -1;
    }
  }

  int32 genuC_driver_open(){

  int rv;
  int fd;

  // Device registration
  rv = i2c_dev_register_uC(
    &bus_path[0],
    &genuC_path[0]
  );
  if(rv == 0)
    CFE_EVS_SendEvent(BLINKY_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "BLINKY: Device registered correctly at %s",
                      genuC_path);

  fd = open(&genuC_path[0], O_RDWR);
  if(fd >= 0)
    CFE_EVS_SendEvent(BLINKY_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "BLINKY: Device opened correctly at %s",
                      genuC_path);
  close(fd);

  if(rv == 0 && fd >=0){
    return CFE_SUCCESS;
  }else{
    return -1;
  }

}

  #ifdef uC_reading

  static int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes, uint8_t **buff);

  static int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes, uint8_t **buff){
  int rv;
  uint8_t value[nr_bytes];
  i2c_msg msgs[] = {{
    .addr = i2c_address,
    .flags = 0,
    .buf = &data_address,
    .len = 1,
  }, {
    .addr = i2c_address,
    .flags = I2C_M_RD,
    .buf = value,
    .len = nr_bytes,
  }};
  struct i2c_rdwr_ioctl_data payload = {
    .msgs = msgs,
    .nmsgs = sizeof(msgs)/sizeof(msgs[0]),
  };
  uint16_t i;

  rv = ioctl(fd, I2C_RDWR, &payload);
  if (rv < 0) {
    printf("ioctl failed...\n");
  } else {

    free(*buff);
    *buff = malloc(nr_bytes * sizeof(uint8_t));

    for (i = 0; i < nr_bytes; ++i) {
      (*buff)[i] = value[i];
    }
  }

  return rv;
  }

  int uC_get_bytes(uint16_t chip_address, uint8_t register_add, uint8_t **buff){

  int fd;
  int rv;

  uint8_t *tmp;
  tmp = NULL;

  free(*buff);
  *buff = malloc(1 * sizeof(uint8_t));

  uint16_t nr_bytes = (uint16_t) 1;
  uint8_t data_address = (uint8_t) register_add;

  fd = open(&bus_path[0], O_RDWR);
  if (fd < 0) {
    printf("Couldn't open bus...\n");
    return 1;
  }

  if(chip_address == 0){
    chip_address = (uint16_t) UC_ADDRESS;
  }

  rv = read_bytes(fd, chip_address, data_address, nr_bytes, &tmp);

  close(fd);

  (*buff)[0] = *tmp;
  free(tmp);

  return rv;
  }

  #endif

  int uC_set_bytes(uint16_t chip_address, uint8_t **val, int numBytes){

  int fd;
  int rv;

  if(chip_address == 0){
    chip_address = (uint16_t) UC_ADDRESS;
  }

  uint8_t writebuff[numBytes];

  for(int i = 0; i<numBytes; i++){
    writebuff[i] = (*val)[i];
  }

  i2c_msg msgs[] = {{
    .addr = chip_address,
    .flags = 0,
    .buf = writebuff,
    .len = numBytes,
  }};
  struct i2c_rdwr_ioctl_data payload = {
    .msgs = msgs,
    .nmsgs = sizeof(msgs)/sizeof(msgs[0]),
  };

  fd = open(&bus_path[0], O_RDWR);
  if (fd < 0) {
    printf("Couldn't open bus...\n");
    return 1;
  }

  rv = ioctl(fd, I2C_RDWR, &payload);
  if (rv < 0) {
    perror("ioctl failed");
  }
  close(fd);

  return rv;
  }

  int i2c_dev_register_uC(const char *bus_path, const char *dev_path){
  i2c_dev *dev;

  dev = i2c_dev_alloc_and_init(sizeof(*dev), bus_path, UC_ADDRESS);
  if (dev == NULL) {
    return -1;
  }

  dev->ioctl = uC_ioctl;

  return i2c_dev_register(dev, dev_path);
  }

  static int uC_ioctl(i2c_dev *dev, ioctl_command_t command, void *arg){
  int err;

  // Variables for the Send test
  int numBytes = 3;
  uint8_t *val;

  switch (command) {
    case UC_SEND_TEST:

      val = NULL;
      val = malloc(numBytes * sizeof(uint8_t));

      val[0] = 0x03;
      val[1] = 0x06;
      val[2] = 0x09;

      err = uC_set_bytes(UC_ADDRESS, &val, numBytes); //Send 0x03, 0x06 and 0x09 to the uC default address
      break;

    default:
      err = -ENOTTY;
      break;
  }

  return err;
  }

  int uC_send_test(int fd){
    return ioctl(fd, UC_SEND_TEST, NULL);
  }
#endif
