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
    ** Initialize output RF packet.
    */
    CFE_MSG_Init(CFE_MSG_PTR(BLINKY_Data.OutData.TelemetryHeader), CFE_SB_ValueToMsgId(BLINKY_RF_DATA_MID),
                 sizeof(BLINKY_Data.OutData));

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
    ** Subscribe to RF command packets
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(BLINKY_SEND_RF_MID), BLINKY_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Blinky: Error Subscribing to Command, RC = 0x%08lX\n", (unsigned long)status);

        return status;
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

    CFE_EVS_SendEvent(BLINKY_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "BLINKY: Calling rtems_gpio_initialize");
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

        case BLINKY_SEND_RF_MID:
            BLINKY_ReportRFTelemetry((CFE_MSG_CommandHeader_t *)SBBufPtr);
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

int32 BLINKY_ReportRFTelemetry(const CFE_MSG_CommandHeader_t *Msg){

  /*
  ** Get command execution counters...
  */
  BLINKY_Data.OutData.CommandErrorCounter = BLINKY_Data.ErrCounter;
  BLINKY_Data.OutData.CommandCounter      = BLINKY_Data.CmdCounter;

  BLINKY_Data.OutData.AppID_H = (uint8_t) ((BLINKY_HK_TLM_MID >> 8) & 0xff);
  BLINKY_Data.OutData.AppID_L = (uint8_t) (BLINKY_HK_TLM_MID & 0xff);

  for(int i=0;i<8;i++){
    if(i < 4){
      BLINKY_Data.OutData.byte_group_1[i] = BLINKY_Data.LedState[i];
    }else{
      BLINKY_Data.OutData.byte_group_2[i] = BLINKY_Data.LedState[i];
    }
  }

  for(int i=0;i<4;i++){
    BLINKY_Data.OutData.byte_group_3[i] = 0;
    BLINKY_Data.OutData.byte_group_4[i] = 0;
    BLINKY_Data.OutData.byte_group_5[i] = 0;
    BLINKY_Data.OutData.byte_group_6[i] = 0;
    BLINKY_Data.OutData.byte_group_7[i] = 0;
    BLINKY_Data.OutData.byte_group_8[i] = 0;
    BLINKY_Data.OutData.byte_group_9[i] = 0;
  }

  /*
  ** Send housekeeping telemetry packet...
  */
  CFE_SB_TimeStampMsg(CFE_MSG_PTR(BLINKY_Data.OutData.TelemetryHeader));
  CFE_SB_TransmitMsg(CFE_MSG_PTR(BLINKY_Data.OutData.TelemetryHeader), true);

  return CFE_SUCCESS;
}

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
    ** Send housekeeping telemetry packet...
    */
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(BLINKY_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(BLINKY_Data.HkTlm.TelemetryHeader), true);

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
