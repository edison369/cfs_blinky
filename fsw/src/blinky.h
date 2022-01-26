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
*******************************************************************************/

/**
 * @file
 *
 * Main header file for the BLINKY application
 */

#ifndef BLINKY_H
#define BLINKY_H

/*
** Required header files.
*/
#include "cfe.h"
#include "cfe_error.h"
#include "cfe_evs.h"
#include "cfe_sb.h"
#include "cfe_es.h"

#include "blinky_perfids.h"
#include "blinky_msgids.h"
#include "blinky_msg.h"

/***********************************************************************/
#define BLINKY_PIPE_DEPTH 32 /* Depth of the Command Pipe for Application */

#define BLINKY_LED_ON        1
#define BLINKY_LED_OFF       0

#define BLINKY_NUM_LEDS      8
/************************************************************************
** Type Definitions
*************************************************************************/

/*
** Global Data
*/
typedef struct
{
    /*
    ** Command interface counters...
    */
    uint8 CmdCounter;
    uint8 ErrCounter;

    uint8 LedState[BLINKY_NUM_LEDS];

    /*
    ** Housekeeping telemetry packet...
    */
    BLINKY_HkTlm_t HkTlm;

    /*
    ** Run Status variable used in the main processing loop
    */
    uint32 RunStatus;

    /*
    ** Operational data (not reported in housekeeping)...
    */
    CFE_SB_PipeId_t CommandPipe;

    /*
    ** Initialization data (not reported in housekeeping)...
    */
    char   PipeName[CFE_MISSION_MAX_API_LEN];
    uint16 PipeDepth;

    CFE_EVS_BinFilter_t EventFilters[BLINKY_EVENT_COUNTS];

} BLINKY_Data_t;

/****************************************************************************/
/*
** Local function prototypes.
**
** Note: Except for the entry point (BLINKY_Main), these
**       functions are not called from any other source module.
*/
void  BLINKY_Main(void);
int32 BLINKY_Init(void);
void  BLINKY_ProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr);
void  BLINKY_ProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr);
int32 BLINKY_ReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg);
int32 BLINKY_ResetCounters(const BLINKY_ResetCountersCmd_t *Msg);
int32 BLINKY_LedOn(const BLINKY_LedStateCmd_t *Msg);
int32 BLINKY_LedOff(const BLINKY_LedStateCmd_t *Msg);
int32 BLINKY_Noop(const BLINKY_NoopCmd_t *Msg);

bool BLINKY_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength);

#endif /* BLINKY_H */
