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
 * Define BLINKY App  Messages and info
 */

#ifndef BLINKY_MSG_H
#define BLINKY_MSG_H

/*
** BLINKY App command codes
*/
#define BLINKY_NOOP_CC           0
#define BLINKY_RESET_COUNTERS_CC 1
#define BLINKY_LED_ON_CC         2
#define BLINKY_LED_OFF_CC        3

/*************************************************************************/

/*
** Type definition (generic "no arguments" command)
*/
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader; /**< \brief Command header */
} BLINKY_NoArgsCmd_t;

/*
** Type definition (led on/off command)
*/
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader; /**< \brief Command header */
    uint8                   LedNumber;  /**< \brief LED Number     */
} BLINKY_LedStateCmd_t;


/*
** The following commands all share the "NoArgs" format
**
** They are each given their own type name matching the command name, which
** allows them to change independently in the future without changing the prototype
** of the handler function
*/
typedef BLINKY_NoArgsCmd_t BLINKY_NoopCmd_t;
typedef BLINKY_NoArgsCmd_t BLINKY_ResetCountersCmd_t;

/*************************************************************************/
/*
** Type definition (BLINKY housekeeping)
*/

typedef struct
{
    uint8 CommandCounter;
    uint8 CommandErrorCounter;
    uint8 spare[2];
    uint8 LedState[8];
} BLINKY_HkTlm_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t  TelemetryHeader; /**< \brief Telemetry header */
    uint8_t AppID_H;
    uint8_t AppID_L;
    uint8 CommandCounter;
    uint8 CommandErrorCounter;
    uint8 spare[2];
    uint8 byte_group_1[4];    // LED 0-3
    uint8 byte_group_2[4];    // LED 4-7
    uint8 byte_group_3[4];    // empty
    uint8 byte_group_4[4];    // empty
    uint8 byte_group_5[4];    // empty
    uint8 byte_group_6[4];    // empty
    uint8 byte_group_7[4];    // empty
    uint8 byte_group_8[4];    // empty
    uint8 byte_group_9[4];    // empty
} BLINKY_OutData_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t  TelemetryHeader; /**< \brief Telemetry header */
    BLINKY_HkTlm_Payload_t     Payload;         /**< \brief Telemetry payload */
} BLINKY_HkTlm_t;

#endif /* BLINKY_MSG_H */
