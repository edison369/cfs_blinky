/************************************************************************
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
*************************************************************************/

/**
 * @file
 *
 * Define BLINKY App Events IDs
 */

#ifndef BLINKY_EVENTS_H
#define BLINKY_EVENTS_H

#define BLINKY_RESERVED_EID          0
#define BLINKY_STARTUP_INF_EID       1
#define BLINKY_COMMAND_ERR_EID       2
#define BLINKY_COMMANDNOP_INF_EID    3
#define BLINKY_COMMANDRST_INF_EID    4
#define BLINKY_COMMANDLEDON_INF_EID  5
#define BLINKY_COMMANDLEDOFF_INF_EID 6
#define BLINKY_INVALID_MSGID_ERR_EID 7
#define BLINKY_LEN_ERR_EID           8
#define BLINKY_PIPE_ERR_EID          9

#define BLINKY_EVENT_COUNTS          9

#endif /* BLINKY_EVENTS_H */
