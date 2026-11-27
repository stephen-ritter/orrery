/************************************************************************
 * NASA Docket No. GSC-18,719-1, and identified as “core Flight System: Bootes”
 *
 * Copyright (c) 2020 United States Government as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License. You may obtain
 * a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ************************************************************************/

#include "cfe_tbl_filedef.h" /* Required to obtain the CFE_TBL_FILEDEF macro definition */
#include "sch_lab_tbl.h"
#include "cfe_sb.h" /* Required to use the CFE_SB_MSGID_WRAP_VALUE macro */

/*
** Include headers for message IDs here
*/
#include "cf_msgids.h"
#include "orrery_msgids.h" /* fcs SEND_HK (mission custom app) */

/*
** SCH Lab schedule table
** When populating this table:
**  1. The entire table is processed (SCH_LAB_MAX_SCHEDULE_ENTRIES) but entries with a
**     packet rate of 0 are skipped
**  2. You can have commented out entries or entries with a packet rate of 0
**  3. If the table grows too big, increase SCH_LAB_MAX_SCHEDULE_ENTRIES
**
** 2026-11-27 prebaked purge: the schedule speaks only to apps the wake list
** can serve. The thirteen retired modules (registry/apps.toml) left their
** SEND_HK rows with them - a scheduled poll into an un-subscribed MID is
** SB-refusal noise on every tick, and an app nobody commands does not
** deserve a heartbeat.
*/

/* clang-format: off */
SCH_LAB_ScheduleTable_t SCH_TBL_Structure =
{
    .TickRate = 10,
    .Config   = {
        {CFE_SB_MSGID_WRAP_VALUE(CFE_ES_SEND_HK_MID),     40, 0}, /* every 4.0 seconds */
        {CFE_SB_MSGID_WRAP_VALUE(CFE_EVS_SEND_HK_MID),    42, 0}, /* every 4.2 seconds */
        {CFE_SB_MSGID_WRAP_VALUE(CFE_TIME_SEND_HK_MID),   44, 0}, /* every 4.4 seconds */
        {CFE_SB_MSGID_WRAP_VALUE(CFE_SB_SEND_HK_MID),     46, 0}, /* every 4.6 seconds */
        {CFE_SB_MSGID_WRAP_VALUE(CFE_TBL_SEND_HK_MID),    48, 0}, /* every 4.8 seconds */
        {CFE_SB_MSGID_WRAP_VALUE(CF_SEND_HK_MID),         56, 0}, /* every 5.6 seconds */
        {CFE_SB_MSGID_WRAP_VALUE(CF_WAKE_UP_MID),         1,  0}, /* every 0.1 seconds (example of 10 Hz packet) */
        {CFE_SB_MSGID_WRAP_VALUE(FCS_SEND_HK_MID),         78, 0}, /* every 7.8 seconds */
        {CFE_SB_MSGID_WRAP_VALUE(OBC_HK_SEND_HK_MID),      10, 0}, /* OBC HK every 1.0 second */
        {CFE_SB_MSGID_WRAP_VALUE(SPS_SEND_HK_MID),         80, 0}, /* sps queue state every 8.0 s */
    }
};

/*
** The macro below identifies:
**    1) the data structure type to use as the table image format
**    2) the name of the table to be placed into the cFE Table File Header
**    3) a brief description of the contents of the file image
**    4) the desired name of the table image binary file that is cFE compatible
*/
CFE_TBL_FILEDEF(SCH_TBL_Structure, SCH_LAB.Schedule, Schedule Lab MsgID Table, sch_lab_table.tbl)
