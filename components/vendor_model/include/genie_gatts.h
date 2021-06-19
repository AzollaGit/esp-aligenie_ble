// Copyright (C) 2018-2020 Alibaba Group Holding Limited
// Adaptations to ESP-IDF Copyright (c) 2020 Espressif Systems (Shanghai) Co. Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _GENIE_GATTS_H_
#define _GENIE_GATTS_H_

#ifdef __cplusplus
extern "C" {
#endif /**< __cplusplus */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"

#if 0
#define APP_CONFIG_TRIPLES_PRODUCT_ID       CONFIG_TRIPLES_PRODUCT_ID
#define APP_CONFIG_TRIPLES_DEVICE_NAME      CONFIG_TRIPLES_DEVICE_NAME
#define APP_CONFIG_TRIPLES_DEVICE_SECRET    CONFIG_TRIPLES_DEVICE_SECRET
#else 
#define APP_CONFIG_TRIPLES_PRODUCT_ID       8105109
#define APP_CONFIG_TRIPLES_DEVICE_NAME      "18146cfa2476"
#define APP_CONFIG_TRIPLES_DEVICE_SECRET    "9d6092e210ce30902edbb0d5e8426402"
#endif

/*
 * DEFINES
 ****************************************************************************************
 */
 
#define gatts_sprintf(s,...)         sprintf((char*)(s), ##__VA_ARGS__)
#define GATTS_DATA_MAX_LEN           (244)  // 512
#define GATTS_STATUS_MAX_LEN         (20)
#define GATTS_DATA_BUFF_MAX_LEN      (2*1024)

/* Attributes State Machine */
enum
{
    IDX_SVC,

    IDX_READ_CHAR,
    IDX_READ_VAL,

    IDX_WRITE_CHAR,
    IDX_WRITE_VAL,

    IDX_IND_CHAR,
    IDX_IND_VAL,

    IDX_RW_NORSP_CHAR,
    IDX_RW_NORSP_VAL,

    IDX_NOTIFY_CHAR,
    IDX_NOTIFY_VAL,
    IDX_NOTIFY_CFG,

    HRS_IDX_NB,
};

/*
 * AliGenie GATTS OTA protocol command defines
 ****************************************************************************************
 */
 
#define DEV_FIRMWARE_VERSION          0x00000002      // "00.00.02"  

#define APP_QUERY_VERSION_CMD               0x20      // 0x20：手机查询设备固件版本
#define DEV_REPORTED_VERSION_CMD            0x21      // 0x21：设备上报设备固件版本，版本为0x00000002
#define APP_QUERY_UPDATE_CMD                0x22      // 0x22：手机下发升级请求，升级请求包含五个字段，分别为：固件类型、固件版本号、固件大小、CRC16、升级标示符。
#define DEV_REPORTED_UPDATE_CMD             0x23      // 0x23：设备应答升级请求，1：允许升级 0：不可以升级。
#define DEV_REPORTED_FRAME_CMD              0x24      // 0x24：设备上报当前已接收的帧序号和接收的数据长度
#define APP_INFORM_UPDATE_END_CMD           0x25      // 0x25：手机通知设备固件下发结束，可以做固件检查 
#define DEV_REPORTED_FRAME_RESUIT_CMD       0x26      // 0x26：设备收完包后，上报固件检查结果； 1：固件检查成功；0：固件检查失败
#define APP_LSSUE_FIRM_DATA_CMD             0x2F      // 0x2F：手机下发固件的分包数据

/*
 * function declaration
 ****************************************************************************************
 */

void ble_gatts_init(void);

#ifdef __cplusplus
}
#endif /**< __cplusplus */

#endif /* _GENIE_GATTS_H_ */