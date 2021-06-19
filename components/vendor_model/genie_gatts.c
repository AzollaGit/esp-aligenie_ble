/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/****************************************************************************
*
* This demo showcases BLE GATT server. It can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server demo.
* Client demo will enable gatt_server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"  

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "sdkconfig.h"
 
#include "genie_gatts.h"

#define TAG   "AliGenie_GATTS"



#define PROFILE_NUM             1
#define PROFILE_APP_IDX         0
#define GATTS_APP_ID            0x01
#define SAMPLE_DEVICE_NAME      "ESP_GATTS"    //The Device Name Characteristics in GAP
#define GATTS_SVC_INST_ID	    0

// AIS服务声明为Primary Service，Service UUID为0xFEB3。
#define GATTS_CHAR_UUID_PRIMARY     0xFEB3
// AIS（Alibaba IoT Service）服务包含以下5个Characteristics
#define GATTS_CHAR_UUID_READ        0xFED4
#define GATTS_CHAR_UUID_WRITE       0xFED5
#define GATTS_CHAR_UUID_INDICATE    0xFED6
#define GATTS_CHAR_UUID_W_NORSP     0xFED7
#define GATTS_CHAR_UUID_NOTIFY      0xFED8


static uint16_t gatts_mtu_size = 512;
static uint16_t gatts_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;

static xQueueHandle cmd_cmd_queue = NULL;

static bool enable_data_ntf = false;
static bool is_connected = false;
static esp_bd_addr_t gatts_remote_bda = {0x0,};

static uint16_t gatts_handle_table[HRS_IDX_NB];

 

static uint8_t adv_gatts_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    0x0F,       // Manufacturer Specific Data Length
    0xFF,       // Manufacturer Specific Data Type
    0xA8, 0x01, // CID 0x01A8
    0xB5,       // VID & Subtype
    // bit3~0 ：阿里巴巴蓝牙规范版本号，当前版本号为5，值为0b0101
    // bit7~4 ：Subtype类型
    // 0b1000：蓝牙基础类型，mesh设备AIS广播使用此类型
    // 0b1001：蓝牙Beacon类型
    // 0b1010：蓝牙语音类型
    // 0b1011：蓝牙GATT类型，接入的BLE品类设备使用此类型
    0x36,       // FMSK: SDK提供能力Function Mask，比如安全、OTA、蓝牙版本、安全广播等
    // 1~0	蓝牙版本，00：BLE4.0； 01：BLE4.2；10：BLE5.0；11：BLE5.0以上
    // 2	0：不支持OTA；1：支持OTA   0011 0110
    // 3	0：不进行安全认证； 1：进行安全认证，详细流程参考安全认证章节 (安全认证是可选的！)
    // 4	0：一型一密；1：一机一密
    // 5	配网标识，0：未配网；1：已配网
    // 7~6	保留将来使用，全部填0
    0x95, 0xAC, 0x7B, 0x00,             // PID: 6~9产品Product ID (4Byte)，由生活物联网平台颁发。ProductId: (int)8105109 = 0x007BAC95
    0x76, 0x24, 0xfa, 0x6c, 0x14, 0x18  // MAC: 10~15蓝牙设备MAC地址 (6Byte)，唯一设备地址，由生活物联网平台颁发。DeviceName: "18146cfa2476"
};

 
void config_genie_adv_uuid(void)
{
    // FMSK...
// #if CONFIG_GENIE_OTA_SAFETY_CERT    // OTA 需要安全认证
//     adv_gatts_uuid[5] = 0x0E;  // 1110
// #else
//     adv_gatts_uuid[5] = 0x06;
// #endif

    // PID...
    adv_gatts_uuid[6] =  APP_CONFIG_TRIPLES_PRODUCT_ID & 0xFF;
    adv_gatts_uuid[7] = (APP_CONFIG_TRIPLES_PRODUCT_ID >> 8) & 0xFF;
    adv_gatts_uuid[8] = (APP_CONFIG_TRIPLES_PRODUCT_ID >> 16) & 0xFF;
    adv_gatts_uuid[9] = (APP_CONFIG_TRIPLES_PRODUCT_ID >> 24) & 0xFF;

    // MAC...
    uint8_t mac[6] = { 0 };
    sscanf(APP_CONFIG_TRIPLES_DEVICE_NAME, "%02x%02x%02x%02x%02x%02x",
           mac + 0, mac + 1, mac + 2, mac + 3, mac + 4, mac + 5);

    for (uint8_t i = 0; i < 6; i++) {  // 小端
        adv_gatts_uuid[10 + i] = mac[5 - i];
    }

    printf("\r\n\r\nadv_gatts_uuid: ");
    for (int i = 0; i < 16; i++) {
        printf("%02x ", adv_gatts_uuid[i]);
    } printf("\r\n\r\n\r\n");
}

//===================================================================================================================
//===================================================================================================================




static esp_ble_adv_params_t gatts_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gatts_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};


/*
 *  GATTS PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write_nr = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE|ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_ind = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_INDICATE;

#if 1

static const uint16_t gatts_service_uuid = GATTS_CHAR_UUID_PRIMARY;

///GATTS Service - read characteristic, read&write 
static const uint16_t gatts_read_uuid = GATTS_CHAR_UUID_READ;
static const uint8_t  gatts_read_val[GATTS_DATA_MAX_LEN] = {0x00};

///GATTS Service - write characteristic, read 
static const uint16_t gatts_write_uuid = GATTS_CHAR_UUID_WRITE;
static const uint8_t  gatts_write_val[GATTS_DATA_MAX_LEN] = {0x00};

///GATTS Service - indicate characteristic, Read/Indicate 
static const uint16_t gatts_ind_uuid = GATTS_CHAR_UUID_INDICATE;
static const uint8_t  gatts_ind_val[20] = {0x00};

///GATTS Service - Write With NoRsp characteristic, read&write without response
static const uint16_t gatts_norsp_uuid = GATTS_CHAR_UUID_W_NORSP;
static const uint8_t  gatts_norsp_val[GATTS_DATA_MAX_LEN] = {0x00};

///GATTS Service - Notify characteristic, read&write without response
static const uint16_t gatts_notify_uuid = GATTS_CHAR_UUID_NOTIFY;
static const uint8_t  gatts_notify_val[20] = {0x00};
static const uint8_t  gatts_notify_ccc[2] = {0x00, 0x00};

///Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t gatts_attr_tab[HRS_IDX_NB] =
{
    //GATTS -  Service Declaration
    [IDX_SVC]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(gatts_service_uuid), sizeof(gatts_service_uuid), (uint8_t *)&gatts_service_uuid}},

    //GATTS -  data read characteristic Declaration
    [IDX_READ_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    //GATTS -  data read characteristic Value
    [IDX_READ_VAL]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&gatts_read_uuid, ESP_GATT_PERM_READ,
    GATTS_DATA_MAX_LEN, sizeof(gatts_read_val), (uint8_t *)gatts_read_val}},

    //GATTS -  data write characteristic Declaration
    [IDX_WRITE_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //GATTS -  data write characteristic Value
    [IDX_WRITE_VAL]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&gatts_write_uuid, ESP_GATT_PERM_WRITE,
    GATTS_DATA_MAX_LEN, sizeof(gatts_write_val), (uint8_t *)gatts_write_val}},

    //GATTS -  data Indicate characteristic Declaration
    [IDX_IND_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_ind}},

    //GATTS -  data Indicate characteristic Value
    [IDX_IND_VAL]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&gatts_ind_uuid, ESP_GATT_PERM_READ,
    GATTS_DATA_MAX_LEN, sizeof(gatts_ind_val), (uint8_t *)gatts_ind_val}},

    //GATTS -  data write characteristic Declaration
    [IDX_RW_NORSP_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_nr}},

    //GATTS -  data write characteristic Value
    [IDX_RW_NORSP_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&gatts_norsp_uuid, ESP_GATT_PERM_WRITE,
    GATTS_DATA_MAX_LEN, sizeof(gatts_norsp_val), (uint8_t *)gatts_norsp_val}},

    //GATTS -  data notify characteristic Declaration
    [IDX_NOTIFY_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //GATTS -  data notify characteristic Value
    [IDX_NOTIFY_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&gatts_notify_uuid, ESP_GATT_PERM_READ,
    GATTS_DATA_MAX_LEN, sizeof(gatts_notify_val), (uint8_t *)gatts_notify_val}},

    //GATTS -  data notify characteristic - Client Characteristic Configuration Descriptor
    [IDX_NOTIFY_CFG]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(gatts_notify_ccc), (uint8_t *)gatts_notify_ccc}},
};
#endif

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;
    for (int i = 0; i < HRS_IDX_NB ; i++) {
        if (handle == gatts_handle_table[i]) {
            return i;
        }
    }
    return error;
}


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    ESP_LOGE(TAG, "GAP_EVT, event %d", event);

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT");
        esp_ble_gap_start_advertising(&gatts_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_ADV_START_COMPLETE_EVT");
        //advertising start complete event to indicate advertising start successfully or failed
        if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising start failed: %s", esp_err_to_name(err));
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                param->update_conn_params.status,
                param->update_conn_params.min_int,
                param->update_conn_params.max_int,
                param->update_conn_params.conn_int,
                param->update_conn_params.latency,
                param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT:
        ESP_LOGI(TAG, "update phy params status = %d; tx_phy = %d; rx_phy = %d;", 
                param->phy_update.status,
                param->phy_update.tx_phy, 
                param->phy_update.rx_phy);

        // esp_log_buffer_hex(TAG, param->phy_update.bda, 6);        
        
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
    uint8_t res = 0xff;

    ESP_LOGI(TAG, "GATTS_EVT = %d", event);

    switch (event) {
    	case ESP_GATTS_REG_EVT:
    	    ESP_LOGI(TAG, "%s %d", __func__, __LINE__);
        	esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);

        	ESP_LOGI(TAG, "%s %d", __func__, __LINE__);
        	esp_ble_gap_config_adv_data_raw((uint8_t *)adv_gatts_uuid, sizeof(adv_gatts_uuid));

        	ESP_LOGI(TAG, "%s %d", __func__, __LINE__);
        	esp_ble_gatts_create_attr_tab(gatts_attr_tab, gatts_if, HRS_IDX_NB, GATTS_SVC_INST_ID);
       	break;
    	case ESP_GATTS_READ_EVT:
            res = find_char_and_desr_index(p_data->read.handle);
            ESP_LOGI(TAG, "ESP_GATTS_READ_EVT : handle = %d", res);
            if (res == IDX_READ_VAL){
                // TODO: client read the characteristic
                ESP_LOGI(TAG, "IDX_READ_VAL...");
            } else if (res == IDX_IND_VAL){
                // TODO: client read the characteristic
                ESP_LOGI(TAG, "IDX_IND_VAL...");
            } else if (res == IDX_NOTIFY_VAL){
                // TODO: client read the characteristic
                ESP_LOGI(TAG, "IDX_NOTIFY_VAL...");
            }
       	 break;
    	case ESP_GATTS_WRITE_EVT: {
    	    res = find_char_and_desr_index(p_data->write.handle);
            if (p_data->write.is_prep == false) {
                ESP_LOGI(TAG, "ESP_GATTS_WRITE_EVT : handle = %d", res);
                if (res == IDX_WRITE_VAL) {
                    ESP_LOGI(TAG, "gatts_mtu_size = %d", gatts_mtu_size - 3);
                    uint8_t * write_buff = (uint8_t *)malloc(gatts_mtu_size - 3);
                    if (write_buff == NULL) {
                        ESP_LOGE(TAG, "%s malloc failed", __func__);
                        break;
                    }
                    ESP_LOGI(TAG, "IDX_WRITE_VAL->write.len = %d", p_data->write.len);
                    // esp_log_buffer_char(TAG, (char *)(p_data->write.value), p_data->write.len);

                    memset(write_buff, 0x00, (gatts_mtu_size - 3));
                    memcpy(write_buff, p_data->write.value, p_data->write.len);
 
                    xQueueSend(cmd_cmd_queue, &write_buff, 0);
                    //free(write_buff);   // 不能释放内存，不然数据会出问题！
                } else if (res == IDX_RW_NORSP_VAL){
                    ESP_LOGI(TAG, "IDX_RW_NORSP_VAL->write.len = %d", p_data->write.len);
                    esp_log_buffer_char(TAG, (char *)(p_data->write.value), p_data->write.len);
                } else {
                    // TODO:
                }
            } 
      	 	break;
    	}
    	case ESP_GATTS_EXEC_WRITE_EVT: {
    	    ESP_LOGI(TAG, "ESP_GATTS_EXEC_WRITE_EVT");
    	    if (p_data->exec_write.exec_write_flag) {
    	        //print_write_buffer();
    	        //free_write_buffer();
    	    }
    	    break;
    	}
    	case ESP_GATTS_MTU_EVT:
    	    gatts_mtu_size = p_data->mtu.mtu;
            ESP_LOGI(TAG, "gatts_mtu_size = %d", gatts_mtu_size);
    	    break;
    	case ESP_GATTS_CONF_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT");
    	    break;
    	case ESP_GATTS_UNREG_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_UNREG_EVT");
        	break;
    	case ESP_GATTS_DELETE_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_DELETE_EVT");
        	break;
    	case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_START_EVT");
        	break;
    	case ESP_GATTS_STOP_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_STOP_EVT");
        	break;
    	case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT");
    	    gatts_conn_id = p_data->connect.conn_id;
    	    spp_gatts_if = gatts_if;
    	    is_connected = true;
    	    memcpy(&gatts_remote_bda, &p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        	break;
    	case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT");
    	    is_connected = false;
    	    enable_data_ntf = false;
    	    esp_ble_gap_start_advertising(&gatts_adv_params);
    	    break;
    	case ESP_GATTS_OPEN_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_OPEN_EVT");
    	    break;
    	case ESP_GATTS_CANCEL_OPEN_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CANCEL_OPEN_EVT");
    	    break;
    	case ESP_GATTS_CLOSE_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CLOSE_EVT");
    	    break;
    	case ESP_GATTS_LISTEN_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_LISTEN_EVT");
    	    break;
    	case ESP_GATTS_CONGEST_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONGEST_EVT");
    	    break;
    	case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
    	    ESP_LOGI(TAG, "The number handle =%x",param->add_attr_tab.num_handle);
    	    if (param->add_attr_tab.status != ESP_GATT_OK){
    	        ESP_LOGE(TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
    	    }
    	    else if (param->add_attr_tab.num_handle != HRS_IDX_NB){
    	        ESP_LOGE(TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, HRS_IDX_NB);
    	    }
    	    else {
    	        memcpy(gatts_handle_table, param->add_attr_tab.handles, sizeof(gatts_handle_table));
    	        esp_ble_gatts_start_service(gatts_handle_table[IDX_SVC]);
    	    }
    	    break;
    	}
    	default:
    	    break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(TAG, "gatts event = %d, gatts if %d", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gatts_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(TAG, "Reg app failed, app_id %04x, status %d",param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gatts_profile_tab[idx].gatts_if) {
                if (gatts_profile_tab[idx].gatts_cb) {
                    gatts_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}


void gatts_cmd_task(void * arg)
{
    uint8_t * cmd_buff = NULL;
    cmd_buff = (uint8_t *)malloc(gatts_mtu_size - 3);
    if (cmd_buff == NULL) {
        ESP_LOGE(TAG, "%s malloc failed", __func__);
        goto err;
    }

    while (true) {

        vTaskDelay(50 / portTICK_PERIOD_MS);
        if (xQueueReceive(cmd_cmd_queue, &cmd_buff, portMAX_DELAY)) {
            ESP_LOGI(TAG, "cmd_buff: %s", cmd_buff);
            //esp_log_buffer_hex(TAG, cmd_buff, strlen((char *)cmd_buff));
        }
    }

err:
    free(cmd_buff);
    vTaskDelete(NULL);
}

static void gatts_cmd_task_init(void)
{
    cmd_cmd_queue = xQueueCreate(100, sizeof(uint32_t));
    xTaskCreate(gatts_cmd_task, "gatts_cmd_task", 4096, NULL, 10, NULL);
}
 
//===================================================================================================================
//===================================================================================================================

void ble_gatts_init(void)
{
    esp_err_t ret;
#if 0
    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
#endif

    config_genie_adv_uuid();

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(GATTS_APP_ID);
    if (ret){
        ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
        return;
    }
   
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    gatts_cmd_task_init();

    return;
}
