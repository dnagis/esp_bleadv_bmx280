/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_sleep.h"

#include "i2c_bme280.h"
#include "bmx280.h"

static const char *TAG = "VVNX";
uint8_t temp_intpart = 99; //valeur par défaut -> 63 en hex
uint8_t temp_decpart = 99; //valeur par défaut -> 63 en hex
bool temp_pos = true; //si la température est positive true si neg false

/*
 * Bluetooth
 */

#define HCI_H4_CMD_PREAMBLE_SIZE           (4)

/*  HCI Command opcode group field(OGF) */
#define HCI_GRP_HOST_CONT_BASEBAND_CMDS    (0x03 << 10)            /* 0x0C00 */
#define HCI_GRP_BLE_CMDS                   (0x08 << 10)

#define HCI_RESET                          (0x0003 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)
#define HCI_BLE_WRITE_ADV_ENABLE           (0x000A | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_ADV_PARAMS           (0x0006 | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_ADV_DATA             (0x0008 | HCI_GRP_BLE_CMDS)

#define HCIC_PARAM_SIZE_WRITE_ADV_ENABLE        (1)
#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS    (15)
#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA      (31)

#define BD_ADDR_LEN     (6)                     /* Device address length */
typedef uint8_t bd_addr_t[BD_ADDR_LEN];         /* Device address */

#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT8_TO_STREAM(p, u8)   {*(p)++ = (uint8_t)(u8);}
#define BDADDR_TO_STREAM(p, a)   {int ijk; for (ijk = 0; ijk < BD_ADDR_LEN;  ijk++) *(p)++ = (uint8_t) a[BD_ADDR_LEN - 1 - ijk];}
#define ARRAY_TO_STREAM(p, a, len) {int ijk; for (ijk = 0; ijk < len;        ijk++) *(p)++ = (uint8_t) a[ijk];}

enum {
    H4_TYPE_COMMAND = 1,
    H4_TYPE_ACL     = 2,
    H4_TYPE_SCO     = 3,
    H4_TYPE_EVENT   = 4
};

static uint8_t hci_cmd_buf[128];

/*
 * @brief: BT controller callback function, used to notify the upper layer that
 *         controller is ready to receive command
 */
static void controller_rcv_pkt_ready(void)
{
    printf("controller rcv pkt ready\n");
}

/*
 * @brief: BT controller callback function, to transfer data packet to upper
 *         controller is ready to receive command
 */
static int host_rcv_pkt(uint8_t *data, uint16_t len)
{
    printf("host rcv pkt: ");
    for (uint16_t i = 0; i < len; i++) {
        printf("%02x", data[i]);
    }
    printf("\n");
    return 0;
}

static esp_vhci_host_callback_t vhci_host_cb = {
    controller_rcv_pkt_ready,
    host_rcv_pkt
};

static uint16_t make_cmd_reset(uint8_t *buf)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_RESET);
    UINT8_TO_STREAM (buf, 0);
    return HCI_H4_CMD_PREAMBLE_SIZE;
}

static uint16_t make_cmd_ble_set_adv_enable (uint8_t *buf, uint8_t adv_enable)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_ENABLE);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_WRITE_ADV_ENABLE);
    UINT8_TO_STREAM (buf, adv_enable);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_WRITE_ADV_ENABLE;
}

static uint16_t make_cmd_ble_set_adv_param (uint8_t *buf, uint16_t adv_int_min, uint16_t adv_int_max,
        uint8_t adv_type, uint8_t addr_type_own,
        uint8_t addr_type_dir, bd_addr_t direct_bda,
        uint8_t channel_map, uint8_t adv_filter_policy)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_PARAMS);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS );

    UINT16_TO_STREAM (buf, adv_int_min);
    UINT16_TO_STREAM (buf, adv_int_max);
    UINT8_TO_STREAM (buf, adv_type);
    UINT8_TO_STREAM (buf, addr_type_own);
    UINT8_TO_STREAM (buf, addr_type_dir);
    BDADDR_TO_STREAM (buf, direct_bda);
    UINT8_TO_STREAM (buf, channel_map);
    UINT8_TO_STREAM (buf, adv_filter_policy);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS;
}


static uint16_t make_cmd_ble_set_adv_data(uint8_t *buf, uint8_t data_len, uint8_t *p_data)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_DATA);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1);

    memset(buf, 0, HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA);

    if (p_data != NULL && data_len > 0) {
        if (data_len > HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA) {
            data_len = HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA;
        }

        UINT8_TO_STREAM (buf, data_len);

        ARRAY_TO_STREAM (buf, p_data, data_len);
    }
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1;
}

static void hci_cmd_send_reset(void)
{
    uint16_t sz = make_cmd_reset (hci_cmd_buf);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_adv_start(void)
{
    uint16_t sz = make_cmd_ble_set_adv_enable (hci_cmd_buf, 1);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_set_adv_param(void)
{
    uint16_t adv_intv_min = 256; // 160ms
    uint16_t adv_intv_max = 256; // 160ms
    uint8_t adv_type = 0; // connectable undirected advertising (ADV_IND)
    uint8_t own_addr_type = 0; // Public Device Address
    uint8_t peer_addr_type = 0; // Public Device Address
    uint8_t peer_addr[6] = {0x80, 0x81, 0x82, 0x83, 0x84, 0x85};
    uint8_t adv_chn_map = 0x07; // 37, 38, 39
    uint8_t adv_filter_policy = 0; // Process All Conn and Scan

    uint16_t sz = make_cmd_ble_set_adv_param(hci_cmd_buf,
                  adv_intv_min,
                  adv_intv_max,
                  adv_type,
                  own_addr_type,
                  peer_addr_type,
                  peer_addr,
                  adv_chn_map,
                  adv_filter_policy);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_set_adv_data(void)
{


    uint8_t adv_data[31] = {0x02, 0x01, 0x06};
    uint8_t adv_data_len;
    


    /*adv_data[3] = name_len + 1;
    for (int i = 0; i < name_len; i++) {
        adv_data[5 + i] = (uint8_t)adv_temp[i];
    }
    adv_data_len = 5 + name_len;*/
    
    adv_data[3] = 3; //taille de la partie température: un byte pour temp_pos (true si temperature positive false otherwise, et partie avant virgule, et partie après virgule
    if (temp_pos == true) adv_data[4] = 1; else adv_data[4] = 0;
    adv_data[5] = temp_intpart; //arrive en hexa de l'autre côté (hexa c'est la forme d'affichage, en fait c'est un int tout le long)
    adv_data[6] = temp_decpart; //arrive en hexa de l'autre côté (hexa c'est la forme d'affichage, en fait c'est un int tout le long)
    adv_data_len = 7;
    
    printf("Les champs température que l'on va advertiser ---------->temp_pos:%i et temp: %i.%i\n",adv_data[4], temp_intpart, temp_decpart);

    
    uint16_t sz = make_cmd_ble_set_adv_data(hci_cmd_buf, adv_data_len, (uint8_t *)adv_data);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

/*
 * @brief: send HCI commands to perform BLE advertising;
 */
//void bleAdvtTask(void *pvParameters)
void bleAdvtTask()
{
    //int cmd_cnt = 0;
    bool send_avail = false;
    esp_vhci_host_register_callback(&vhci_host_cb);
    printf("BLE advt task start\n");
    //while (1) {
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
	send_avail = esp_vhci_host_check_send_available();
	printf("BLE Advertise, flag_send_avail: %d\n", send_avail);
	
    hci_cmd_send_reset();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    hci_cmd_send_ble_set_adv_param();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    hci_cmd_send_ble_set_adv_data();
    vTaskDelay(1000 / portTICK_PERIOD_MS);    
    hci_cmd_send_ble_adv_start();
 }
//Bluetooth


/*
 * Température
 */

typedef struct {
	env_data_t env_data;
	long timestamp;
} env_data_record_t;

static env_data_record_t last_env_data[2];
static bmx280_config_t bmx280_config[2];

static void env_sensor_callback(env_data_t* env_data) {
	if (env_data->sensor_idx <= 1) {
		ESP_LOGI(TAG,"env (%d): temp : %.2f C, pressure: %.2f hPa, humidity: %.2f %%", env_data->sensor_idx, env_data->temp, env_data->pressure, env_data->humidity);
		env_data_record_t* r = last_env_data + env_data->sensor_idx;
		r->timestamp = oap_epoch_sec();
		memcpy(&last_env_data->env_data, env_data, sizeof(env_data_t));
		
		//sprintf(adv_temp,"%.2f",env_data->temp); //avant je faisait ça
		//env_data_t défini dans oap_data_env.h -> temp est un double
		//https://stackoverflow.com/questions/499939/extract-decimal-part-from-a-floating-point-number-in-c
		
		//double essai_temp_neg = -4.18; //pour faire des essais de temperature fictive
		if (env_data->temp < 0) temp_pos = false;
		temp_intpart = (int)fabs(env_data->temp); //fabs = valeur absolue d'un double (gestion des temps negs)
		temp_decpart = ((int)(fabs(env_data->temp)*100)%100);
		
	} else {
		ESP_LOGE(TAG, "env (%d) - invalid sensor", env_data->sensor_idx);
	}
}

static void env_sensors_init() {
	memset(&last_env_data, 0, sizeof(env_data_record_t)*2);
	memset(bmx280_config, 0, sizeof(bmx280_config_t)*2);

	if (bmx280_set_hardware_config(&bmx280_config[0], 0) == ESP_OK) {
		bmx280_config[0].interval = 5000;
		bmx280_config[0].callback = &env_sensor_callback;

		if (bmx280_init(&bmx280_config[0]) != ESP_OK) {
			ESP_LOGE(TAG, "couldn't initialise bmx280 sensor %d", 0);
		}
	}

	if (bmx280_set_hardware_config(&bmx280_config[1], 1) == ESP_OK) {
		bmx280_config[1].interval = 5000;
		bmx280_config[1].callback = &env_sensor_callback;

		if (bmx280_init(&bmx280_config[1]) != ESP_OK) {
			ESP_LOGE(TAG, "couldn't initialise bmx280 sensor %d", 1);
		}
	}
}

//Température

void app_main()
{
    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGI(TAG, "Bluetooth controller release classic bt memory failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGI(TAG, "Bluetooth controller initialize failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
        ESP_LOGI(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return;
    }
    
    env_sensors_init(); 


    ESP_LOGI(TAG, "on va lancer bleAdvtTask");
    bleAdvtTask();
    ESP_LOGI(TAG, "on advertise 10 secondes");
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "fin... on se revoit après un dodo bien mérité...");
    esp_sleep_enable_timer_wakeup(5 * 1000000);
	esp_deep_sleep_start();
}

