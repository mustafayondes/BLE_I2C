#include <stdio.h>
#include "esp_log.h"


#include "nvs_flash.h"	// Adding library for Bluetooth Low Energy
#include "esp_nimble_hci.h" // Bluetooth Configuration should be done from menuconfig
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "freertos/semphr.h" // Adding library for FreeRTOS Task and Mutex
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include"driver/i2c.h" // Adding library for I2C
#include"lm75a.h"
#include"aht10.h"


#define LM75A_SERVICE_UUID                  	0x1805 // Universally Unique Identifier to write services and characteristic
#define AHT10_SERVICE_UUID						0x1810
#define LM75A_CHR_UUID                  	    0x2A1F
#define AHT10_CHR_UUID							0x2A30


#define SDA_GPIO	21	//I2C pins
#define SCL_GPIO	22



#define DEVICE_NAME "MY BLE DEVICE xyz"

void ble_app_advertise(void);
float lm75a_read();
int cmpfunc(const void *a, const void *b);
float aht10_read();
void 	i2c_init();


float sensor_result;
float sensor_result2=0.0f;
uint8_t ble_addr_type;

SemaphoreHandle_t mutexBus;
SemaphoreHandle_t mutexBus2;

void task1(void *params) // FreeRTOS is used to read more data from sensors at the same time
{
  while (true)
  {
    printf("reading temperature \n");
    if (xSemaphoreTake(mutexBus, 1000 / portTICK_PERIOD_MS)) // Mutex avoid to conflict in I2C Bus. Other wise data of
    														//sensors might be sent same time.
    {
      sensor_result = lm75a_read();
      xSemaphoreGive(mutexBus);
    }
    else
    {
      printf("writing temperature timed out \n");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
void task2(void *params)
{
  while (true)
  {
    printf("reading temperature \n");
    if (xSemaphoreTake(mutexBus, 1000 / portTICK_PERIOD_MS))
    {
      sensor_result2 = aht10_read();
      xSemaphoreGive(mutexBus);
    }
    else
    {
      printf("writing temperature timed out \n");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

static int sensor_read(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
   char str[50] = {0};
   float * read_temp = &sensor_result;
   sprintf(str, "Temperature Value: %.2f", *read_temp);
   os_mbuf_append(ctxt->om, str, strlen(str));
   return 0;
}

static int sensor_read2(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
   char str[50] = {0};
   float * sensor = &sensor_result2;
   sprintf(str, "Humidity Value : %.2f", *sensor);
   os_mbuf_append(ctxt->om, str, strlen(str));
    return 0;
}

static const struct ble_gatt_svc_def gat_svcs[] = { // Write services for each sensors to read data .
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY, // Bluetooth type
        .uuid = BLE_UUID16_DECLARE(LM75A_SERVICE_UUID),  // read data OF lm75a from 0x1805 UUID
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = BLE_UUID16_DECLARE(LM75A_CHR_UUID),
                .flags = BLE_GATT_CHR_F_READ,
                .access_cb = sensor_read //  specifies a pointer to a function to be used to access the property.
            },
            {
                0
            }
        }
    },
	{
	        .type = BLE_GATT_SVC_TYPE_PRIMARY,
	        .uuid = BLE_UUID16_DECLARE(AHT10_SERVICE_UUID),//read data of aht10 from 0x1810 UUID
	        .characteristics = (struct ble_gatt_chr_def[]){
	            {
	                .uuid = BLE_UUID16_DECLARE(AHT10_CHR_UUID),
	                .flags = BLE_GATT_CHR_F_READ,
	                .access_cb = sensor_read2 //   specifies a pointer to a function to be used to access the property.

	            },
	            {
	                0
	            }
	        }
	    },

    {
        0
    }
};


static int ble_gap_event(struct ble_gap_event *event, void *arg) // Put condition to check BLE if it is connrcted or not
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:		 // if it is connected, write connect
        ESP_LOGI("GAP", "BLE_GAP_EVENT_CONNECT %s", event->connect.status == 0 ? "OK" : "Failed");
        if (event->connect.status != 0)
        {
            ble_app_advertise(); // if connection lost, broadcast advertisement
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("GAP", "BLE_GAP_EVENT_DISCONNECT"); // if it is disconnection , broadcast advertisement
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE_GAP_EVENT_ADV_COMPLETE");
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI("GAP", "BLE_GAP_EVENT_SUBSCRIBE");
        break;
    default:
        break;
    }
    return 0;
}

void ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_DISC_LTD; 	// initialization of BLE advertisement
    fields.tx_pwr_lvl_is_present = 1;								// feature of ble_hs_adv_fields struct
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)ble_svc_gap_device_name();
    fields.name_len = strlen(ble_svc_gap_device_name());
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_app_advertise(); // run BLE advertisement
}

void host_task(void *param)
{
    nimble_port_run(); // run NimBLE Bluetooth Low Energy (BLE) protocol stack.
}

void app_main(void)
{
    nvs_flash_init();

    esp_nimble_hci_init();
    nimble_port_init();

    ble_svc_gap_device_name_set(DEVICE_NAME);
    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_gatts_count_cfg(gat_svcs);
    ble_gatts_add_svcs(gat_svcs);


    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(host_task);

  mutexBus = xSemaphoreCreateMutex();
  mutexBus2 = xSemaphoreCreateMutex();
  xTaskCreate(&task1, "temperature reading", 2048, NULL, 2, NULL); // FreeRTOS TASK1 for LM75A Temperature data
  xTaskCreate(&task2, "humidity reading", 2048, NULL, 2, NULL); //FreeRTOS TASK2 for BME280 Temperature data

}


