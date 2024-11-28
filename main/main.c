#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "mqtt_client.h"


#include "lwip/err.h"
#include "lwip/sys.h"

#include "driver/i2c_master.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#define I2C_MASTER_SCL_IO           22    // Chân SCL của ESP32
#define I2C_MASTER_SDA_IO           21    // Chân SDA của ESP32
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

#define AHT20_SENSOR_ADDR           0x38
#define AHT20_CMD_CALIBRATE         0xBE
#define AHT20_CMD_TRIGGER           0xAC
#define AHT20_CMD_RESET             0xBA

// Cấu hình ADC
#define ADC_CHANNEL ADC_CHANNEL_6 // GPIO34 (hoặc thay đổi theo chân bạn dùng)
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_11
#define SAMPLE_COUNT 64

//Cấu hình Wi-Fi
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define MAX_RETRY 5


#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

const char *TAG = "Smoke Sensor";

static esp_err_t adc_init(void){
    esp_err_t ret = adc1_config_width(ADC_WIDTH);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}
float read_smoke_sensor(void) {
    int raw_reading = 0;

    // Lấy trung bình giá trị ADC
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        raw_reading += adc1_get_raw(ADC_CHANNEL);
    }
    raw_reading /= SAMPLE_COUNT;

    // Chuyển đổi sang giá trị điện áp
    float voltage = (float)raw_reading / 4095.0 * 3.3; // 3.3V là điện áp tham chiếu
    ESP_LOGI(TAG, "Raw: %d, Voltage: %.2f V", raw_reading, voltage);

    return voltage;
}

// Hàm khởi tạo I2C
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

// gửi lệnh đến aht20
static esp_err_t aht20_send_command(uint8_t command, uint8_t *data, size_t data_len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT20_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, command, true);
    if (data != NULL && data_len > 0) {
        i2c_master_write(cmd, data, data_len, true);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}


// Hàm đọc nhiệt độ và độ ẩm từ cảm biến
static esp_err_t aht20_read_data(float *temperature, float *humidity) {
    uint8_t data[6];

    // Gửi lệnh đo
    uint8_t trigger_data[2] = {0x33, 0x00};
    ESP_ERROR_CHECK(aht20_send_command(AHT20_CMD_TRIGGER, trigger_data, sizeof(trigger_data)));

    // Chờ cảm biến đo (80ms)
    vTaskDelay(pdMS_TO_TICKS(80));

    // Đọc dữ liệu
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT20_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, sizeof(data), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    // Xử lý dữ liệu
    uint32_t raw_humidity = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    uint32_t raw_temperature = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];

    *humidity = ((float)raw_humidity / 1048576) * 100;
    *temperature = ((float)raw_temperature / 1048576) * 200 - 50;

    return ESP_OK;
}

static void read_temperature_humidity(void *arg) {
    float temperature, humidity;
    while (1) {
        esp_err_t ret = aht20_read_data(&temperature, &humidity);
        if (ret == ESP_OK) {
            ESP_LOGI("AHT20", "Temperature: %.2f °C, Humidity: %.2f %%", temperature, humidity);
        } else {
            ESP_LOGE("AHT20", "Failed to read data from sensor");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


wifi_config_t wifi_config = {
    .sta = {
        .ssid = "TP-LINK_2.4GHz_4A4A",
        .password = "0987654321",
    },  
};


// Save wifi configuration to NVS
void save_wifi_config() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error (%s) opening NVS handle", esp_err_to_name(err));
        return;
    }

    err = nvs_set_blob(nvs_handle, "wifi_config", &wifi_config, sizeof(wifi_config));
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error (%s) saving NVS data", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
}

// Load wifi configuration from NVS
void load_wifi_config() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error (%s) opening NVS handle", esp_err_to_name(err));
        return;
    }

    size_t size = sizeof(wifi_config);
    err = nvs_get_blob(nvs_handle, "wifi_config", &wifi_config, &size);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error (%s) reading NVS data", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
}


// Bộ xử lý sự kiện Wi-Fi
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    static int retry_count = 0;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (retry_count < MAX_RETRY)
        {
            esp_wifi_connect();
            retry_count++;
            ESP_LOGI(TAG, "Retrying connection to Wi-Fi...");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "phong202",
            .password = "55555555",
            
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap wifi");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to wifi");
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}


void app_main(void)
{   
    esp_err_t ret;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    
    ESP_LOGI(TAG, "Initializing I2C master...");
    ESP_ERROR_CHECK(i2c_master_init());

    ESP_LOGI(TAG, "AHT20 initialized.");
    
    xTaskCreate(read_temperature_humidity, "read_temperature_humidity", 2048, (void *)I2C_MASTER_NUM, 5, NULL);

    ESP_ERROR_CHECK(adc_init());
    while (1) {
        // Đọc giá trị từ cảm biến khói
        float voltage = read_smoke_sensor();

        // Xử lý logic phát hiện khói
        if (voltage > 2.5) { // Ví dụ: Ngưỡng phát hiện khói
            ESP_LOGW(TAG, "Smoke detected!");
        } else {
            ESP_LOGI(TAG, "Air quality normal.");
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Chờ 1 giây trước lần đọc tiếp theo
    }
    
    
}
