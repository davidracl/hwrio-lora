#include <application.h>
#include <stdio.h>
#include <time.h>
#include <twr_delay.h>
#include <math.h>

#define SEND_DATA_INTERVAL_AFTER_X_SAMPLES 5
#define MEASURE_INTERVAL (1 * 60 * 1000)
#define CO2_DIFFERENCE_IN_PPM_TO_SEND_IMMEDIATELY 100

#define SCD41_ADDR 0x62

#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF

#define MAX_BUFFER_SIZE SEND_DATA_INTERVAL_AFTER_X_SAMPLES
#define PAYLOAD_BUFFER_SIZE (1 + 2 * MAX_BUFFER_SIZE)
#define PAYLOAD_OFFSET_BUFFER 6

enum
{
    HEADER_BOOT = 0x00,
    HEADER_UPDATE = 0x01,
    HEADER_BUTTON_CLICK = 0x02

} header = HEADER_BOOT;

twr_module_lcd_button_t btnL = TWR_MODULE_LCD_BUTTON_LEFT;
twr_module_lcd_button_t btnR = TWR_MODULE_LCD_BUTTON_RIGHT;
twr_led_t lcdLed;
twr_gfx_t *pgfx;

twr_cmwx1zzabz_t lora;

twr_scheduler_task_id_t write_sensor_data_task_id;
twr_scheduler_task_id_t get_sensor_data_task_id;
twr_scheduler_task_id_t measure_task_id;
twr_scheduler_task_id_t send_lora_message_task_id;
twr_scheduler_task_id_t update_display_task_id;
twr_scheduler_task_id_t calibration_task_id;

TWR_DATA_STREAM_FLOAT_BUFFER(sm_temperature_buffer, MAX_BUFFER_SIZE)
TWR_DATA_STREAM_FLOAT_BUFFER(sm_humidity_buffer, MAX_BUFFER_SIZE)
TWR_DATA_STREAM_FLOAT_BUFFER(sm_co2_buffer, MAX_BUFFER_SIZE)

twr_data_stream_t sm_temperature;
twr_data_stream_t sm_humidity;
twr_data_stream_t sm_co2;

uint8_t page_number = 1;
bool firstWrite = true;
bool shouldClearBuffers = false;

typedef struct
{
    uint16_t altitude;
    uint16_t co2_target;
    float temperature_offset;
} calibration_settings_t;

bool calibrationMode = false;
calibration_settings_t calibration_setting;

void lora_callback(twr_cmwx1zzabz_t *self, twr_cmwx1zzabz_event_t event, void *event_param);
static void measure_task(void *param);
static void send_lora_message_task(void *param);
static void update_display_task(void *param);
static void calibration_task(void *param);
static void update_display_task_measuring();
static void update_display_task_calibration();

uint8_t sensor_common_generate_crc(const uint8_t *data, uint16_t count)
{
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;
    /* calculates 8-Bit checksum with given polynomial */
    for (current_byte = 0; current_byte < count; ++current_byte)
    {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

static void clearBuffers(){
    twr_data_stream_reset(&sm_temperature);
    twr_data_stream_reset(&sm_humidity);
    twr_data_stream_reset(&sm_co2);
}

static void update_display_task(void *param)
{
    if (!twr_gfx_display_is_ready(pgfx))
    {
        return;
    }

    twr_system_pll_enable();

    twr_gfx_clear(pgfx);

    twr_gfx_set_font(pgfx, &twr_font_ubuntu_24);

    if(calibrationMode == true) {
        update_display_task_calibration();
    } else {
        update_display_task_measuring();
    }

    twr_gfx_update(pgfx);
    twr_system_pll_disable();
}

static void update_display_task_calibration(){
    char str[20];

    snprintf(str, sizeof(str), "Kalibrace");
    int w = twr_gfx_calc_string_width(pgfx, str);
    twr_gfx_draw_string(pgfx, 64 - w / 2, 10, str, 1);

    twr_gfx_set_font(pgfx, &twr_font_ubuntu_15);
    switch (page_number)
    {

    case 1:
    {
        snprintf(str, sizeof(str), "CO2 target");
        w = twr_gfx_calc_string_width(pgfx, str);
        twr_gfx_draw_string(pgfx, 64 - w / 2, 30, str, 1);

        snprintf(str, sizeof(str), "%d ppm", calibration_setting.co2_target);
        w = twr_gfx_calc_string_width(pgfx, str);
        twr_gfx_draw_string(pgfx, 64 - w / 2, 60, str, 1);
        break;
    }

    case 2:
    {
        snprintf(str, sizeof(str), "Nadmorska vyska");
        w = twr_gfx_calc_string_width(pgfx, str);
        twr_gfx_draw_string(pgfx, 64 - w / 2, 30, str, 1);

        snprintf(str, sizeof(str), "%d m", calibration_setting.altitude);
        w = twr_gfx_calc_string_width(pgfx, str);
        twr_gfx_draw_string(pgfx, 64 - w / 2, 60, str, 1);
        break;
    }

    case 3:
    {
        snprintf(str, sizeof(str), "Offset teploty");
        w = twr_gfx_calc_string_width(pgfx, str);
        twr_gfx_draw_string(pgfx, 64 - w / 2, 30, str, 1);

        snprintf(str, sizeof(str), "%.1f °C", calibration_setting.temperature_offset);
        w = twr_gfx_calc_string_width(pgfx, str);
        twr_gfx_draw_string(pgfx, 64 - w / 2, 60, str, 1);
        break;
    }

    case 4:
    {
        snprintf(str, sizeof(str), "Zacit kalibraci?");
        w = twr_gfx_calc_string_width(pgfx, str);
        twr_gfx_draw_string(pgfx, 64 - w / 2, 60, str, 1);
        break;
    }

    case 5:
    {
        snprintf(str, sizeof(str), "Probiha kalibrace");
        w = twr_gfx_calc_string_width(pgfx, str);
        twr_gfx_draw_string(pgfx, 64 - w / 2, 60, str, 1);
        twr_scheduler_plan_now(calibration_task_id);
        break;
    }
    }
}

static void update_display_task_measuring(){
    
    if (page_number > 4)
        page_number = 1;

    if (page_number < 1)
        page_number = 4;

    char str[16];

    switch (page_number)
    {
    case 1:
    {
        float temperature_avg = NAN;
        twr_data_stream_get_last(&sm_temperature, &temperature_avg);

        snprintf(str, sizeof(str), "Teplota");
        int w = twr_gfx_calc_string_width(pgfx, str);
        twr_gfx_draw_string(pgfx, 64 - w / 2, 25, str, 1);

        if (!isnan(temperature_avg))
        {
            snprintf(str, sizeof(str), "%.1f °C", temperature_avg);
            int w = twr_gfx_calc_string_width(pgfx, str);
            twr_gfx_draw_string(pgfx, 64 - w / 2, 75, str, 1);
        }
        else
        {
            snprintf(str, sizeof(str), "- °C");
            int w = twr_gfx_calc_string_width(pgfx, str);
            twr_gfx_draw_string(pgfx, 64 - w / 2, 75, str, 1);
        }
        break;
    }

    case 2:
    {
        float humidity_avg = NAN;
        twr_data_stream_get_last(&sm_humidity, &humidity_avg);

        snprintf(str, sizeof(str), "Rel. vlhkost");
        int w = twr_gfx_calc_string_width(pgfx, str);
        twr_gfx_draw_string(pgfx, 64 - w / 2, 25, str, 1);

        if (!isnan(humidity_avg))
        {
            snprintf(str, sizeof(str), "%.1f %%", humidity_avg);
            int w = twr_gfx_calc_string_width(pgfx, str);
            twr_gfx_draw_string(pgfx, 64 - w / 2, 75, str, 1);
        }
        else
        {
            snprintf(str, sizeof(str), "- %%");
            int w = twr_gfx_calc_string_width(pgfx, str);
            twr_gfx_draw_string(pgfx, 64 - w / 2, 75, str, 1);
        }
        break;
    }

    case 3:
    {
        float co2_avg = NAN;
        twr_data_stream_get_last(&sm_co2, &co2_avg);

        snprintf(str, sizeof(str), "CO2");
        int w = twr_gfx_calc_string_width(pgfx, str);
        twr_gfx_draw_string(pgfx, 64 - w / 2, 25, str, 1);

        if (!isnan(co2_avg))
        {
            snprintf(str, sizeof(str), "%.0f ppm", co2_avg);
            int w = twr_gfx_calc_string_width(pgfx, str);
            twr_gfx_draw_string(pgfx, 64 - w / 2, 75, str, 1);
        }
        else
        {
            snprintf(str, sizeof(str), "- ppm");
            int w = twr_gfx_calc_string_width(pgfx, str);
            twr_gfx_draw_string(pgfx, 64 - w / 2, 75, str, 1);
        }

        break;
    }

    case 4:
    {
        float temperature_avg = NAN, humidity_avg = NAN, co2_avg = NAN;
        twr_data_stream_get_last(&sm_temperature, &temperature_avg);
        twr_data_stream_get_last(&sm_humidity, &humidity_avg);
        twr_data_stream_get_last(&sm_co2, &co2_avg);

        if (!isnan(temperature_avg))
        {
            snprintf(str, sizeof(str), "%.1f °C", temperature_avg);
            int w = twr_gfx_calc_string_width(pgfx, str);
            twr_gfx_draw_string(pgfx, 64 - w / 2, 25, str, 1);
        }
        else
        {
            snprintf(str, sizeof(str), "- °C");
            int w = twr_gfx_calc_string_width(pgfx, str);
            twr_gfx_draw_string(pgfx, 64 - w / 2, 25, str, 1);
        }

        if (!isnan(humidity_avg))
        {
            snprintf(str, sizeof(str), "%.1f %%", humidity_avg);
            int w = twr_gfx_calc_string_width(pgfx, str);
            twr_gfx_draw_string(pgfx, 64 - w / 2, 60, str, 1);
        }
        else
        {
            snprintf(str, sizeof(str), "- %%");
            int w = twr_gfx_calc_string_width(pgfx, str);
            twr_gfx_draw_string(pgfx, 64 - w / 2, 60, str, 1);
        }

        if (!isnan(co2_avg))
        {
            snprintf(str, sizeof(str), "%.0f ppm", co2_avg);
            int w = twr_gfx_calc_string_width(pgfx, str);
            twr_gfx_draw_string(pgfx, 64 - w / 2, 90, str, 1);
        }
        else
        {
            snprintf(str, sizeof(str), "- ppm");
            int w = twr_gfx_calc_string_width(pgfx, str);
            twr_gfx_draw_string(pgfx, 64 - w / 2, 90, str, 1);
        }

        break;
    }
    }
}


void write_sensor_data_task()
{
write_sensor_data_sequence:
    twr_i2c_transfer_t singleShotTransfer;
    uint8_t tx_buffer[2] = {0x21, 0x9D};

    singleShotTransfer.device_address = SCD41_ADDR;
    singleShotTransfer.buffer = tx_buffer;
    singleShotTransfer.length = sizeof(tx_buffer);

    bool res = twr_i2c_write(TWR_I2C_I2C0, &singleShotTransfer);

    if (res == true)
    {
        twr_scheduler_plan_relative(get_sensor_data_task_id, 5500);
    }
    else
    {
        if (firstWrite)
        {
            firstWrite = false;
            goto write_sensor_data_sequence;
        }
        twr_log_debug("I2C failed");
    }
}

bool isCo2ChangingFast(float previousValue, float currentValue){
    return (fabsf(currentValue - previousValue) >= CO2_DIFFERENCE_IN_PPM_TO_SEND_IMMEDIATELY);
}

void get_sensor_data_task()
{
    twr_scheduler_plan_now(update_display_task_id);

    if(shouldClearBuffers){
        clearBuffers();
        shouldClearBuffers = false;
    }

    twr_i2c_transfer_t readData;
    uint8_t rx_buffer[9];

    readData.device_address = SCD41_ADDR;
    readData.buffer = rx_buffer;
    readData.length = sizeof(rx_buffer);

    twr_i2c_read(TWR_I2C_I2C0, &readData);

    uint8_t crcCO2[2] = {rx_buffer[0], rx_buffer[1]};
    uint8_t crcCO2Result = sensor_common_generate_crc(crcCO2, 2);
    if (crcCO2Result == rx_buffer[2])
    {
        float previousCo2Value = NAN;
        twr_data_stream_get_last(&sm_co2, &previousCo2Value);

        float value = (float)((uint16_t)(rx_buffer[0] << 8 | rx_buffer[1]));
        twr_data_stream_feed(&sm_co2, &value);
        twr_log_debug("CO2: %0.2f ppm", value);
 
        if(!isnan(previousCo2Value) && isCo2ChangingFast(previousCo2Value, value)){
            twr_scheduler_plan_now(send_lora_message_task_id);
        }
    }
    else
    {
        twr_log_debug("CO2: CRC check invalid");
    }

    uint8_t crcTemperature[2] = {rx_buffer[3], rx_buffer[4]};
    uint8_t crcTemperatureResult = sensor_common_generate_crc(crcTemperature, 2);
    if (crcTemperatureResult == rx_buffer[5])
    {
        float value = (float)((uint16_t)(rx_buffer[3] << 8 | rx_buffer[4]));
        value = -45 + 175 * (value / 65535.0f);
        twr_data_stream_feed(&sm_temperature, &value);
        twr_log_debug("Temperature: %0.2f °C", value);
    }
    else
    {
        twr_log_debug("Temperature: CRC check invalid");
    }

    uint8_t crcHumidity[2] = {rx_buffer[6], rx_buffer[7]};
    uint8_t crcHumidityResult = sensor_common_generate_crc(crcHumidity, 2);
    if (crcHumidityResult == rx_buffer[8])
    {
        uint16_t value = ((uint16_t)(rx_buffer[6] << 8 | rx_buffer[7]));
        float value2 = value * 100.0f / 65535.0f;
        twr_data_stream_feed(&sm_humidity, &value2);
        twr_log_debug("Humidity: %0.2f %%", value2);
    }
    else
    {
        twr_log_debug("Humidity: CRC check invalid");
    }

    if(twr_data_stream_get_counter(&sm_temperature) >= MAX_BUFFER_SIZE){
        twr_scheduler_plan_now(send_lora_message_task_id);
    }
}

void lcd_event_handler(twr_module_lcd_event_t event, void *event_param)
{
    if (event == TWR_MODULE_LCD_EVENT_LEFT_CLICK)
    {
        twr_log_debug("LEFT PRESS");
        if(calibrationMode == false){
            page_number--;
        } else {
            if(page_number == 1){
                calibration_setting.co2_target -= 1;
            } else if (page_number == 2){
                calibration_setting.altitude -= 50;
            } else if (page_number == 3){
                calibration_setting.temperature_offset -= 0.1;
            }
        }

        twr_scheduler_plan_now(update_display_task_id);
    }

    if (event == TWR_MODULE_LCD_EVENT_RIGHT_CLICK)
    {
        twr_log_debug("RIGHT PRESS");
        if(calibrationMode == false){
            page_number++;
        } else {
            if(page_number == 1){
                calibration_setting.co2_target += 1;
            } else if (page_number == 2){
                calibration_setting.altitude += 50;
            } else if (page_number == 3){
                calibration_setting.temperature_offset += 0.1;
            }
        }

        twr_scheduler_plan_now(update_display_task_id);
    }

    if (event == TWR_MODULE_LCD_EVENT_BOTH_HOLD)
    {
        twr_log_debug("BOTH HOLD");
        if(calibrationMode == false){
            page_number = 1;
            calibration_setting.co2_target = 440;
            calibration_setting.altitude = 0;
            calibration_setting.temperature_offset = 0;
            calibrationMode = true;
        } else {
            calibrationMode = false;
            shouldClearBuffers = true;
            twr_scheduler_plan_now(measure_task_id);
        }

        twr_scheduler_plan_now(update_display_task_id);
    }

    if (event == TWR_MODULE_LCD_EVENT_LEFT_HOLD)
    {
        twr_log_debug("LEFT HOLD");
        if(calibrationMode == true){
            if(page_number == 1) return;
            page_number--;
        }

        twr_scheduler_plan_now(update_display_task_id);
    }

    if (event == TWR_MODULE_LCD_EVENT_RIGHT_HOLD)
    {
        twr_log_debug("RIGHT HOLD");
        if(calibrationMode == true){
            if(page_number == 5) return;
            page_number++;
        }

        twr_scheduler_plan_now(update_display_task_id);
    }
}

void application_init(void)
{
    twr_log_init(TWR_LOG_LEVEL_DUMP, TWR_LOG_TIMESTAMP_ABS);

    twr_data_stream_init(&sm_temperature, 1, &sm_temperature_buffer);
    twr_data_stream_init(&sm_humidity, 1, &sm_humidity_buffer);
    twr_data_stream_init(&sm_co2, 1, &sm_co2_buffer);

    const twr_led_driver_t *driver = twr_module_lcd_get_led_driver();
    twr_led_init_virtual(&lcdLed, TWR_MODULE_LCD_LED_BLUE, driver, 1);

    twr_module_lcd_init();

    twr_module_lcd_set_event_handler(lcd_event_handler, NULL);

    pgfx = twr_module_lcd_get_gfx();
    twr_gfx_set_font(pgfx, &twr_font_ubuntu_15);

    twr_cmwx1zzabz_init(&lora, TWR_UART_UART1);
    twr_cmwx1zzabz_set_event_handler(&lora, lora_callback, NULL);
    twr_cmwx1zzabz_set_class(&lora, TWR_CMWX1ZZABZ_CONFIG_CLASS_A);

    twr_gpio_init(TWR_GPIO_P1);
    twr_gpio_set_mode(TWR_GPIO_P1, TWR_GPIO_MODE_OUTPUT);
    twr_gpio_set_output(TWR_GPIO_P1, 1);

    twr_gpio_init(TWR_GPIO_P0);
    twr_gpio_set_mode(TWR_GPIO_P0, TWR_GPIO_MODE_INPUT);

    twr_i2c_init(TWR_I2C_I2C0, TWR_I2C_SPEED_100_KHZ);

    twr_delay_us(65000);

    write_sensor_data_task_id = twr_scheduler_register(write_sensor_data_task, NULL, TWR_TICK_INFINITY);
    get_sensor_data_task_id = twr_scheduler_register(get_sensor_data_task, NULL, TWR_TICK_INFINITY);
    measure_task_id = twr_scheduler_register(measure_task, NULL, TWR_TICK_INFINITY);
    send_lora_message_task_id = twr_scheduler_register(send_lora_message_task, NULL, TWR_TICK_INFINITY);
    update_display_task_id = twr_scheduler_register(update_display_task, NULL, TWR_TICK_INFINITY);
    calibration_task_id = twr_scheduler_register(calibration_task, NULL, TWR_TICK_INFINITY);
}

static void measure_task(void *param)
{
    if(calibrationMode) return;

    twr_scheduler_plan_now(write_sensor_data_task_id);

    twr_scheduler_plan_current_relative(MEASURE_INTERVAL);
}

static void send_lora_message_task(void *param)
{
    int number_of_samples = twr_data_stream_get_counter(&sm_temperature);
    int size = 1;

    if(header == HEADER_UPDATE) size = 1 + PAYLOAD_OFFSET_BUFFER * number_of_samples;

    uint8_t payload_buffer[size];

    memset(payload_buffer, 0xff, sizeof(payload_buffer));

    payload_buffer[0] = header;

    for (int sample_index = 1; sample_index <= number_of_samples; sample_index++)
    {
        const int offset_index = (sample_index - 1) * PAYLOAD_OFFSET_BUFFER;
        float temperature_avg = NAN;
        
        twr_data_stream_get_nth(&sm_temperature, -sample_index, &temperature_avg);

        if (!isnan(temperature_avg))
        {
            int16_t temperature_i16 = (int16_t)(temperature_avg * 10.f);

            payload_buffer[1 + offset_index] = temperature_i16 >> 8;
            payload_buffer[2 + offset_index] = temperature_i16;
        }

        float humidity_avg = NAN;

        twr_data_stream_get_nth(&sm_humidity, -sample_index, &humidity_avg);

        if (!isnan(humidity_avg))
        {
            int16_t humidity_i16 = (int16_t)(humidity_avg * 10.f);

            payload_buffer[3 + offset_index] = humidity_i16 >> 8;
            payload_buffer[4 + offset_index] = humidity_i16;
        }

        float co2_value = NAN;

        twr_data_stream_get_nth(&sm_co2, -sample_index, &co2_value);

        if (!isnan(co2_value))
        {
            uint16_t co2_i16 = (uint16_t)(co2_value);
            payload_buffer[5 + offset_index] = co2_i16 >> 8;
            payload_buffer[6 + offset_index] = co2_i16;
        }
    }
    
    if(header == HEADER_UPDATE){
        shouldClearBuffers = true;
    }

    header = HEADER_UPDATE;

    twr_cmwx1zzabz_send_message(&lora, payload_buffer, sizeof(payload_buffer));

    char tmp[sizeof(payload_buffer) * 2 + 1];
    for (size_t i = 0; i < sizeof(payload_buffer); i++)
    {
        sprintf(tmp + i * 2, "%02x", payload_buffer[i]);
    }

    twr_log_debug("$SEND: %s", tmp);
}

void application_task(void)
{
    if (!twr_cmwx1zzabz_is_ready(&lora))
    {
        twr_scheduler_plan_current_relative(100);

        return;
    }
    
    get_automatic_self_calibration_state();
    get_sensor_altitude();
    get_sensor_temperature_offset();

    reinit_settings_scd41();

    twr_scheduler_plan_now(send_lora_message_task_id);
    twr_scheduler_plan_now(measure_task_id);
}

void reinit_settings_scd41(){
    twr_log_debug("$REINIT_SETTINGS_SCD41");

    twr_i2c_transfer_t transfer;
    uint8_t tx_buffer[5] = {0x36, 0x46};

    transfer.device_address = SCD41_ADDR;
    transfer.buffer = tx_buffer;
    transfer.length = sizeof(tx_buffer);

    bool res = twr_i2c_write(TWR_I2C_I2C0, &transfer);

    twr_delay_us(20000);
}


void persist_settings(){
    twr_i2c_transfer_t transfer;
    uint8_t tx_buffer[5] = {0x36, 0x15};

    transfer.device_address = SCD41_ADDR;
    transfer.buffer = tx_buffer;
    transfer.length = sizeof(tx_buffer);

    bool res = twr_i2c_write(TWR_I2C_I2C0, &transfer);

    // sleep 800 ms
    for (size_t i = 0; i < 13; i++)
    {
        twr_delay_us(61539);
    }
}

void perform_forced_recalibration()
{
    twr_log_debug("$PERFORM_FOCED_RECALIBRATION START");

    twr_i2c_transfer_t transfer;

    uint8_t tx_buffer[5] = {0x36, 0x2f, 0x01, 0xb8, 0x73}; // 440 ppm

    int16_t co2_target = (int16_t)(calibration_setting.co2_target);
    uint8_t inputParameters[] = { (uint8_t)(co2_target >> 8), (uint8_t)co2_target };
    tx_buffer[2] = inputParameters[0];
    tx_buffer[3] = inputParameters[1];
    tx_buffer[4] = sensor_common_generate_crc(inputParameters, 2);

    transfer.device_address = SCD41_ADDR;
    transfer.buffer = tx_buffer;
    transfer.length = sizeof(tx_buffer);

    bool res = twr_i2c_write(TWR_I2C_I2C0, &transfer);

    if (res == true)
    {
        twr_log_debug("I2C OK");
    }
    else
    {
        twr_log_debug("I2C FAIL");
    }

    // sleep 400 ms
    for (size_t i = 0; i < 7; i++)
    {
        twr_delay_us(57143);
    }

    twr_i2c_transfer_t readData;
    uint8_t rx_buffer[3];

    readData.device_address = SCD41_ADDR;
    readData.buffer = rx_buffer;
    readData.length = sizeof(rx_buffer);

    twr_i2c_read(TWR_I2C_I2C0, &readData);

    char tmp[sizeof(rx_buffer) * 2 + 1];
    for (size_t i = 0; i < sizeof(rx_buffer); i++)
    {
        sprintf(tmp + i * 2, "%02x", rx_buffer[i]);
    }

    twr_log_debug("$PERFORM_FORCED_CALIBRATION: %s", tmp);
}

void set_automatic_self_calibration_disabled()
{
    twr_i2c_transfer_t transfer;
    uint8_t tx_buffer[5] = {0x24, 0x16, 0x00, 0x00, 0x81};

    transfer.device_address = SCD41_ADDR;
    transfer.buffer = tx_buffer;
    transfer.length = sizeof(tx_buffer);

    bool res = twr_i2c_write(TWR_I2C_I2C0, &transfer);

    if (res == true)
    {
        twr_log_debug("I2C OK");
    }
    else
    {
        twr_log_debug("I2C FAIL");
    }

    twr_delay_us(1000);
}


void get_automatic_self_calibration_state()
{
    twr_i2c_transfer_t transfer;
    uint8_t tx_buffer[2] = {0x23, 0x13};

    transfer.device_address = SCD41_ADDR;
    transfer.buffer = tx_buffer;
    transfer.length = sizeof(tx_buffer);

    bool res = twr_i2c_write(TWR_I2C_I2C0, &transfer);

    if (res == true){
        twr_log_debug("I2C OK");
    } else {
        twr_log_debug("I2C FAIL");
    }

    twr_delay_us(1000);

    twr_i2c_transfer_t readData;
    uint8_t rx_buffer[3];

    readData.device_address = SCD41_ADDR;
    readData.buffer = rx_buffer;
    readData.length = sizeof(rx_buffer);

    twr_i2c_read(TWR_I2C_I2C0, &readData);

    char tmp[sizeof(rx_buffer) * 2 + 1];
    for (size_t i = 0; i < sizeof(rx_buffer); i++)
    {
        sprintf(tmp + i * 2, "%02x", rx_buffer[i]);
    }

    twr_log_debug("$SELF_CALIBRATION_STATE: %s", tmp);
}

void write_sensor_temperature_offset()
{
    twr_i2c_transfer_t transfer;
    uint8_t tx_buffer[5] = {0x24, 0x1d, 0x00, 0x00, 0x81};
    //uint8_t tx_buffer[5] = {0x24, 0x1d, 0x05, 0xda, 0x29};

    int16_t temperature_offset = (int16_t)(calibration_setting.temperature_offset * 65535.0f / 175.0f);
    uint8_t inputParameters[] = { (uint8_t)(temperature_offset >> 8), (uint8_t)temperature_offset };
    tx_buffer[2] = inputParameters[0];
    tx_buffer[3] = inputParameters[1];
    tx_buffer[4] = sensor_common_generate_crc(inputParameters, 2);

    transfer.device_address = SCD41_ADDR;
    transfer.buffer = tx_buffer;
    transfer.length = sizeof(tx_buffer);

    bool res = twr_i2c_write(TWR_I2C_I2C0, &transfer);

    if (res == true)
    {
        twr_log_debug("I2C OK");
    }
    else
    {
        twr_log_debug("I2C FAIL");
    }

    twr_delay_us(1000);
}

void get_sensor_temperature_offset()
{
    twr_i2c_transfer_t transfer;
    uint8_t tx_buffer[2] = {0x23, 0x18};

    transfer.device_address = SCD41_ADDR;
    transfer.buffer = tx_buffer;
    transfer.length = sizeof(tx_buffer);

    bool res = twr_i2c_write(TWR_I2C_I2C0, &transfer);

    if (res == true){
        twr_log_debug("I2C OK");
    } else {
        twr_log_debug("I2C FAIL");
    }

    twr_delay_us(1000);

    twr_i2c_transfer_t readData;
    uint8_t rx_buffer[3];

    readData.device_address = SCD41_ADDR;
    readData.buffer = rx_buffer;
    readData.length = sizeof(rx_buffer);

    twr_i2c_read(TWR_I2C_I2C0, &readData);

    char tmp[sizeof(rx_buffer) * 2 + 1];
    for (size_t i = 0; i < sizeof(rx_buffer); i++)
    {
        sprintf(tmp + i * 2, "%02x", rx_buffer[i]);
    }

    twr_log_debug("$SENSOR_TEMPERATURE_OFFSET: %s", tmp);
}

void write_sensor_altitude()
{
    twr_i2c_transfer_t transfer;
    uint8_t tx_buffer[5] = {0x24, 0x27, 0x19, 0x00, 0x2C};

    int16_t altitude = (int16_t)(calibration_setting.altitude);
    uint8_t inputParameters[] = { (uint8_t)(altitude >> 8), (uint8_t)altitude };
    tx_buffer[2] = inputParameters[0];
    tx_buffer[3] = inputParameters[1];
    tx_buffer[4] = sensor_common_generate_crc(inputParameters, 2);

    transfer.device_address = SCD41_ADDR;
    transfer.buffer = tx_buffer;
    transfer.length = sizeof(tx_buffer);

    bool res = twr_i2c_write(TWR_I2C_I2C0, &transfer);

    if (res == true)
    {
        twr_log_debug("I2C OK");
    }
    else
    {
        twr_log_debug("I2C FAIL");
    }

    twr_delay_us(1000);
}


void get_sensor_altitude()
{
    twr_i2c_transfer_t transfer;
    uint8_t tx_buffer[2] = {0x23, 0x22};

    transfer.device_address = SCD41_ADDR;
    transfer.buffer = tx_buffer;
    transfer.length = sizeof(tx_buffer);

    bool res = twr_i2c_write(TWR_I2C_I2C0, &transfer);

    if (res == true){
        twr_log_debug("I2C OK");
    } else {
        twr_log_debug("I2C FAIL");
    }

    twr_delay_us(1000);

    twr_i2c_transfer_t readData;
    uint8_t rx_buffer[3];

    readData.device_address = SCD41_ADDR;
    readData.buffer = rx_buffer;
    readData.length = sizeof(rx_buffer);

    twr_i2c_read(TWR_I2C_I2C0, &readData);

    char tmp[sizeof(rx_buffer) * 2 + 1];
    for (size_t i = 0; i < sizeof(rx_buffer); i++)
    {
        sprintf(tmp + i * 2, "%02x", rx_buffer[i]);
    }

    twr_log_debug("$SENSOR_ALTITUDE: %s", tmp);
}

static void calibration_task(void *param){
    set_automatic_self_calibration_disabled();
    get_automatic_self_calibration_state();
    
    write_sensor_altitude();
    get_sensor_altitude();

    write_sensor_temperature_offset();
    get_sensor_temperature_offset();

    perform_forced_recalibration();

    persist_settings();

    twr_log_debug("$CALIBRATION DONE");
    calibrationMode = false;
    page_number = 1;
    clearBuffers();
    twr_scheduler_plan_relative(update_display_task_id, 100);
    twr_scheduler_plan_relative(measure_task_id, 200);
}

void lora_callback(twr_cmwx1zzabz_t *self, twr_cmwx1zzabz_event_t event, void *event_param)
{
    if (event == TWR_CMWX1ZZABZ_EVENT_ERROR)
    {
    }
    else if (event == TWR_CMWX1ZZABZ_EVENT_SEND_MESSAGE_START)
    {
    }
    else if (event == TWR_CMWX1ZZABZ_EVENT_SEND_MESSAGE_DONE)
    {
    }
    else if (event == TWR_CMWX1ZZABZ_EVENT_READY)
    {
    }
    else if (event == TWR_CMWX1ZZABZ_EVENT_JOIN_SUCCESS)
    {
        twr_log_debug("$JOIN_OK");
    }
    else if (event == TWR_CMWX1ZZABZ_EVENT_JOIN_ERROR)
    {
        twr_log_debug("$JOIN_ERROR");
    }
}