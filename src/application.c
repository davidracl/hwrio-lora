// Tower Kit documentation https://tower.hardwario.com/
// SDK API description https://sdk.hardwario.com/
// Forum https://forum.hardwario.com/

#include <application.h>
#include <stdio.h>
#include <time.h>
#include <twr_delay.h>

#define SEND_DATA_INTERVAL          (1 * 60 * 1000)
#define MEASURE_INTERVAL            (1 * 30 * 1000)

#define SCD41_ADDR 0x62                                                                             
#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF

enum {
    HEADER_BOOT         = 0x00,
    HEADER_UPDATE       = 0x01,
    HEADER_BUTTON_CLICK = 0x02

} header = HEADER_BOOT;

twr_module_lcd_button_t btnL = TWR_MODULE_LCD_BUTTON_LEFT;
twr_module_lcd_button_t btnR = TWR_MODULE_LCD_BUTTON_RIGHT;
twr_cmwx1zzabz_t lora;
twr_led_t lcdLed;
twr_gfx_t *pgfx;

twr_led_t led;

twr_scheduler_task_id_t battery_measure_task_id;
twr_scheduler_task_id_t measurement_task_id;
twr_scheduler_task_id_t refresh_display_task_id;
twr_scheduler_task_id_t send_lora_message_task_id;

TWR_DATA_STREAM_FLOAT_BUFFER(sm_voltage_buffer, 8)
TWR_DATA_STREAM_FLOAT_BUFFER(sm_temperature_buffer, (SEND_DATA_INTERVAL / MEASURE_INTERVAL))
TWR_DATA_STREAM_FLOAT_BUFFER(sm_humidity_buffer, (SEND_DATA_INTERVAL / MEASURE_INTERVAL))
TWR_DATA_STREAM_FLOAT_BUFFER(sm_co2_buffer, (SEND_DATA_INTERVAL / MEASURE_INTERVAL))

twr_data_stream_t sm_voltage;
twr_data_stream_t sm_temperature;
twr_data_stream_t sm_humidity;
twr_data_stream_t sm_co2;

void lora_callback(twr_cmwx1zzabz_t *self, twr_cmwx1zzabz_event_t event, void *event_param);
static void send_lora_message_task(void *param);

uint8_t generate_8bit_crc(const uint8_t* data, uint16_t count) {
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;
    /* calculates 8-Bit checksum with given polynomial */
    for (current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

bool is_crc_correct(const uint8_t data, const uint8_t crc) {
    return data == crc;
}

bool scd41_send_single_shot_measurement_command() {
    twr_log_debug("scd41_send_single_shot_measurement_command start");  
    uint8_t tx_buffer[2] = { 0x21, 0x9D };

    twr_i2c_transfer_t singleShotTransfer;
    singleShotTransfer.device_address = SCD41_ADDR;
    singleShotTransfer.buffer = tx_buffer;                  
    singleShotTransfer.length = sizeof(tx_buffer);                  

    bool state = twr_i2c_write(TWR_I2C_I2C0, &singleShotTransfer);

    twr_delay_us(20000);
    return state;
}

void scd41_read_latest_measured_data(uint8_t *destination) {
    twr_i2c_transfer_t readDataTransfer;
    // todo: memory alocation
    uint8_t rx_buffer[9];

    readDataTransfer.device_address = SCD41_ADDR;
    readDataTransfer.buffer = rx_buffer;
    readDataTransfer.length = sizeof(rx_buffer);

    twr_i2c_read(TWR_I2C_I2C0, &readDataTransfer);

//    destination = &rx_buffer;
    memcpy(destination, rx_buffer, sizeof(rx_buffer));
}

bool process_measured_data(uint8_t* measuredRawData) {
    uint8_t expectedCrc;
    float value;

    uint8_t measuredDataCo2[2] = { measuredRawData[0], measuredRawData[1] };
    expectedCrc = generate_8bit_crc(measuredDataCo2, 2);

    if (is_crc_correct(expectedCrc, measuredRawData[2])) {
        value = (float)((int16_t)(measuredRawData[0] << 8 | measuredRawData[1]));  

        twr_log_debug("CO2: %0.2f ppm", value); 

        twr_data_stream_feed(&sm_co2, &value);
    }  

    uint8_t measuredDataTemperature[2] = { measuredRawData[3], measuredRawData[4] };
    expectedCrc = generate_8bit_crc(measuredDataTemperature, 2);

    if (is_crc_correct(expectedCrc, measuredRawData[5])) {
        value = (float)((int16_t)(measuredRawData[3] << 8 | measuredRawData[4]));   
        value = -45 + 175 * (value / 65535.0f);

        twr_log_debug("Teplota: %0.2f °C", value);

        twr_data_stream_feed(&sm_temperature, &value);
    }

    uint8_t measuredDataHumidity[2] = { measuredRawData[6], measuredRawData[7] };
    expectedCrc = generate_8bit_crc(measuredDataHumidity, 2);

    if (is_crc_correct(expectedCrc, measuredRawData[8])) {
        value = (float)((int16_t)(measuredRawData[6] << 8 | measuredRawData[7]));      
        value = value * 100.0f / 65535.0f;

        twr_log_debug("Vlhkost: %0.2f %%", value);

        twr_data_stream_feed(&sm_humidity, &value);
    }     

    return true;
}

void scd41_init(){
    twr_gpio_init(TWR_GPIO_P1);
    twr_gpio_set_mode(TWR_GPIO_P1, TWR_GPIO_MODE_OUTPUT);                   
             
    twr_i2c_init(TWR_I2C_I2C0, TWR_I2C_SPEED_100_KHZ);
    twr_delay_us(20);
}

bool scd41_is_turned_on(){
    return twr_gpio_get_output(TWR_GPIO_P1);
}

static void scd41_turn_on(){
    twr_gpio_set_output(TWR_GPIO_P1, 1);
}

static void scd41_turn_off(){
    twr_gpio_set_output(TWR_GPIO_P1, 0);
}


static void measurement_task(void* param) {
    twr_log_debug("measurement_task start");  
    (void) param; 
    static uint8_t measuredDataRaw[9];

    if(!scd41_is_turned_on){
        scd41_turn_on();
    }

    if(scd41_send_single_shot_measurement_command()){
        twr_log_debug("scd41_send_single_shot_measurement_command ok");  

        // todo: think about it
        //memset(measuredDataRaw, 0xff, sizeof(measuredDataRaw));

        scd41_read_latest_measured_data(measuredDataRaw);

        // todo: if failed after X attemps, send error message, flag can be moved via params of fn
        if(!process_measured_data(measuredDataRaw)){
            twr_scheduler_plan_current_relative(3 * 1000);
        }
    };
    
    // scd41_turn_off();
    twr_log_debug("measurement_task end");

    twr_scheduler_plan_current_relative(MEASURE_INTERVAL);
    twr_scheduler_plan_now(refresh_display_task_id);
}

static void refresh_display_task(void* param) {
    (void) param;

}

void battery_event_handler(twr_module_battery_event_t event, void *event_param)
{
    if (event == TWR_MODULE_BATTERY_EVENT_UPDATE)
    {
        float voltage = NAN;

        twr_module_battery_get_voltage(&voltage);

        twr_data_stream_feed(&sm_voltage, &voltage);
    }
}

static void battery_measure_task(void *param)
{
    if (!twr_module_battery_measure())
    {
        twr_scheduler_plan_current_now();
    }
}

void lcd_event_handler(twr_module_lcd_event_t event, void *event_param) {
    
    twr_module_lcd_clear();

    if (event == TWR_MODULE_LCD_EVENT_LEFT_PRESS) {
        twr_log_debug("Levé tlačítko stisknuto.");                                              // Výpis do konzole.

        twr_led_pulse(&lcdLed, 500);                                                                // Puls LEDek.

        char txt[14] = "Leve tlacitko";                                                             // String je dlouhy 13 znaků, ale je potřeba nastavit velikost + 1 (tedy 14).
        twr_gfx_draw_string(pgfx, 10, 5, txt, true);                                                // Vypsání stringu na LCD.
        twr_gfx_draw_line(pgfx, 0, 21, 128, 23, true);                                              // Vykreslení čáry.

        twr_gfx_update(pgfx);                                                                       // Aktualizace.
    }

    if (event == TWR_MODULE_LCD_EVENT_RIGHT_PRESS) {
        twr_log_debug("Pravé tlačítko stisknuto.");

        twr_led_pulse(&lcdLed, 500);

        char txt[15] = "Prave tlacitko";
        twr_gfx_draw_string(pgfx, 10, 5, txt, true);
        twr_gfx_draw_line(pgfx, 0, 21, 128, 23, true);

        twr_gfx_update(pgfx);
    }
}

void application_init(void)
{
    twr_log_init(TWR_LOG_LEVEL_DUMP, TWR_LOG_TIMESTAMP_ABS);
    // twr_log_init(TWR_LOG_LEVEL_OFF, TWR_LOG_TIMESTAMP_ABS);
    twr_log_debug("Init");

    twr_led_init(&led, TWR_GPIO_LED, false, 0);
    twr_led_pulse(&led, 300);

    twr_module_lcd_init();
    twr_module_lcd_set_event_handler(lcd_event_handler, NULL);

    pgfx = twr_module_lcd_get_gfx();
    twr_gfx_set_font(pgfx, &twr_font_ubuntu_15);
    refresh_display_task_id = twr_scheduler_register(refresh_display_task, NULL, TWR_TICK_INFINITY);

    twr_data_stream_init(&sm_voltage, 1, &sm_voltage_buffer);
    twr_data_stream_init(&sm_temperature, 1, &sm_temperature_buffer);
    twr_data_stream_init(&sm_humidity, 1, &sm_humidity_buffer);
    twr_data_stream_init(&sm_co2, 1, &sm_co2_buffer);

    twr_module_battery_init();
    twr_module_battery_set_event_handler(battery_event_handler, NULL);
    battery_measure_task_id = twr_scheduler_register(battery_measure_task, NULL, 2020);

    twr_cmwx1zzabz_init(&lora, TWR_UART_UART1);
    twr_cmwx1zzabz_set_event_handler(&lora, lora_callback, NULL);
    twr_cmwx1zzabz_set_class(&lora, TWR_CMWX1ZZABZ_CONFIG_CLASS_A);

    scd41_init();
    measurement_task_id = twr_scheduler_register(measurement_task, NULL, TWR_TICK_INFINITY);
    send_lora_message_task_id = twr_scheduler_register(send_lora_message_task, NULL, TWR_TICK_INFINITY);

    twr_scheduler_plan_from_now(measurement_task_id, 5 * 1000);
    twr_log_debug("Init done");
}

static void send_lora_message_task(void *param){
    static uint8_t payload_buffer[9];

    memset(payload_buffer, 0xff, sizeof(payload_buffer));

    payload_buffer[0] = header;

    float battery_voltage_avg = NAN;

    twr_data_stream_get_average(&sm_voltage, &battery_voltage_avg);

    if (!isnan(battery_voltage_avg))
    {
        int16_t battery_i16 = (int16_t) (battery_voltage_avg);

        payload_buffer[1] = battery_i16 >> 8;
        payload_buffer[2] = battery_i16;
    }

    float temperature_avg = NAN;

    twr_data_stream_get_average(&sm_temperature, &temperature_avg);

    if (!isnan(temperature_avg))
    {
        int16_t temperature_i16 = (int16_t) (temperature_avg);

        payload_buffer[3] = temperature_i16 >> 8;
        payload_buffer[4] = temperature_i16;
    }

    float humidity_avg = NAN;

    twr_data_stream_get_average(&sm_humidity, &humidity_avg);

    if (!isnan(humidity_avg))
    {
        int16_t humidity_i16 = (int16_t) (humidity_avg);

        payload_buffer[5] = humidity_i16 >> 8;
        payload_buffer[6] = humidity_i16;
    }

    float co2_avg = NAN;

    twr_data_stream_get_average(&sm_co2, &co2_avg);

    if (!isnan(co2_avg))
    {
        uint16_t co2_i16 = (uint16_t) (co2_avg);
        payload_buffer[7] = co2_i16 >> 8;
        payload_buffer[8] = co2_i16;
    }

    header = HEADER_UPDATE;

    twr_cmwx1zzabz_send_message(&lora, payload_buffer, sizeof(payload_buffer));

    static char tmp[sizeof(payload_buffer) * 2 + 1];
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

    twr_scheduler_plan_now(send_lora_message_task_id);

    twr_scheduler_plan_current_relative(SEND_DATA_INTERVAL);
}

void lora_callback(twr_cmwx1zzabz_t *self, twr_cmwx1zzabz_event_t event, void *event_param)
{
    if (event == TWR_CMWX1ZZABZ_EVENT_ERROR)
    {
        twr_led_set_mode(&led, TWR_LED_MODE_BLINK_FAST);
    }
    else if (event == TWR_CMWX1ZZABZ_EVENT_SEND_MESSAGE_START)
    {
        twr_led_set_mode(&led, TWR_LED_MODE_ON);

        twr_scheduler_plan_relative(battery_measure_task_id, 20);
    }
    else if (event == TWR_CMWX1ZZABZ_EVENT_SEND_MESSAGE_DONE)
    {
        twr_led_set_mode(&led, TWR_LED_MODE_OFF);
    }
    else if (event == TWR_CMWX1ZZABZ_EVENT_READY)
    {
        twr_led_set_mode(&led, TWR_LED_MODE_OFF);
    }
    else if (event == TWR_CMWX1ZZABZ_EVENT_JOIN_SUCCESS)
    {
        twr_log_debug("$JOIN_OK");
        twr_led_set_mode(&led, TWR_LED_MODE_OFF);
    }
    else if (event == TWR_CMWX1ZZABZ_EVENT_JOIN_ERROR)
    {
        twr_log_debug("$JOIN_ERROR");
    }
}