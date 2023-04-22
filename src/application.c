#include <application.h>

#include <stdio.h>

#include <time.h>

#include <twr_delay.h>


#define SEND_DATA_INTERVAL          (20 * 60 * 1000)
#define MEASURE_INTERVAL            (5 * 30 * 1000)

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
twr_led_t lcdLed;
twr_gfx_t *pgfx;

twr_cmwx1zzabz_t lora;


twr_scheduler_task_id_t write_sensor_data_id;
twr_scheduler_task_id_t get_sensor_data_id;
twr_scheduler_task_id_t measure_task_id; 
twr_scheduler_task_id_t send_lora_message_task_id;
twr_scheduler_task_id_t update_display_task_id; 

TWR_DATA_STREAM_FLOAT_BUFFER(sm_temperature_buffer, (SEND_DATA_INTERVAL / MEASURE_INTERVAL))
TWR_DATA_STREAM_FLOAT_BUFFER(sm_humidity_buffer, (SEND_DATA_INTERVAL / MEASURE_INTERVAL))
TWR_DATA_STREAM_FLOAT_BUFFER(sm_co2_buffer, (SEND_DATA_INTERVAL / MEASURE_INTERVAL))

twr_data_stream_t sm_temperature;
twr_data_stream_t sm_humidity;
twr_data_stream_t sm_co2;

uint8_t page_number = 1;

void lora_callback(twr_cmwx1zzabz_t *self, twr_cmwx1zzabz_event_t event, void *event_param);
static void measure_task(void *param);
static void send_lora_message_task(void *param);
static void update_display_task(void *param);

uint8_t sensor_common_generate_crc(const uint8_t* data, uint16_t count) {
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

static void update_display_task(void *param)
{
    if (!twr_gfx_display_is_ready(pgfx))
    {
        return;
    }

    twr_system_pll_enable();

    twr_gfx_clear(pgfx);

    twr_gfx_set_font(pgfx, &twr_font_ubuntu_24);    
    
    if(page_number > 3) page_number = 1;
    if(page_number < 1) page_number = 3; 
    
    char str[16];

    switch (page_number) {
      case 1: {
            float temperature_avg = NAN;
            twr_data_stream_get_average(&sm_temperature, &temperature_avg);

            snprintf(str, sizeof(str), "Teplota");
            int w = twr_gfx_calc_string_width(pgfx, str);
            twr_gfx_draw_string(pgfx, 64 - w / 2, 25, str, 1);

            if (!isnan(temperature_avg))
            {
                snprintf(str, sizeof(str), "%.1f °C", temperature_avg);
                int w = twr_gfx_calc_string_width(pgfx, str);
                twr_gfx_draw_string(pgfx, 64 - w / 2, 75, str, 1);
            } else {
                snprintf(str, sizeof(str), "- °C");
                int w = twr_gfx_calc_string_width(pgfx, str);
                twr_gfx_draw_string(pgfx, 64 - w / 2, 75, str, 1);
            }
        break;
      }
        
      case 2: {
            float humidity_avg = NAN;
            twr_data_stream_get_average(&sm_humidity, &humidity_avg);

            snprintf(str, sizeof(str), "Rel. vlhkost");
            int w = twr_gfx_calc_string_width(pgfx, str);
            twr_gfx_draw_string(pgfx, 64 - w / 2, 25, str, 1);

            if (!isnan(humidity_avg))
            {
                snprintf(str, sizeof(str), "%.1f %%", humidity_avg);
                int w = twr_gfx_calc_string_width(pgfx, str);
                twr_gfx_draw_string(pgfx, 64 - w / 2, 75, str, 1);
            } else {
                snprintf(str, sizeof(str), "- %%");
                int w = twr_gfx_calc_string_width(pgfx, str);
                twr_gfx_draw_string(pgfx, 64 - w / 2, 75, str, 1);
            }
        break;
      }
        
       case 3: {
            float co2_avg = NAN;
            twr_data_stream_get_average(&sm_co2, &co2_avg);

            snprintf(str, sizeof(str), "CO2");
            int w = twr_gfx_calc_string_width(pgfx, str);
            twr_gfx_draw_string(pgfx, 64 - w / 2, 25, str, 1);

            if (!isnan(co2_avg))
            {
                snprintf(str, sizeof(str), "%.0f ppm", co2_avg);
                int w = twr_gfx_calc_string_width(pgfx, str);
                twr_gfx_draw_string(pgfx, 64 - w / 2, 75, str, 1);
            } else {
                snprintf(str, sizeof(str), "- ppm");
                int w = twr_gfx_calc_string_width(pgfx, str);
                twr_gfx_draw_string(pgfx, 64 - w / 2, 75, str, 1);
            }
            
        break;
       }
    }

    twr_gfx_draw_line(pgfx, 0, 113, 128, 113, 1);

    twr_gfx_update(pgfx);   
    twr_system_pll_disable(); 
}


bool firstWrite = true;

void write_sensor_data() {
write_sensor_data_sequence:
    twr_i2c_transfer_t singleShot;
    uint8_t tx_buffer[2] = { 0x21, 0x9D };                                                           // Buffer se dvěma bajty.

    singleShot.device_address = SCD41_ADDR;                                                         // Parametry pro strukturu transfer.
    singleShot.buffer = tx_buffer;                  
    singleShot.length = sizeof(tx_buffer);                  

    bool res = twr_i2c_write(TWR_I2C_I2C0, &singleShot);                                            // Zapíše data na I2C a vrátí 1 - OK; 0 - NOK.

    if (res == true) {                                                                              // Když vysledek je true pak čti data.
        twr_scheduler_plan_relative(get_sensor_data_id, 5500);
    }
    else {
        if(firstWrite){
            firstWrite = false;
            goto write_sensor_data_sequence;
        }
        twr_log_debug("I2C failed");  
    }
}

void get_sensor_data() {
    twr_i2c_transfer_t readData;
    uint8_t rx_buffer[9];

    readData.device_address = SCD41_ADDR;
    readData.buffer = rx_buffer;            
    readData.length = sizeof(rx_buffer);            

    twr_i2c_read(TWR_I2C_I2C0, &readData);

    uint8_t crcCO2[2] = { rx_buffer[0], rx_buffer[1] };
    uint8_t crcCO2Result = sensor_common_generate_crc(crcCO2, 2);
    if (crcCO2Result == rx_buffer[2]) {
        float value = (float)((uint16_t)(rx_buffer[0] << 8 | rx_buffer[1]));   
        twr_data_stream_feed(&sm_co2, &value);
        twr_log_debug("CO2: %0.2f ppm", value);
    }           
    else {
        twr_log_debug("CO2: CRC check invalid");          
    }           

    uint8_t crcTemperature[2] = { rx_buffer[3], rx_buffer[4] };
    uint8_t crcTemperatureResult = sensor_common_generate_crc(crcTemperature, 2);           
    if (crcTemperatureResult == rx_buffer[5]) {         
        float value = (float)((uint16_t)(rx_buffer[3] << 8 | rx_buffer[4]));           
        value = -45 + 175 * (value / 65535.0f);
        twr_data_stream_feed(&sm_temperature, &value);
        twr_log_debug("Temperature: %0.2f °C", value);         
    }           
    else {          
        twr_log_debug("Temperature: CRC check invalid");          
    }           

    uint8_t crcHumidity[2] = { rx_buffer[6], rx_buffer[7] };                                        // Totéž co výše akorát jiné bajty.
    uint8_t crcHumidityResult = sensor_common_generate_crc(crcHumidity, 2);         
    if (crcHumidityResult == rx_buffer[8]) {            
        uint16_t value = ((uint16_t)(rx_buffer[6] << 8 | rx_buffer[7]));          
        float value2 = value * 100.0f / 65535.0f;
        twr_data_stream_feed(&sm_humidity, &value2);
        twr_log_debug("Humidity: %0.2f %%", value2);
    }
    else {
        twr_log_debug("Humidity: CRC check invalid");
    }

    twr_scheduler_plan_now(update_display_task_id);
}

void lcd_event_handler(twr_module_lcd_event_t event, void *event_param)
{
    if (event == TWR_MODULE_LCD_EVENT_LEFT_CLICK) {
        twr_log_debug("LEFT PRESS");
        page_number--;
        twr_scheduler_plan_now(update_display_task_id);
    }

    if (event == TWR_MODULE_LCD_EVENT_RIGHT_CLICK) {
        twr_log_debug("RIGHT PRESS");
        page_number++;
        twr_scheduler_plan_now(update_display_task_id);

    }
}

void application_init(void)
{
    twr_log_init(TWR_LOG_LEVEL_DUMP, TWR_LOG_TIMESTAMP_ABS);

    twr_data_stream_init(&sm_temperature, 1, &sm_temperature_buffer);
    twr_data_stream_init(&sm_humidity, 1, &sm_humidity_buffer);
    twr_data_stream_init(&sm_co2, 1, &sm_co2_buffer);

    const twr_led_driver_t* driver = twr_module_lcd_get_led_driver();
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
                    
    twr_i2c_init(TWR_I2C_I2C0, TWR_I2C_SPEED_100_KHZ);                                              // Inicializace I2C na 100 kHz.
                    
    twr_delay_us(65000);

    write_sensor_data_id = twr_scheduler_register(write_sensor_data, NULL, TWR_TICK_INFINITY);      // Registrace tasku pro zapsání dat.
    get_sensor_data_id   = twr_scheduler_register(get_sensor_data, NULL, TWR_TICK_INFINITY);        // Registrace tasku pro čtení dat.   
    measure_task_id   = twr_scheduler_register(measure_task, NULL, TWR_TICK_INFINITY);  
    send_lora_message_task_id   = twr_scheduler_register(send_lora_message_task, NULL, TWR_TICK_INFINITY); 
    update_display_task_id   = twr_scheduler_register(update_display_task, NULL, TWR_TICK_INFINITY); 

    twr_scheduler_plan_now(measure_task_id);
}


static void measure_task(void *param)
{
    twr_scheduler_plan_now(write_sensor_data_id);

    twr_scheduler_plan_current_relative(MEASURE_INTERVAL);
}

static void send_lora_message_task(void *param){
    static uint8_t payload_buffer[7];

    memset(payload_buffer, 0xff, sizeof(payload_buffer));

    payload_buffer[0] = header;

    float temperature_avg = NAN;

    twr_data_stream_get_average(&sm_temperature, &temperature_avg);

    if (!isnan(temperature_avg))
    {
        int16_t temperature_i16 = (int16_t) (temperature_avg);

        payload_buffer[1] = temperature_i16 >> 8;
        payload_buffer[2] = temperature_i16;
    }

    float humidity_avg = NAN;

    twr_data_stream_get_average(&sm_humidity, &humidity_avg);

    if (!isnan(humidity_avg))
    {
        int16_t humidity_i16 = (int16_t) (humidity_avg);

        payload_buffer[3] = humidity_i16 >> 8;
        payload_buffer[4] = humidity_i16;
    }

    float co2_avg = NAN;

    twr_data_stream_get_average(&sm_co2, &co2_avg);

    if (!isnan(co2_avg))
    {
        uint16_t co2_i16 = (uint16_t) (co2_avg);
        payload_buffer[5] = co2_i16 >> 8;
        payload_buffer[6] = co2_i16;
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