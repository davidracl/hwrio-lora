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

twr_scheduler_task_id_t write_sensor_data_id;
twr_scheduler_task_id_t get_sensor_data_id;
twr_scheduler_task_id_t battery_measure_task_id;
twr_scheduler_task_id_t measurement_task_id;
twr_scheduler_task_id_t refresh_display_task_id;

TWR_DATA_STREAM_FLOAT_BUFFER(sm_voltage_buffer, 8)
TWR_DATA_STREAM_FLOAT_BUFFER(sm_temperature_buffer, (SEND_DATA_INTERVAL / MEASURE_INTERVAL))
TWR_DATA_STREAM_FLOAT_BUFFER(sm_humidity_buffer, (SEND_DATA_INTERVAL / MEASURE_INTERVAL))
TWR_DATA_STREAM_FLOAT_BUFFER(sm_co2_buffer, (SEND_DATA_INTERVAL / MEASURE_INTERVAL))

twr_data_stream_t sm_voltage;
twr_data_stream_t sm_temperature;
twr_data_stream_t sm_humidity;
twr_data_stream_t sm_co2;

void lora_callback(twr_cmwx1zzabz_t *self, twr_cmwx1zzabz_event_t event, void *event_param);

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

bool isCrcCorrect(const uint8_t* data, const uint8_t* crc) {
    return data == crc;
}

bool scd41_send_single_shot_measurement_command() {
    uint8_t tx_buffer[2] = { 0x21, 0x9D };

    twr_i2c_transfer_t singleShotTransfer;
    singleShotTransfer.device_address = SCD41_ADDR;
    singleShotTransfer.buffer = tx_buffer;                  
    singleShotTransfer.length = sizeof(tx_buffer);                  

    return twr_i2c_write(TWR_I2C_I2C0, &singleShotTransfer);
}

bool scd41_read_latest_measured_data() {
    

    return false;
}


void write_sensor_data() {
    twr_i2c_transfer_t singleShot;
    uint8_t tx_buffer[2] = { 0x21, 0x9D};                                                           // Buffer se dvěma bajty.

    singleShot.device_address = SCD41_ADDR;                                                         // Parametry pro strukturu transfer.
    singleShot.buffer = tx_buffer;                  
    singleShot.length = sizeof(tx_buffer);                  

    bool res = twr_i2c_write(TWR_I2C_I2C0, &singleShot);                                            // Zapíše data na I2C a vrátí 1 - OK; 0 - NOK.

    if (res == true) {                                                                              // Když vysledek je true pak čti data.
        twr_scheduler_plan_relative(get_sensor_data_id, 5500);
    }
    else {
        twr_atci_printfln("Chyba při zápisu dat na I2C.");  
    }
}

void get_sensor_data() {
    twr_i2c_transfer_t readData;
    uint8_t rx_buffer[9];
    float CO2Val            = 0;
    float temperatureVal    = 0;
    float humidityVal       = 0;

    readData.device_address = SCD41_ADDR;
    readData.buffer = rx_buffer;            
    readData.length = sizeof(rx_buffer);            

    twr_i2c_read(TWR_I2C_I2C0, &readData);                                                          // Čtení dat z I2C linky.

    uint8_t crcCO2[2] = { rx_buffer[0], rx_buffer[1] };                                             // Uložení 2 bajtů pro crc checksum.
    uint8_t crcCO2Result = sensor_common_generate_crc(crcCO2, 2);                                   // Uložení výsledku CRC.
    if (crcCO2Result == rx_buffer[2]) {                                                             // Když CRC kontrola je shodná s přijatým CRC bajtem.
        CO2Val = (float)((int16_t)(rx_buffer[0] << 8 | rx_buffer[1]));                              // Sjednocení 2 bajtů do wordu.
        twr_atci_printfln("CO2: %0.2f ppm", CO2Val); 
        
        twr_data_stream_feed(&sm_co2, &CO2Val);
    }           
    else {
        twr_atci_printfln("CO2: Chyba při čtení dat. CRC checksum není správné.");          
    }           

    uint8_t crcTemperature[2] = { rx_buffer[3], rx_buffer[4] };                                     // Totéž co výše akorát jiné bajty.
    uint8_t crcTemperatureResult = sensor_common_generate_crc(crcTemperature, 2);           
    if (crcTemperatureResult == rx_buffer[5]) {         
        float temperatureData = (float)((int16_t)(rx_buffer[3] << 8 | rx_buffer[4]));           
        temperatureVal = -45 + 175 * (temperatureData / 65535.0f);                                  // Výpočet hodnoty pro teplotu z datasheetu.
        twr_atci_printfln("Teplota: %0.2f °C", temperatureVal);

        twr_data_stream_feed(&sm_temperature, &temperatureVal);
    }           
    else {          
        twr_atci_printfln("Teplota: Chyba při čtení dat. CRC checksum není správné.");          
    }           

    uint8_t crcHumidity[2] = { rx_buffer[6], rx_buffer[7] };                                        // Totéž co výše akorát jiné bajty.
    uint8_t crcHumidityResult = sensor_common_generate_crc(crcHumidity, 2);         
    if (crcHumidityResult == rx_buffer[8]) {            
        float humidityData = (float)((int16_t)(rx_buffer[6] << 8 | rx_buffer[7]));          
        humidityVal = humidityData * 100.0f / 65535.0f;                                             // Výpočet hodnoty pro vlhkost z datasheetu.
        twr_atci_printfln("Vlhkost: %0.2f %%", humidityVal);

        twr_data_stream_feed(&sm_humidity, &humidityVal);
    }
    else {
        twr_atci_printfln("Vlhkost: Chyba při čtení dat. CRC checksum není správné.");
    }
}

static void scd41_start(){
    twr_gpio_set_output(TWR_GPIO_P1, 1);
}

static void scd41_stop_scd41(){
    twr_gpio_set_output(TWR_GPIO_P1, 1);
}

static void measurement_task(void* param) {
    (void) param;

    scd41_start();

    if(scd41_send_single_shot_measurement_command()){

    };
    
    scd41_stop();

    twr_scheduler_plan_current_relative(MEASURE_INTERVAL);
    twr_scheduler_plan_now(refresh_display_task_id);
}

static void refresh_display_task(void* param) {
    (void) param;

}

void lcd_event_handler(twr_module_lcd_event_t event, void *event_param) {
    
    twr_module_lcd_clear();

    if (event == TWR_MODULE_LCD_EVENT_LEFT_PRESS) {
        twr_atci_printfln("Levé tlačítko stisknuto.");                                              // Výpis do konzole.

        twr_led_pulse(&lcdLed, 500);                                                                // Puls LEDek.

        char txt[14] = "Leve tlacitko";                                                             // String je dlouhy 13 znaků, ale je potřeba nastavit velikost + 1 (tedy 14).
        twr_gfx_draw_string(pgfx, 10, 5, txt, true);                                                // Vypsání stringu na LCD.
        twr_gfx_draw_line(pgfx, 0, 21, 128, 23, true);                                              // Vykreslení čáry.

        twr_gfx_update(pgfx);                                                                       // Aktualizace.
    }

    if (event == TWR_MODULE_LCD_EVENT_RIGHT_PRESS) {
        twr_atci_printfln("Pravé tlačítko stisknuto.");

        twr_led_pulse(&lcdLed, 500);

        char txt[15] = "Prave tlacitko";
        twr_gfx_draw_string(pgfx, 10, 5, txt, true);
        twr_gfx_draw_line(pgfx, 0, 21, 128, 23, true);

        twr_gfx_update(pgfx);
    }
}

void application_init(void)
{
    twr_data_stream_init(&sm_voltage, 1, &sm_voltage_buffer);
    twr_data_stream_init(&sm_temperature, 1, &sm_temperature_buffer);
    twr_data_stream_init(&sm_humidity, 1, &sm_humidity_buffer);
    twr_data_stream_init(&sm_co2, 1, &sm_co2_buffer);

    const twr_led_driver_t* driver = twr_module_lcd_get_led_driver();
    twr_led_init_virtual(&lcdLed, TWR_MODULE_LCD_LED_BLUE, driver, 1);

    twr_module_lcd_init();

    twr_module_lcd_set_event_handler(lcd_event_handler, NULL);

    pgfx = twr_module_lcd_get_gfx();
    twr_gfx_set_font(pgfx, &twr_font_ubuntu_15);

    static const twr_atci_command_t commands[] = {
            TWR_ATCI_COMMAND_CLAC,                  
            TWR_ATCI_COMMAND_HELP                   
    };                  
                    
    twr_atci_init(commands, TWR_ATCI_COMMANDS_LENGTH(commands));

    twr_data_stream_init(&sm_voltage, 1, &sm_voltage_buffer);

    twr_gpio_init(TWR_GPIO_P1);
    twr_gpio_set_mode(TWR_GPIO_P1, TWR_GPIO_MODE_OUTPUT);                   
             
    twr_i2c_init(TWR_I2C_I2C0, TWR_I2C_SPEED_100_KHZ);

    // TODO: cleancode
    write_sensor_data_id = twr_scheduler_register(write_sensor_data, NULL, TWR_TICK_INFINITY);
    get_sensor_data_id   = twr_scheduler_register(get_sensor_data, NULL, TWR_TICK_INFINITY);
    measurement_task_id   = twr_scheduler_register(measurement_task, NULL, TWR_TICK_INFINITY);
    refresh_display_task_id   = twr_scheduler_register(refresh_display_task, NULL, TWR_TICK_INFINITY);

    twr_cmwx1zzabz_init(&lora, TWR_UART_UART1);
    twr_cmwx1zzabz_set_event_handler(&lora, lora_callback, NULL);
    twr_cmwx1zzabz_set_class(&lora, TWR_CMWX1ZZABZ_CONFIG_CLASS_A);

    twr_scheduler_plan_from_now(measurement_task_id, 5 * 1000);
}

void application_task(void)
{
    if (!twr_cmwx1zzabz_is_ready(&lora))
    {
        twr_scheduler_plan_current_relative(100);

        return;
    }

    static uint8_t payload_buffer[9];

    memset(payload_buffer, 0xff, sizeof(payload_buffer));

    payload_buffer[0] = header;

    float battery_voltage_avg = NAN;

    twr_data_stream_get_average(&sm_temperature, &battery_voltage_avg);

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

    twr_atci_printfln("$SEND: %s", tmp);
    
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
        twr_atci_printfln("$JOIN_OK");
        twr_led_set_mode(&led, TWR_LED_MODE_OFF);
    }
    else if (event == TWR_CMWX1ZZABZ_EVENT_JOIN_ERROR)
    {
        twr_atci_printfln("$JOIN_ERROR");
    }
}

void battery_measure_task(void *param)
{
    if (!twr_module_battery_measure())
    {
        twr_scheduler_plan_current_now();
    }
}