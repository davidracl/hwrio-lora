// Tower Kit documentation https://tower.hardwario.com/
// SDK API description https://sdk.hardwario.com/
// Forum https://forum.hardwario.com/

#include <application.h>

#include <stdio.h>

#include <time.h>

#include <twr_delay.h>

#define SCD41_ADDR 0x62                                                                             // Adresa senzoru SCD41 na sběrnici I2C

#define CRC8_POLYNOMIAL 0x31                                                                        // Použito při výpočtu CRC.
#define CRC8_INIT 0xFF

twr_module_lcd_button_t btnL = TWR_MODULE_LCD_BUTTON_LEFT;                                          // Parametry k LCD.
twr_module_lcd_button_t btnR = TWR_MODULE_LCD_BUTTON_RIGHT;
twr_led_t lcdLed;

twr_gfx_t *pgfx;

twr_scheduler_task_id_t write_sensor_data_id;                                                       // ID tasku.
twr_scheduler_task_id_t get_sensor_data_id;

float CO2Val            = 0;                                                                        // Předpřipravené proměnné pro uložení dat.
float temperatureVal    = 0;
float humidityVal       = 0;

uint8_t sensor_common_generate_crc(const uint8_t* data, uint16_t count) {                           // Kontrola CRC. Kód z datasheetu.
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
    twr_i2c_transfer_t readData;                                                                    // Struktura pro čtení dat.
    uint8_t rx_buffer[9];                                                                           // Buffer pro příjem 9 bajtů.

    readData.device_address = SCD41_ADDR;                                                           // Adresa SCD modulu.
    readData.buffer = rx_buffer;            
    readData.length = sizeof(rx_buffer);            

    twr_i2c_read(TWR_I2C_I2C0, &readData);                                                          // Čtení dat z I2C linky.

    uint8_t crcCO2[2] = { rx_buffer[0], rx_buffer[1] };                                             // Uložení 2 bajtů pro crc checksum.
    uint8_t crcCO2Result = sensor_common_generate_crc(crcCO2, 2);                                   // Uložení výsledku CRC.
    if (crcCO2Result == rx_buffer[2]) {                                                             // Když CRC kontrola je shodná s přijatým CRC bajtem.
        CO2Val = (float)((int16_t)(rx_buffer[0] << 8 | rx_buffer[1]));                              // Sjednocení 2 bajtů do wordu.
        twr_atci_printfln("CO2: %0.2f ppm", CO2Val);                                                // Výpis do konzoly.
    }           
    else {                                                                                          // V případě, že CRC nesouhlasi.
        twr_atci_printfln("CO2: Chyba při čtení dat. CRC checksum není správné.");          
    }           

    uint8_t crcTemperature[2] = { rx_buffer[3], rx_buffer[4] };                                     // Totéž co výše akorát jiné bajty.
    uint8_t crcTemperatureResult = sensor_common_generate_crc(crcTemperature, 2);           
    if (crcTemperatureResult == rx_buffer[5]) {         
        float temperatureData = (float)((int16_t)(rx_buffer[3] << 8 | rx_buffer[4]));           
        temperatureVal = -45 + 175 * (temperatureData / 65535.0f);                                  // Výpočet hodnoty pro teplotu z datasheetu.
        twr_atci_printfln("Teplota: %0.2f °C", temperatureVal);         
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
    }
    else {
        twr_atci_printfln("Vlhkost: Chyba při čtení dat. CRC checksum není správné.");
    }
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
    const twr_led_driver_t* driver = twr_module_lcd_get_led_driver();                               // LED na LCD modulu.
    twr_led_init_virtual(&lcdLed, TWR_MODULE_LCD_LED_BLUE, driver, 1);                              // Inicializace LED.

    twr_module_lcd_init();                                                                          // Inicializace LCD.

    twr_module_lcd_set_event_handler(lcd_event_handler, NULL);                                      // Event pro tlačitka.

    pgfx = twr_module_lcd_get_gfx();
    twr_gfx_set_font(pgfx, &twr_font_ubuntu_15);

    static const twr_atci_command_t commands[] = {                                                  // Bez tohoto nejede sériová linka.
            TWR_ATCI_COMMAND_CLAC,                  
            TWR_ATCI_COMMAND_HELP                   
    };                  
                    
    twr_atci_init(commands, TWR_ATCI_COMMANDS_LENGTH(commands));                                    // Inicializace sériové linky.
                    
    twr_gpio_init(TWR_GPIO_P1);                                                                     // Inicializace GPIO.
    twr_gpio_set_mode(TWR_GPIO_P1, TWR_GPIO_MODE_OUTPUT);                   
                    
    twr_gpio_set_output(TWR_GPIO_P1, 1);                                                            // Napájení pro SCD41.
                    
    twr_i2c_init(TWR_I2C_I2C0, TWR_I2C_SPEED_100_KHZ);                                              // Inicializace I2C na 100 kHz.
                    
    twr_delay_us(65000);                                                                            // Po inicializaci je potřeba čekat nějakou dobu,
    twr_delay_us(65000);                                                                            // ale nikde není psáno kolik.
    twr_delay_us(65000);                                                                            // Tady tento delay je dostatečný.
    twr_delay_us(65000);

    write_sensor_data_id = twr_scheduler_register(write_sensor_data, NULL, TWR_TICK_INFINITY);      // Registrace tasku pro zapsání dat.
    get_sensor_data_id   = twr_scheduler_register(get_sensor_data, NULL, TWR_TICK_INFINITY);        // Registrace tasku pro čtení dat.

    write_sensor_data();                                                                            // Prvotní spuštění funkce. Při prvním spuštění nejsou dostupná data.
}

void application_task(void)
{
    twr_scheduler_plan_now(write_sensor_data_id);                                                   // Start task write_sensor_data_id.
    
    twr_scheduler_plan_current_relative(10000);                                                     // Volání funkce co 10 s. Méně jak 10 s nedoporučuji.
}