#ifndef _SCD41_H
#define _SCD41_H

#define CRC8_POLYNOMIAL 0x31                                                                        // Použito při výpočtu CRC.
#define CRC8_INIT 0xFF

#include <twr_i2c.h>
#include <twr_scheduler.h>

struct twr_scd41_t
{
    twr_i2c_channel_t _i2c_channel;
    uint8_t _i2c_address;
    twr_scheduler_task_id_t _task_id_interval;
    twr_scheduler_task_id_t _task_id_measure;
    void (*_event_handler)(twr_sht30_t *, twr_sht30_event_t, void *);
    void *_event_param;
    bool _measurement_active;
    twr_tick_t _update_interval;
    twr_sht30_state_t _state;
    twr_tick_t _tick_ready;
    bool _humidity_valid;
    bool _temperature_valid;
    uint16_t _reg_humidity;
    uint16_t _reg_temperature;
};

void twr_scd41_init(twr_scd41_t *self, twr_i2c_channel_t i2c_channel, uint8_t i2c_address);

void twr_scd41_deinit(twr_scd41_t *self);
