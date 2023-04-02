#include <scd41.h>
#include <twr_crc.h>
#include <twr_log.h>

#define SCD41_ADDRESS_DEFAULT 0x62

#define _TWR_SCD41_DELAY_RUN 20
#define _TWR_SCD41_DELAY_INITIALIZATION 50
#define _TWR_SCD41_DELAY_MEASUREMENT 20

static void _twr_sht30_task_interval(void *param)

void twr_scd41_init(twr_sht41_t *self, twr_i2c_channel_t i2c_channel, uint8_t i2c_address)
{
    memset(self, 0, sizeof(*self));

    self->_i2c_channel = i2c_channel;
    self->_i2c_address = i2c_address;

    self->_task_id_interval = twr_scheduler_register(_twr_sht30_task_interval, self, TWR_TICK_INFINITY);
    self->_task_id_measure = twr_scheduler_register(_twr_sht30_task_measure, self, _TWR_SCD41_DELAY_RUN);

    self->_tick_ready = _TWR_SCD41_DELAY_RUN;

    twr_i2c_init(self->_i2c_channel, TWR_I2C_SPEED_100_KHZ);
}

void twr_scd41_deinit(twr_scd41_t *self)
{
    _twr_sht30_write(self, 0xa230);
    twr_scheduler_unregister(self->_task_id_interval);
    twr_scheduler_unregister(self->_task_id_measure);
}

static void _twr_sht30_task_interval(void *param)
{
    twr_sht30_t *self = param;

    twr_sht30_measure(self);

    twr_scheduler_plan_current_relative(self->_update_interval);
}
