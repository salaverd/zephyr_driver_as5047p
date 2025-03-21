
#define DT_DRV_COMPAT ams_as5047p

#include <errno.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(ams_as5047p, CONFIG_SENSOR_LOG_LEVEL);

#define AS5047P_ANGLE_REGISTER_H    0x3FFE
#define AS5047P_FULL_ANGLE	        360
#define AS5047P_PULSES_PER_REV      16384           //14 bit
#define AS5047P_MILLION_UNIT        1000000 

#define AS5047P_SPI_OPERATION            \
	(SPI_OP_MODE_MASTER | SPI_WORD_SET(16))

struct as5047p_dev_cfg {
	struct spi_dt_spec spi;
};

/* Device run time data */
struct as5047p_dev_data {
	uint16_t position;
};


static int as5047p_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct as5047p_dev_data *dev_data = dev->data;
	const struct as5047p_dev_cfg *dev_cfg = dev->config;

	uint16_t tx_data = 0x7FFE;// raw data, 0xFFFF filtered data
	uint16_t rx_data = 0;
	
	struct spi_buf tx_buf = { .buf = (uint8_t *)&tx_data, .len = sizeof(tx_data) };
	struct spi_buf rx_buf = { .buf = (uint8_t *)&rx_data, .len = sizeof(rx_data) };
	struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

	int err = spi_transceive_dt(&dev_cfg->spi, &tx, &rx);
	err = spi_transceive_dt(&dev_cfg->spi, &tx, &rx);		
	if (!err) {
		dev_data->position = rx_data & 0x3FFF; 		// 14-bit mask

	} else {
		LOG_ERR("SPI transaction failed: %d", err);
	}

	return err;
}

static int as5047p_get(const struct device *dev, enum sensor_channel chan,
			struct sensor_value *val)
{
	struct as5047p_dev_data *dev_data = dev->data;

	if (chan == SENSOR_CHAN_ROTATION) {
		val->val1 = ((int32_t)dev_data->position * AS5047P_FULL_ANGLE) /
							AS5047P_PULSES_PER_REV;

		val->val2 = (((int32_t)dev_data->position * AS5047P_FULL_ANGLE) %
			     AS5047P_PULSES_PER_REV) * (AS5047P_MILLION_UNIT / AS5047P_PULSES_PER_REV);
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int as5047p_initialize(const struct device *dev)
{
    struct as5047p_dev_data *const dev_data = dev->data;

    dev_data->position = 0;

	LOG_INF("Device %s initialized", dev->name);

	return 0;
}

static const struct sensor_driver_api as5047p_driver_api = 
{
	.sample_fetch = as5047p_fetch,
	.channel_get = as5047p_get,
};



#define AS5047P_INIT(n)						\
	static struct as5047p_dev_data as5047p_data_##n;		\
	static const struct as5047p_dev_cfg as5047p_cfg_##n = {\
		.spi = SPI_DT_SPEC_INST_GET(n, \
				AS5047P_SPI_OPERATION, \
				1), \
	};	\
									\
	SENSOR_DEVICE_DT_INST_DEFINE(n, &as5047p_initialize, NULL,	\
			    &as5047p_data_##n, &as5047p_cfg_##n, \
			    POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,	\
			    &as5047p_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AS5047P_INIT)
