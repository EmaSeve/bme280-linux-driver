#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/bitops.h>
#include <linux/kthread.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>

MODULE_AUTHOR("EmaSeve");
MODULE_DESCRIPTION("BME280 I2C DRIVER AOS");
MODULE_LICENSE("GPL");

#include "bme280_aos.h"

#define BME280_DRV_NAME "bme280-aos"
#define POLL_MIN_US 500
#define POLL_MAX_US 1000

static int osr_to_bits(unsigned int osr);
static int filter_to_bits(unsigned int filter);
static int standby_to_bits(unsigned int standby_us);
static unsigned long typical_measurement_time_us(struct bme280_data *data);
static unsigned long max_measurement_time_us(struct bme280_data *data);
static u8 build_ctrl_meas(struct bme280_data *data, u8 mode);
static int bme280_read_measurement(struct bme280_data *data, struct bme280_raw_sample *raw);
static int bme280_read_calibration(struct bme280_data *data);
static s32 bme280_read_chip_id(struct i2c_client *client);
static void bme280_compensate(struct bme280_data *data, const struct bme280_raw_sample *raw, 
                              struct bme280_sample *sample);

/* ============================== SHARED MODE CONTROL =============================== */

/* Poll REG_STATUS until the measuring bit reaches the requested state.
 * target = true waits for a measurement to start, while target = false
 * waits for a measurement to complete. */
static int wait_measure_register(struct bme280_data *data, bool target, unsigned long timeout_ms) {
    unsigned long timeout;
    int status;

    timeout = jiffies + msecs_to_jiffies(timeout_ms);

    do {
        
        status = i2c_smbus_read_byte_data(data->client, REG_STATUS);
        if (status < 0)
            return status;

        /* REG_STATUS bit 3 indicates whether a measurement is in progress. */
        if (!!(status & BIT(3)) == target)
            return 0;

        usleep_range(POLL_MIN_US, POLL_MAX_US);

    } while (time_before(jiffies, timeout));

    return -ETIMEDOUT;
}

/* Switch sensor mode: write to 'ctrl_meas' register */
static int set_mode(struct bme280_data *data, u8 mode){
    u8 ctrl_meas = build_ctrl_meas(data, mode);

    return i2c_smbus_write_byte_data(data->client, REG_CTRL_MEAS, ctrl_meas);
}

static int bme280_configure(struct bme280_data *data) {
    struct i2c_client *client = data->client;
    int buffered_mode = data->buffered_mode;

    int ret;
    int ctrl_hum;
    int config;
    int filter_bits;
    int standby_bits;

    /* CONFIG must be written while sleeping. */
    ret = set_mode(data, MODE_SLEEP);
    if (ret<0) return ret;

    // WARNING: check parameters validity in the write_raw function
    // Check if it's a possible oversampling factor
    //if ((osr_bit_temperature<0) || (osr_bit_pressure<0) || (osr_bit_humidity<0))
      //      return -EINVAL;

    /* REG_CONFIG register layout: 
     *
     * in forced mode filter and standby time is irrelevant.
     */
    filter_bits = (buffered_mode) ? filter_to_bits(data->filter) : 0;
    standby_bits = (buffered_mode) ? standby_to_bits(data->standby_us) : 0;
    // WARNING: check validity outside (i.e. write_raw()) ?
    if (filter_bits<0 || standby_bits<0)
        return -EINVAL;
    
    config = (standby_bits << 5) | (filter_bits << 2);
    ret = i2c_smbus_write_byte_data(client, REG_CONFIG, config); 
    if (ret < 0) return ret;


    /* Humidity oversampling (0, 1, 2, 4, 8, 16)
     * Warning: changes to this register become effective only after
     * a write operation to ctrl_meas. */
    ctrl_hum = osr_to_bits(data->osr_humidity);
    ret = i2c_smbus_write_byte_data(client, REG_CTRL_HUM, ctrl_hum);
    if (ret < 0) return ret;

    /* CTRL_MEAS register layout:
    *
    *  bit:   7   6   5 |  4   3   2 | 1   0
    *        -----------+------------+-------
    *          osr_temp |osr_pressure| mode
    *
    * set_mode() writes the ctrl_meas and set the mode to MODE_SLEEP if the sensor is 
    * working in MODE_FORCED, otherwise put back MODE_NORMAL. */
    return set_mode(data, buffered_mode ? MODE_NORMAL : MODE_SLEEP);
}

/* ================================== FORCED MODE =================================== */

/* Wait for a MODE_FORCED measurement to complete.
 * Sleep for the typical conversion time first, then poll REG_STATUS
 * for the remaining time up to the maximum expected conversion time. */
static int bme280_wait_measurement(struct bme280_data *data) {
    unsigned long timeout_ms;
    unsigned long typical_meas_time_us;
    unsigned long max_meas_time_us;
    unsigned long residual_timeout_ms = 3;

    typical_meas_time_us = typical_measurement_time_us(data);
    max_meas_time_us = max_measurement_time_us(data); 

    timeout_ms = DIV_ROUND_UP(max_meas_time_us - typical_meas_time_us, 1000) 
                + residual_timeout_ms;

    fsleep(typical_meas_time_us);
    
    return wait_measure_register(data, false, timeout_ms);
}

/* Switch to forced mode */
static int bme280_trigger_measurement(struct bme280_data *data) {
    return set_mode(data, MODE_FORCED); 
}

static int bme280_acquire_raw_measurement(struct bme280_data *data, struct bme280_raw_sample *raw) {
    int ret;

    mutex_lock(&data->lock);

    /* This function works only in MODE_FORCED. */
    if (data->buffered_mode) {
        ret = -EBUSY;
        goto out;
    }

    ret = bme280_trigger_measurement(data);
    if (ret < 0) goto out;
 
    ret = bme280_wait_measurement(data);
    if (ret < 0) goto out;

    ret = bme280_read_measurement(data, raw);

out:
    mutex_unlock(&data->lock);

    return ret;
}

/* =============================== NORMAL MODE  ================================== */

/* Read the latest BME280 measurement, compensate the three channels
 * and push them as a single scan into the IIO buffer. */
static int bme280_push_sample(struct iio_dev *indio_dev) {
    struct bme280_data *data = iio_priv(indio_dev);
    struct bme280_raw_sample raw;
    struct bme280_sample sample;
    struct bme280_scan scan = {};
    int ret;

    mutex_lock(&data->lock);

    ret = bme280_read_measurement(data, &raw);
    if (ret < 0)
        goto out;

    bme280_compensate(data, &raw, &sample);

    scan.temperature = sample.temperature * 10; /* milli-C */
    scan.pressure = DIV_ROUND_CLOSEST(sample.pressure, 256); /* Pa */
    scan.humidity = (sample.humidity * 1000) >> 10; /* milli-%RH */

out:
    mutex_unlock(&data->lock);

    if (ret < 0)
        return ret;

    return iio_push_to_buffers_with_timestamp(indio_dev, &scan, iio_get_time_ns(indio_dev));
}

/* Wait for the next complete measurement cycle in normal mode.
 * First wait for the measuring bit to be set, then wait for it to clear,
 * ensuring that the following read belongs to a newly completed sample. */
static int bme280_wait_next_measurement(struct bme280_data *data) {
    unsigned long cycle_timeout_ms;
    unsigned long measurement_timeout_ms;
    unsigned long residual_time_ms = 1;
    unsigned long max_meas_time_us;
    int ret;

    max_meas_time_us = max_measurement_time_us(data);
    measurement_timeout_ms = DIV_ROUND_UP(max_meas_time_us, 1000) + residual_time_ms;

    cycle_timeout_ms = measurement_timeout_ms + DIV_ROUND_UP(data->standby_us, 1000) + residual_time_ms;

    /* Wait for the next conversion to start. */
    ret = wait_measure_register(data, true, cycle_timeout_ms);
    if (ret < 0)
        return ret;

    /* Wait for that conversion to finish. */
    return wait_measure_register(data, false, measurement_timeout_ms);
}

/* Capture thread used while the IIO buffer is enabled.
 * In normal mode, wait for each new measurement to complete
 * and push the resulting T/P/H scan into the IIO software buffer. */
static int bme280_buffer_thread(void *arg) {
    struct iio_dev *indio_dev = arg;
    struct bme280_data *data = iio_priv(indio_dev);
    int ret;

    while (!kthread_should_stop()) {

        ret = bme280_wait_next_measurement(data);
        if (ret < 0) {
            if (kthread_should_stop())
                break;

            dev_warn(&data->client->dev, "Failed waiting for measurement\n");
            continue;
        }

        ret = bme280_push_sample(indio_dev);
        
        if (ret == -EBUSY) {
            dev_dbg(&data->client->dev, "IIO buffer full, sample dropped\n");
        } else if (ret < 0) {
            dev_warn(&data->client->dev, "Failed to push buffered sample: %d\n", ret);
        }
    }

    return 0;
}

static int bme280_buffer_enable(struct iio_dev *indio_dev){
    struct bme280_data *data = iio_priv(indio_dev);
    int ret;

    mutex_lock(&data->lock);
    
    data->buffered_mode = true;
    ret = bme280_configure(data);
    
    mutex_unlock(&data->lock);

    if (ret < 0) goto err;

    data->task = kthread_run(bme280_buffer_thread, indio_dev, "buffer");

    if (IS_ERR(data->task)) {
        ret = PTR_ERR(data->task);
        data->task = NULL;
        goto err;
    }

    dev_info(&data->client->dev, "IIO buffer enabled, entering normal mode\n");
    return 0;

err:
    mutex_lock(&data->lock);
    
    data->buffered_mode = false;
    ret = bme280_configure(data);
    if (ret < 0)
        dev_err(&data->client->dev, "Failed to restore direct configuration\n");
    
    mutex_unlock(&data->lock);
    return ret;
}

static int bme280_buffer_disable(struct iio_dev *indio_dev){
    struct bme280_data *data = iio_priv(indio_dev);
    int ret;

    if (data->task) {
        kthread_stop(data->task);
        data->task = NULL;
    }

    mutex_lock(&data->lock);

    data->buffered_mode = false;
    ret = bme280_configure(data);
    
    mutex_unlock(&data->lock);

    dev_info(&data->client->dev, "IIO buffer disabled, returning to direct mode\n");
    return ret;
}

static const struct iio_buffer_setup_ops bme280_buffer_ops = {
    .postenable = bme280_buffer_enable,
    .predisable = bme280_buffer_disable,
};

/* ================================= IIO INTERFACE ================================== */

static const struct iio_chan_spec bme280_iio_channels[] = {
    {   // Channel measurement type 
        .type = IIO_TEMP,

        // Declares the attributes exposed by this channel.
        // Their current values are retrieved through the read_raw() callback.
        .info_mask_separate =
            BIT(IIO_CHAN_INFO_PROCESSED) |
            BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),

        // - Supported oversampling ratios are common to all channels -
        // Declares which channel attributes expose a set of supported values.
        // The supported values are retrieved through the read_avail() callback.
        .info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
    
        .scan_index = 0,
        .scan_type = {
            .sign = 's',
            .realbits = 32,
            .storagebits = 32,
            .endianness = IIO_CPU,
        }
    },
    {
        .type = IIO_PRESSURE,
        
        .info_mask_separate = 
            BIT(IIO_CHAN_INFO_PROCESSED) | 
            BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
        
        .info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
   
        .scan_index = 1,
        .scan_type = {
            .sign = 'u',
            .realbits = 32,
            .storagebits = 32,
            .endianness = IIO_CPU,
        }
    },
    {
        .type = IIO_HUMIDITYRELATIVE,
        
        .info_mask_separate = 
            BIT(IIO_CHAN_INFO_PROCESSED) | 
            BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
        
        .info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
    
        .scan_index = 2,
        .scan_type = {
            .sign = 'u',
            .realbits = 32,
            .storagebits = 32,
            .endianness = IIO_CPU,
        }
    },

    IIO_CHAN_SOFT_TIMESTAMP(3),
};

static int bme280_iio_read_raw(struct iio_dev *indio_dev,
                               const struct iio_chan_spec *chan,
                               int *val,
                               int *val2,
                               long mask) {
    struct bme280_data *data = iio_priv(indio_dev);
    struct bme280_raw_sample raw;
    struct bme280_sample sample;
    int ret;

    switch(mask) {
    case IIO_CHAN_INFO_PROCESSED:
        ret = bme280_acquire_raw_measurement(data, &raw);
        if (ret < 0) return ret;

        bme280_compensate(data, &raw, &sample);

        switch (chan->type) {
        case IIO_TEMP:
            /* sample.temperature is in centi-C.
            * IIO temperature input uses milli-C. */
            *val = sample.temperature * 10;
            return IIO_VAL_INT;

        case IIO_PRESSURE:
            /* sample.pressure is Q24.8 Pa.
            * pressure [kPa] =
            * sample.pressure / (256 * 1000) */
            *val = sample.pressure;
            *val2 = 256000;
            return IIO_VAL_FRACTIONAL;

        case IIO_HUMIDITYRELATIVE:
            /* sample.humidity is Q22.10 %RH.
            * IIO humidity input uses milli-%RH. */
            *val = (sample.humidity * 1000) >> 10;
            return IIO_VAL_INT;

        default:
            return -EINVAL;
        }

    // Return the current oversampling ratio for this channel. 
    case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
        mutex_lock(&data->lock);
        switch(chan->type) {
        case IIO_TEMP:
            *val = data->osr_temperature;
            break;

        case IIO_PRESSURE:
            *val = data->osr_pressure;
            break;

        case IIO_HUMIDITYRELATIVE:
            *val = data->osr_humidity;
            break;
        
        default:
            mutex_unlock(&data->lock);
            return -EINVAL;
        }

        mutex_unlock(&data->lock);

        return IIO_VAL_INT;

    default:
        return -EINVAL;
    }
}

static int bme280_iio_write_raw(struct iio_dev *indio_dev, 
                                const struct iio_chan_spec *chan,
                                int val,
                                int val2,
                                long mask){
    if (mask != IIO_CHAN_INFO_OVERSAMPLING_RATIO || val2!=0 || (osr_to_bits(val)<0)) 
        return -EINVAL;
   
    struct bme280_data *data = iio_priv(indio_dev);
    int ret;
    unsigned int *osr;
    unsigned int old_osr;

    // Serialize configuration changes with measurement acquisition
    mutex_lock(&data->lock);

    switch(chan->type) {
    case IIO_TEMP:
        osr = &data->osr_temperature;
        break;
    case IIO_PRESSURE:
        osr = &data->osr_pressure;
        break;
    case IIO_HUMIDITYRELATIVE:
        osr = &data->osr_humidity;
        break;
    default: 
        mutex_unlock(&data->lock);
        return -EINVAL;
    }
    
    old_osr = *osr;
    *osr = val;

    ret = bme280_configure(data);
    if (ret < 0) {
        *osr = old_osr;

        if (bme280_configure(data) < 0)
            dev_err(&data->client->dev, "Failed to restore previous configuration\n");
    }

    mutex_unlock(&data->lock);

    return ret;
}

static const int bme280_osr_available[] = {
    1, 2, 4, 8, 16
};

static int bme280_iio_read_avail(struct iio_dev *indio_dev,
                                 const struct iio_chan_spec *chan,
                                 const int **vals,
                                 int *type,
                                 int *length,
                                 long mask)
{
    if (mask != IIO_CHAN_INFO_OVERSAMPLING_RATIO)
        return -EINVAL;

    *vals = bme280_osr_available;
    *type = IIO_VAL_INT;
    *length = ARRAY_SIZE(bme280_osr_available);

    return IIO_AVAIL_LIST;
}

static const struct iio_info bme280_iio_info = {
    .read_raw = bme280_iio_read_raw,
    .write_raw = bme280_iio_write_raw,
    .read_avail = bme280_iio_read_avail,
};

/* ================================ DRIVER LIFECYCLE ================================ */

static int bme280_probe(struct i2c_client *client) {
    struct iio_dev *indio_dev;
    struct bme280_data *data;
    s32 chip_id;
    int ret;

    /* Init */
    dev_info(&client->dev,
             "probe() called: address=0x%02x, adapter=%d\n",
             client->addr,
             i2c_adapter_id(client->adapter));
    
    /* Check functions support */
    if (!i2c_check_functionality(client->adapter,
            I2C_FUNC_SMBUS_BYTE_DATA |
            I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
        dev_err(&client->dev,
                "Required I2C/SMBus functionality not supported\n");
        return -EOPNOTSUPP;
    }

    /* Check that device chip ID is correct */
    chip_id = bme280_read_chip_id(client);
    if (chip_id < 0) return chip_id;

    dev_info(&client->dev, "BME280 detected, chip ID: 0x%02x\n", chip_id);

    /* Allocate the IIO device and the private driver data */
    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
    if (!indio_dev) return -ENOMEM;

    data = iio_priv(indio_dev);
    data->client = client;
    mutex_init(&data->lock);
    i2c_set_clientdata(client, indio_dev);

    /* Read sensor calibration data */
    ret = bme280_read_calibration(data);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read calibration data\n");
        return ret;
    }

    /* Initialize the default sensor configuration:
    * - temperature, pressure and humidity oversampling: x1
    * - IIR filter disabled
    * - sleep mode between forced measurements */
    data->osr_temperature = 1;
    data->osr_pressure = 1;
    data->osr_humidity = 1;
    /* Configuration used when iio buffered is enabled */
    data->filter = 0;
    data->standby_us = 125000;
    data->task = NULL;
    data->buffered_mode = false;

    /* Configure sensor */
    ret = bme280_configure(data);
    if (ret < 0) return ret;
    
    /* Register iio_dev */
    indio_dev->name = BME280_DRV_NAME;
    indio_dev->info = &bme280_iio_info;
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->channels = bme280_iio_channels;
    indio_dev->num_channels = ARRAY_SIZE(bme280_iio_channels);

    // IIO buffer
    ret = devm_iio_kfifo_buffer_setup(&client->dev, indio_dev, &bme280_buffer_ops);
    if (ret < 0) return ret;

    ret = devm_iio_device_register(&client->dev, indio_dev);
    if (ret < 0)
        return dev_err_probe(&client->dev, ret, "Failed to register IIO device\n");

    return 0;
}

static void bme280_remove(struct i2c_client *client) {
    dev_info(&client->dev, "remove() called \n");
}

/* ============================ LOW-LEVEL SENSOR FUNCTIONS =========================== */

/* Converts the oversampling factor to the bits representation
 * used to set the sensor */
static int osr_to_bits(unsigned int osr) {
    switch(osr) {
    case 1: return 0x01;
    case 2: return 0x02;
    case 4: return 0x03;
    case 8: return 0x04;
    case 16: return 0x05;
    default: return -EINVAL;
    }
}

static int filter_to_bits(unsigned int filter) {
    switch (filter) {
    case 0:  return 0x00;
    case 2:  return 0x01;
    case 4:  return 0x02;
    case 8:  return 0x03;
    case 16: return 0x04;
    default: return -EINVAL;
    }
}

static int standby_to_bits(unsigned int standby_us) {
    switch (standby_us) {
    case 500:     return 0x00;
    case 62500:   return 0x01;
    case 125000:  return 0x02;
    case 250000:  return 0x03;
    case 500000:  return 0x04;
    case 1000000: return 0x05;
    case 10000:   return 0x06;
    case 20000:   return 0x07;
    default:       return -EINVAL;
    }
}

/* Measurement time depends on the oversampling settings.
 * From BME280 datasheet:
 *
 * t_meas_typ [ms] = 1 + 2 * T_oversampling + (2 * P_oversampling + 0.5)
 *                  + (2 * H_oversampling + 0.5)
 */
static unsigned long typical_measurement_time_us(struct bme280_data *data) {
    return 1000 + 2000 * data->osr_temperature + (2000 * data->osr_pressure + 500) +
           (2000 * data->osr_humidity + 500);
}

/* Measurement time depends on the oversampling settings.
 * From BME280 datasheet:
 *
 * t_meas_max [ms] = 1.25 + 2.3 * T_oversampling + (2.3 * P_oversampling + 0.575)
 *                  + (2.3 * H_oversampling + 0.575) 
 */
static unsigned long max_measurement_time_us(struct bme280_data *data) {
    return 1250 + 2300 * data->osr_temperature + (2300 * data->osr_pressure + 575) +
           (2300 * data->osr_humidity + 575);
}

/* Decode a little-endian 16-bit value from two consecutive bytes. */
static u16 read_u16_le(const u8 *buf) {
    return (u16)buf[0] | ((u16)buf[1] << 8);
}

/* Sign-extend a 12-bit two's complement value to s16. */
static s16 sign_extend_12(u16 value) {
    // Check if value < 0: 0x0800 is the 12th bit (sign)
	if (value & 0x0800)
		value |= 0xF000;

	return (s16)value;
}

static u8 build_ctrl_meas(struct bme280_data *data, u8 mode) {
    u8 osr_bit_temperature = osr_to_bits(data->osr_temperature);
    u8 osr_bit_pressure = osr_to_bits(data->osr_pressure);
    
    return (osr_bit_temperature<<5) | (osr_bit_pressure<<2) | mode;
}

static int bme280_read_measurement(struct bme280_data *data, 
                                       struct bme280_raw_sample *raw) {
    u8 buf[RAW_DATA_LEN];
    int ret;

    ret = i2c_smbus_read_i2c_block_data(data->client, REG_RAW_DATA, RAW_DATA_LEN, buf);
    if (ret < 0) return ret;
    if (ret != RAW_DATA_LEN) return -EIO;

    raw->pressure = ((u32)buf[0]<<12) | ((u32)buf[1]<<4) | ((u32)buf[2]>>4);
    raw->temperature = ((u32)buf[3]<<12) | ((u32)buf[4]<<4) | ((u32)buf[5]>>4);
    raw->humidity = ((u16)buf[6]<<8) | ((u16)buf[7]);

    return 0;
}

static int bme280_read_calibration(struct bme280_data *data) {
    struct i2c_client *client = data->client;
    struct bme280_calib *calib = &data->calib;
    u8 calib1[CALIB1_LEN];
    u8 calib2[CALIB2_LEN];
    u16 raw_h4;
    u16 raw_h5;
    int ret;

    ret = i2c_smbus_read_i2c_block_data(client, REG_CALIB1, CALIB1_LEN,
                                        calib1);

    if (ret < 0) return ret;
    if (ret != CALIB1_LEN) return -EIO;

    calib->t1 = read_u16_le(&calib1[0]);
    calib->t2 = (s16)read_u16_le(&calib1[2]);
	calib->t3 = (s16)read_u16_le(&calib1[4]);

	calib->p1 = read_u16_le(&calib1[6]);
	calib->p2 = (s16)read_u16_le(&calib1[8]);
	calib->p3 = (s16)read_u16_le(&calib1[10]);
	calib->p4 = (s16)read_u16_le(&calib1[12]);
	calib->p5 = (s16)read_u16_le(&calib1[14]);
	calib->p6 = (s16)read_u16_le(&calib1[16]);
	calib->p7 = (s16)read_u16_le(&calib1[18]);
	calib->p8 = (s16)read_u16_le(&calib1[20]);
	calib->p9 = (s16)read_u16_le(&calib1[22]);

	calib->h1 = calib1[25];

    ret = i2c_smbus_read_i2c_block_data(client, REG_CALIB2, CALIB2_LEN,
                                        calib2);
    if (ret < 0) return ret;
    if (ret != CALIB2_LEN) return -EIO;

    calib->h2 = (s16)read_u16_le(&calib2[0]);
    calib->h3 = calib2[2];

    // Keep the first 4 bit of calib2[4] and shift by 4 bit calib2[3]
    raw_h4 = ((u16)calib2[3]<<4) | (calib2[4] & 0x0F);
    // H5[11:4] is stored in 0xE6 and H5[3:0] in bits 7:4 of 0xE5 
    raw_h5 = ((u16)calib2[5] << 4) | (calib2[4] >> 4);

    calib->h4 = sign_extend_12(raw_h4);
    calib->h5 = sign_extend_12(raw_h5);
    
    calib->h6 = (s8)calib2[6]; 
    
    return 0;
}

static s32 bme280_read_chip_id(struct i2c_client *client) {
    s32 chip_id;

    chip_id = i2c_smbus_read_byte_data(client, REG_CHIP_ID);
    
    /* Check read error */
    if (chip_id < 0) {
        dev_err(&client->dev, "Failed to read chip ID\n");
        return chip_id;
    }

    /* Check if the discovered device is the one inteded by the driver */
    if (chip_id != CHIP_ID) {
        dev_err(&client->dev, "Unexpected chip ID: 0x%02x\n", chip_id);
        return -ENODEV;
    }
    
    return chip_id;
}

static s32 bme280_compensate_temperature(struct bme280_data *data, s32 adc_temp, s32 *t_fine) {
    const struct bme280_calib *calib = &data->calib;
    s32 var1;
    s32 var2;
    s32 delta;

    var1 = (((adc_temp >> 3) - ((s32)calib->t1 << 1)) * (s32)calib->t2) >> 11;
    delta = (adc_temp >> 4) - (s32)calib->t1;
    var2 = (((delta * delta) >> 12) * (s32)calib->t3) >> 14;
    *t_fine = var1 + var2;

    return (*t_fine * 5 + 128) >> 8;
}

static u32 bme280_compensate_pressure(struct bme280_data *data, s32 adc_press, s32 t_fine) {
    const struct bme280_calib *calib = &data->calib;
    s64 x;
    s64 x2;
    s64 offset;
    s64 sensitivity;
    s64 corr_quad;
    s64 corr_lin;
    s64 p;

    x = (s64)t_fine - 128000;
    x2 = x * x;

    /* Pressure offset compensation */
    offset = x2 * (s64)calib->p6;
    offset += (x * (s64)calib->p5) << 17;
    offset += (s64)calib->p4 << 35;

    /* Pressure sensitivity compensation */
    sensitivity = (x2 * (s64)calib->p3) >> 8;
    sensitivity += (x * (s64)calib->p2) << 12;
    sensitivity = ((((s64)1 << 47) + sensitivity) * (s64)calib->p1) >> 33;

    if (sensitivity == 0)
        return 0;

    /* Pressure calculation */
    p = (s64)1048576 - adc_press;
    p = (((p << 31) - offset) * 3125) / sensitivity;

    /* Nonlinear corrections */
    corr_quad = ((s64)calib->p9 * (p >> 13) * (p >> 13)) >> 25;
    corr_lin = ((s64)calib->p8 * p) >> 19;

    p = ((p + corr_quad + corr_lin) >> 8) + ((s64)calib->p7 << 4);

    return (u32)p;
}

static u32 bme280_compensate_humidity(struct bme280_data *data, s32 adc_hum, s32 t_fine) {
    const struct bme280_calib *calib = &data->calib;
    s32 x;
    s32 adc_corr;
    s32 term1;
    s32 term2a;
    s32 term2b;
    s32 gain;
    s32 nonlinear;
    s32 h;

    x = t_fine - 76800;

    /* Correct raw humidity using H4 and H5 */
    adc_corr = (adc_hum << 14) - ((s32)calib->h4 << 20) - ((s32)calib->h5 * x) + 16384;
    term1 = adc_corr >> 15;

    /* Temperature-dependent humidity gain */
    term2a = (x * (s32)calib->h6) >> 10;
    term2b = ((x * (s32)calib->h3) >> 11) + 32768;

    gain = ((term2a * term2b) >> 10) + 2097152;
    gain = (gain * (s32)calib->h2 + 8192) >> 14;

    h = term1 * gain;

    /* Nonlinear correction */
    nonlinear = (((((h >> 15) * (h >> 15)) >> 7) * (s32)calib->h1) >> 4);

    h -= nonlinear;

    /* Clamp to physical range */
    if (h < 0)
        h = 0;
    else if (h > 419430400)
        h = 419430400;

    return (u32)(h >> 12);
}

static void bme280_compensate(struct bme280_data *data,
                              const struct bme280_raw_sample *raw,
                              struct bme280_sample *sample) {
	s32 t_fine;
	sample->temperature = bme280_compensate_temperature(data, raw->temperature, &t_fine);
	sample->pressure = bme280_compensate_pressure(data, raw->pressure, t_fine);
	sample->humidity = bme280_compensate_humidity(data, raw->humidity, t_fine);
}

/* ============================== MODULE REGISTRATION =============================== */

/* Compatible devices */
static const struct of_device_id bme280_aos_of_match[] = {
    {.compatible = "aos,bme280-aos" },
    {}
};

MODULE_DEVICE_TABLE(of, bme280_aos_of_match);

static struct i2c_driver bme280_aos_driver = {
    .driver = {
        .name = BME280_DRV_NAME,
        .of_match_table = bme280_aos_of_match,
    },

    .probe = bme280_probe,
    .remove = bme280_remove,
};

module_i2c_driver(bme280_aos_driver);

