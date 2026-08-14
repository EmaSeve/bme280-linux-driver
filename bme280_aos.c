#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/bitops.h>
#include "bme280_aos.h"

#define BME280_DRV_NAME "bme280-aos"

#define T_OVRSMPL_FACTOR  1
#define P_OVRSMPL_FACTOR  1
#define H_OVRSMPL_FACTOR  1

#define POLL_MIN_US 500
#define POLL_MAX_US 1000

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

static s32 bme280_compensate_temperature(struct bme280_data *data,
                                         s32 adc_temp,
                                         s32 *t_fine) {
    const struct bme280_calib *calib = &data->calib;
    s32 var1;
    s32 var2;
    s32 delta;

    var1 = (((adc_temp >> 3) - ((s32)calib->t1 << 1)) *
            (s32)calib->t2) >> 11;

    delta = (adc_temp >> 4) - (s32)calib->t1;

    var2 = (((delta * delta) >> 12) *
            (s32)calib->t3) >> 14;

    *t_fine = var1 + var2;

    return (*t_fine * 5 + 128) >> 8;
}

static u32 bme280_compensate_pressure(struct bme280_data *data,
                                      s32 adc_press,
                                      s32 t_fine) {
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
    sensitivity = ((((s64)1 << 47) + sensitivity) *
                   (s64)calib->p1) >> 33;

    if (sensitivity == 0)
        return 0;

    /* Pressure calculation */
    p = (s64)1048576 - adc_press;
    p = (((p << 31) - offset) * 3125) / sensitivity;

    /* Nonlinear corrections */
    corr_quad = ((s64)calib->p9 *
                 (p >> 13) * (p >> 13)) >> 25;

    corr_lin = ((s64)calib->p8 * p) >> 19;

    p = ((p + corr_quad + corr_lin) >> 8) +
        ((s64)calib->p7 << 4);

    return (u32)p;
}

static u32 bme280_compensate_humidity(struct bme280_data *data,
                                      s32 adc_hum,
                                      s32 t_fine) {
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
    adc_corr = (adc_hum << 14) -
               ((s32)calib->h4 << 20) -
               ((s32)calib->h5 * x) +
               16384;

    term1 = adc_corr >> 15;

    /* Temperature-dependent humidity gain */
    term2a = (x * (s32)calib->h6) >> 10;
    term2b = ((x * (s32)calib->h3) >> 11) + 32768;

    gain = ((term2a * term2b) >> 10) + 2097152;
    gain = (gain * (s32)calib->h2 + 8192) >> 14;

    h = term1 * gain;

    /* Nonlinear correction */
    nonlinear = (((((h >> 15) * (h >> 15)) >> 7) *
                  (s32)calib->h1) >> 4);

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

static int bme280_wait_measurement(struct bme280_data *data) {
    unsigned long timeout;
    unsigned long typical_meas_time_us;
    unsigned long max_meas_time_us;
    unsigned long residual_timeout_ms = 3;
    int status;

    /*
     * Measurement time depends on the oversampling settings.
     * From BME280 datasheet:
     *
     * t_meas_typ [ms] = 1 + 2 * T_oversampling + (2 * P_oversampling + 0.5)
     *                  + (2 * H_oversampling + 0.5)
     *
     * t_meas_max [ms] = 1.25 + 2.3 * T_oversampling + (2.3 * P_oversampling + 0.575)
     *                  + (2.3 * H_oversampling + 0.575)
     */
    typical_meas_time_us = 1000 + 2000 * T_OVRSMPL_FACTOR + (2000 * P_OVRSMPL_FACTOR + 500) +
                         (2000 * H_OVRSMPL_FACTOR + 500);

    max_meas_time_us = 1250 + 2300 * T_OVRSMPL_FACTOR + (2300 * P_OVRSMPL_FACTOR + 575) +
                         (2300 * H_OVRSMPL_FACTOR + 575);

    timeout = jiffies + msecs_to_jiffies(DIV_ROUND_UP(max_meas_time_us, 1000) + 
                                        residual_timeout_ms);

    /*
     * Sleep for the typical conversion time first, then poll
     * the status register until the measurement completes.
     */
    fsleep(typical_meas_time_us);

    u8 status_busy = (1<<3) | 1; // 00001001
    do {
        status = i2c_smbus_read_byte_data(data->client, REG_STATUS);
        if (status < 0)
            return status;

        // Sensor correctly executed the measure
        if (!(status & status_busy))
            return 0;

        usleep_range(POLL_MIN_US, POLL_MAX_US);

    } while (time_before(jiffies, timeout));

    return -ETIMEDOUT;
}

/* Switch to forced mode */
static int bme280_trigger_measurement(struct bme280_data *data) {
    u8 ctrl_meas = (T_OVRSMPL_BITS<<5) | (P_OVRSMPL_BITS<<2) | MODE_FORCED;

    return i2c_smbus_write_byte_data(data->client, REG_CTRL_MEAS, ctrl_meas); 
}

static int bme280_acquire_raw_measurement(struct bme280_data *data, 
                                   struct bme280_raw_sample *raw) {
    int ret;

    mutex_lock(&data->lock);

    ret = bme280_trigger_measurement(data);
    if (ret < 0) goto out;

    ret = bme280_wait_measurement(data);
    if (ret < 0) goto out;

    ret = bme280_read_measurement(data, raw);

out:
    mutex_unlock(&data->lock);

    return ret;
}

static int bme280_configure(struct bme280_data *data) {
    struct i2c_client *client = data->client;
    int ret;
    
    /* Disable IIR filter and 3-wire SPI.
    * Standby time is irrelevant in forced mode. */
    ret = i2c_smbus_write_byte_data(client, REG_CONFIG, 0x00); 
    if (ret < 0) return ret;

    /* Humidity oversampling (0, 1, 2, 4, 8, 16)
     * Warning: changes to this register become effective only after
     * a write operation to ctrl_meas. */
    ret = i2c_smbus_write_byte_data(client, REG_CTRL_HUM, H_OVRSMPL_BITS);
    if (ret < 0) return ret;

    /* CTRL_MEAS register layout:
    *
    *  bit:   7   6   5 | 4   3   2 | 1   0
    *        -----------+-----------+-------
    *       temp_ovrsmp |pres_ovrsmp | mode
    */
    u8 ctrl_meas = (T_OVRSMPL_BITS<<5) | (P_OVRSMPL_BITS<<2) | MODE_SLEEP; 
    ret = i2c_smbus_write_byte_data(client, REG_CTRL_MEAS, ctrl_meas);  
    if (ret < 0) return ret;

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
        dev_err(&client->dev, 
                "Unexpected chip ID: 0x%02x\n",
                chip_id);

        return -ENODEV;
    }
    
    return chip_id;
}

static const struct iio_chan_spec bme280_iio_channels[] = {
    {
        .type = IIO_TEMP,
        .info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
    },
    {
        .type = IIO_PRESSURE,
        .info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
    },
    {
        .type = IIO_HUMIDITYRELATIVE,
        .info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
    },
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

    if (mask != IIO_CHAN_INFO_PROCESSED)
        return -EINVAL;

    ret = bme280_acquire_raw_measurement(data, &raw);
    if (ret < 0)
        return ret;

    bme280_compensate(data, &raw, &sample);

    switch (chan->type) {
    case IIO_TEMP:
        /*
         * sample.temperature is in centi-C.
         * IIO temperature input uses milli-C.
         */
        *val = sample.temperature * 10;
        return IIO_VAL_INT;

    case IIO_PRESSURE:
        /*
         * sample.pressure is Q24.8 Pa.
         *
         * pressure [kPa] =
         * sample.pressure / (256 * 1000)
         */
        *val = sample.pressure;
        *val2 = 256000;
        return IIO_VAL_FRACTIONAL;

    case IIO_HUMIDITYRELATIVE:
        /*
         * sample.humidity is Q22.10 %RH.
         * IIO humidity input uses milli-%RH.
         */
        *val = (sample.humidity * 1000) >> 10;
        return IIO_VAL_INT;

    default:
        return -EINVAL;
    }
}

static const struct iio_info bme280_iio_info = {
    .read_raw = bme280_iio_read_raw,
};

static int bme280_probe(struct i2c_client *client) {
    struct iio_dev *indio_dev;
    struct bme280_data *data;
    s32 chip_id;
    int ret;

    /* */
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
    if (chip_id < 0)
        return chip_id;

    dev_info(&client->dev,
             "BME280 detected, chip ID: 0x%02x\n",
             chip_id);

    /* Allocate the IIO device and the private driver data */
    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
    if (!indio_dev)
        return -ENOMEM;

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

    /* Configure measurement mode */
    ret = bme280_configure(data);
    if (ret < 0)
        return ret;
    
    /* confiure and register iio_dev */
    indio_dev->name = BME280_DRV_NAME;
    indio_dev->info = &bme280_iio_info;
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->channels = bme280_iio_channels;
    indio_dev->num_channels = ARRAY_SIZE(bme280_iio_channels);

    ret = devm_iio_device_register(&client->dev, indio_dev);
    if (ret < 0)
        return dev_err_probe(&client->dev, ret,
                             "Failed to register IIO device\n");

    return 0;
}

static void bme280_remove(struct i2c_client *client) {
    dev_info(&client->dev, "remove() called \n");
}

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

MODULE_AUTHOR("EmaSeve");
MODULE_DESCRIPTION("BME280 I2C DRIVER AOS");
MODULE_LICENSE("GPL");
