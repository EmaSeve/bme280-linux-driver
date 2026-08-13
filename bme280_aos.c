#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/delay.h>
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

static int bme280_read_raw_measurement(struct bme280_data *data, struct raw_data *raw) {
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

static int bme280_take_measurement(struct bme280_data *data, struct raw_data *raw) {
    int ret;

    mutex_lock(&data->lock);

    ret = bme280_trigger_measurement(data);
    if (ret < 0) goto out;

    ret = bme280_wait_measurement(data);
    if (ret < 0) goto out;

    ret = bme280_read_raw_measurement(data, raw);

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

static int bme280_probe(struct i2c_client *client) {
    /* */
    dev_info(&client->dev,
             "probe() called: address=0x%02x, adapter=%d\n",
             client->addr,
             i2c_adapter_id(client->adapter));
   
    struct bme280_data *data;
    s32 chip_id;
    struct raw_data raw;
    int ret;

    /* Check functions support */
    if (!i2c_check_functionality(client->adapter, 
            I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
        	dev_err(&client->dev, "Required I2C/SMBus functionality not supported\n");
	        return -EOPNOTSUPP;
    }

    /* Check that device chip ID is correct */
    chip_id = bme280_read_chip_id(client);
    if(chip_id < 0) 
        return chip_id;
    dev_info(&client->dev,
            "BME280 detected, chip ID: 0x%02x\n",
            chip_id);

    /* Read sensor calibration data */

    // devm_ binds the memory allocated to &client->dev: memory is deallocated
    // when driver disconnects.
    data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
    if(!data) return -ENOMEM;
    
    data->client = client;
    mutex_init(&data->lock);
    // Set specific data related to the device, that will be used later
    i2c_set_clientdata(client, data);
    
    ret = bme280_read_calibration(data);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read calibration data\n");
        return ret;
    }
    
    /* Configure measurement mode */
    ret = bme280_configure(data);
    if (ret < 0) return ret;

    /* Take measurement */
    ret = bme280_take_measurement(data, &raw);
    if (ret < 0) return ret;

    dev_info(&client->dev,
         "raw: temp=%u pressure=%u humidity=%u\n",
         raw.temperature,
         raw.pressure,
         raw.humidity);

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
