#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/slab.h>
#include "bme280_aos.h"

#define BME280_DRV_NAME "bme280-aos"

/* Decode a little-endian 16-bit value from two consecutive bytes. */
static u16 read_u16_le(const u8 *buf) {
    return (u16)buf[0] | ((u16)buf[1] << 8);
}

/* Sign-extend a 12-bit two's complement value to s16. */
static s16 sign_extend_12(u16 value)
{
    // Check if value < 0: 0x0800 is the 12th bit (sign)
	if (value & 0x0800)
		value |= 0xF000;

	return (s16)value;
}

static int bme280_read_calibration(struct i2c_client *client, 
                                    struct bme280_calib *calib) {
    u8 calib1[BME280_CALIB1_LEN];
    u8 calib2[BME280_CALIB2_LEN];
    u16 raw_h4;
    u16 raw_h5;
    int ret;

    ret = i2c_smbus_read_i2c_block_data(client, BME280_REG_CALIB1, BME280_CALIB1_LEN,
                                        calib1);

    if (ret < 0) return ret;
    if (ret != BME280_CALIB1_LEN) return -EIO;

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

    ret = i2c_smbus_read_i2c_block_data(client, BME280_REG_CALIB2, BME280_CALIB2_LEN,
                                        calib2);
    if (ret < 0) return ret;
    if (ret != BME280_CALIB2_LEN) return -EIO;

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

    chip_id = i2c_smbus_read_byte_data(client, BME280_REG_CHIP_ID);
    
    /* Check read error */
    if (chip_id < 0) {
        dev_err(&client->dev, "Failed to read chip ID\n");
        return chip_id;
    }

    /* Check if the discovered device is the one inteded by the driver */
    if (chip_id != BME280_CHIP_ID) {
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
    int ret;

    /* Check functions support */
    if (!i2c_check_functionality(client->adapter, 
            I2C_FUNC_SMBUS_READ_BYTE_DATA | I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
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

    // devm_ binds the memory allocated to &client->dev: memery is the deallocated
    // when the driver is disconnected.
    data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
    // Check mem allocation
    if(!data) return -ENOMEM;
    
    data->client = client;
    // Set specific data related to the device, that will be used later
    i2c_set_clientdata(client, data);
    
    ret = bme280_read_calibration(client, &data->calib);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read calibration data\n");
        return ret;
    }

    dev_info(&client->dev,
        	 "T calib: %u %d %d\n",
	         data->calib.t1,
	         data->calib.t2,
	         data->calib.t3);

    dev_info(&client->dev,
	         "P calib: %u %d %d %d %d %d %d %d %d\n",
	         data->calib.p1,
	         data->calib.p2,
	         data->calib.p3,
	         data->calib.p4,
	         data->calib.p5,
	         data->calib.p6,
	         data->calib.p7,
	         data->calib.p8,
	         data->calib.p9);

    dev_info(&client->dev,
	         "H calib: %u %d %u %d %d %d\n",
	         data->calib.h1,
	         data->calib.h2,
	         data->calib.h3,
	         data->calib.h4,
	         data->calib.h5,
	         data->calib.h6);
    
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
