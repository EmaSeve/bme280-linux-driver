#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>

#define BME280_DRV_NAME "bme280-aos"

#define REG_CHIP_ID 0xD0
#define CHIP_ID 0x60

static int bme280_probe(struct i2c_client *client) {
    dev_info(&client->dev,
             "probe() called: address=0x%02x, adapter=%d\n",
             client->addr,
             i2c_adapter_id(client->adapter));
    
    s32 chip_id;

    /* Check that the adapter supports the read byte function */ 
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
        return -EOPNOTSUPP;

    chip_id = i2c_smbus_read_byte_data(client, REG_CHIP_ID);
    
    /* Check if there is an error */
    if (chip_id < 0) {
        dev_err(&client->dev, "Failed to read chip ID\n");
        return chip_id;
    }

    /* Check that the discovered device is the one inteded by the driver */
    if (chip_id != CHIP_ID) {
        dev_err(&client->dev, 
                "Unexpected chip ID: 0x%02x\n",
                chip_id);

        return -ENODEV;
    }

    dev_info(&client->dev,
            "BME280 detected, chip ID: 0x%02x\n",
            chip_id);
    
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
