#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>

#define BME280_DRV_NAME "bme280-aos"

static int bme280_probe(struct i2c_client *client) {
    dev_info(&client->dev,
             "probe() called: address=0x%02x, adapter=%d\n",
             client->addr,
             i2c_adapter_id(client->adapter));
    return 0;
}

static void bme280_remove(struct i2c_client *client) {
    dev_info(&client->dev, "remove() called \n");
}

static const struct of_device_id bme280_aos_of_match[] = {
    {.compatible = "aos,bme280-test" },
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
