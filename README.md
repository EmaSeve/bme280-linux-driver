# BME280 Linux IIO Driver

Linux I²C driver for the Bosch BME280 environmental sensor, integrated with the Linux Industrial I/O (IIO) subsystem.

The BME280 measures temperature, atmospheric pressure and relative humidity. The driver exposes two acquisition paths to userspace:

- **Forced mode** exposes on-demand measurements and is intended for weather monitoring, humidity sensing, and other applications requiring a low sampling rate and low power consumption.

- **Normal mode** exposes buffered continuous measurements and is intended for applications requiring periodic acquisition. The sensor performs measurements automatically at regular intervals, and the resulting samples are delivered to userspace through the IIO buffer.

The acquisition behavior can be configured from userspace through the IIO sysfs interface.

Example of buffered normal-mode acquisition:

```text
Buffered normal mode
--------------------------------
Temperature oversampling : x2
Pressure oversampling    : x4
Humidity oversampling    : x1
IIR filter coefficient   : 2
Standby time             : 250000 us
Buffer length            : 16 samples
Printing one sample every 1000 ms

time [s]     temperature [C]  pressure [Pa]    humidity [%RH]
---------------------------------------------------------------
0.000        26.610           100231           46.423
1.097        26.630           100232           46.959
2.192        28.420           100166           68.845
3.285        28.920           100197           72.946
4.380        29.410           100197           76.673
```

## Sensor configuration

Temperature, pressure and humidity oversampling can be configured independently. The IIR filter coefficient and standby time are global parameters used by buffered normal mode.

Configuration must be performed while the IIO buffer is disabled.

For example:

```bash
IIO_DEV=/sys/bus/iio/devices/iio:deviceX

# Oversampling
# Configure each sensor channel independently
echo 2 | sudo tee $IIO_DEV/in_temp_oversampling_ratio
echo 4 | sudo tee $IIO_DEV/in_pressure_oversampling_ratio
echo 1 | sudo tee $IIO_DEV/in_humidityrelative_oversampling_ratio

# IIR filter coefficient
echo 8 | sudo tee $IIO_DEV/filter_coefficient

# Normal-mode standby time, in microseconds
echo 250000 | sudo tee $IIO_DEV/standby_time_us
```

The supported values for each parameter can be inspected directly from sysfs:

```bash
cat $IIO_DEV/oversampling_ratio_available
cat $IIO_DEV/filter_coefficient_available
cat $IIO_DEV/standby_time_us_available
```

Oversampling affects both acquisition modes, while the filter coefficient and standby time are applied when buffered normal mode is enabled.

## Using the driver

### Forced mode

Forced mode is available when the IIO buffer is disabled. Measurements are requested directly through the processed IIO channel attributes:

```bash
IIO_DEV=/sys/bus/iio/devices/iio:deviceX

echo 0 | sudo tee $IIO_DEV/buffer0/enable

cat $IIO_DEV/in_temp_input
cat $IIO_DEV/in_pressure_input
cat $IIO_DEV/in_humidityrelative_input
```

### Buffered normal mode

For continuous acquisition, the desired scan elements are selected and the IIO buffer is enabled. Temperature, pressure and humidity are then delivered as part of the same timestamped scan.

```bash
IIO_DEV=/sys/bus/iio/devices/iio:deviceX

# Make sure the buffer is disabled before configuring it
echo 0 | sudo tee $IIO_DEV/buffer0/enable

echo 8 | sudo tee $IIO_DEV/filter_coefficient
echo 250000 | sudo tee $IIO_DEV/standby_time_us

# Select the scan elements
echo 1 | sudo tee $IIO_DEV/scan_elements/in_temp_en
echo 1 | sudo tee $IIO_DEV/scan_elements/in_pressure_en
echo 1 | sudo tee $IIO_DEV/scan_elements/in_humidityrelative_en
echo 1 | sudo tee $IIO_DEV/scan_elements/in_timestamp_en

# Configure the software buffer
echo 16 | sudo tee $IIO_DEV/buffer0/length

# Start buffered normal-mode acquisition
echo 1 | sudo tee $IIO_DEV/buffer0/enable
```

The resulting binary scan stream can be read from:

```text
/dev/iio:deviceX
```

When acquisition is no longer required, disable the buffer:

```bash
echo 0 | sudo tee $IIO_DEV/buffer0/enable
```

Disabling the buffer returns the driver to direct forced-mode operation.

## Demo

A minimal userspace demo is provided in **userspace/demo.c**, run it with:

```bash
sudo ./demo forced
sudo ./demo buffer
```
