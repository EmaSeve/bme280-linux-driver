#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>

#define IIO_SYSFS "/sys/bus/iio/devices/iio:device0"
// WARNING: the name should be checked, not hardcoded
#define IIO_DEVICE "/dev/iio:device0"

#define BUFFER_LENGTH 16
#define DEFAULT_PRINT_PERIOD_MS 1000

#define TEMP_OVERSAMPLING       "2"
#define PRESSURE_OVERSAMPLING   "4"
#define HUMIDITY_OVERSAMPLING   "1"

#define FILTER_COEFFICIENT      "2"
#define STANDBY_TIME_US         "250000"

static volatile sig_atomic_t stop = 0;


/*
 * Buffered scan layout for the current driver:
 *
 *   scan_index 0 -> temperature : s32, milli-C
 *   scan_index 1 -> pressure    : u32, Pa
 *   scan_index 2 -> humidity    : u32, milli-%RH
 *   scan_index 3 -> timestamp   : s64, ns
 *
 * After the three 32-bit channels, IIO aligns the 64-bit timestamp
 * to an 8-byte boundary, therefore 4 bytes of padding are present.
 */
struct bme280_scan {
    int32_t temperature;
    uint32_t pressure;
    uint32_t humidity;
    uint32_t padding;
    int64_t timestamp;
} __attribute__((packed));


static void handle_sigint(int sig) {
    (void)sig;
    stop = 1;
}


static int write_sysfs(const char *path, const char *value) {
    int fd = open(path, O_WRONLY);

    if (fd < 0) {
        perror(path);
        return -1;
    }

    if (write(fd, value, strlen(value)) < 0) {
        perror(path);
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}


static int read_long_sysfs(const char *path, long *value) {
    FILE *f = fopen(path, "r");

    if (!f) {
        perror(path);
        return -1;
    }

    if (fscanf(f, "%ld", value) != 1) {
        fclose(f);
        return -1;
    }

    fclose(f);
    return 0;
}


static int read_double_sysfs(const char *path, double *value) {
    FILE *f = fopen(path, "r");

    if (!f) {
        perror(path);
        return -1;
    }

    if (fscanf(f, "%lf", value) != 1) {
        fclose(f);
        return -1;
    }

    fclose(f);
    return 0;
}


/* ============================== FORCED MODE ============================== */

static int run_forced_demo(void) {
    long temperature;
    long humidity;
    double pressure;

    /* Make sure the buffered mode is disabled.
     * The driver will therefore stay in direct/forced mode. */
    if (write_sysfs(IIO_SYSFS "/buffer0/enable", "0") < 0)
        return -1;

    if (read_long_sysfs(IIO_SYSFS "/in_temp_input", &temperature) < 0)
        return -1;

    if (read_double_sysfs(IIO_SYSFS "/in_pressure_input", &pressure) < 0)
        return -1;

    if (read_long_sysfs(IIO_SYSFS "/in_humidityrelative_input", &humidity) < 0)
        return -1;

    printf("Forced mode - direct IIO reads\n");
    printf("--------------------------------\n");
    printf("Temperature : %.2f C\n", temperature / 1000.0);
    printf("Pressure    : %.2f Pa\n", pressure * 1000.0);
    printf("Humidity    : %.2f %%RH\n", humidity / 1000.0);

    return 0;
}


/* ============================= BUFFER MODE ============================== */

static int configure_normal_mode(void) {
    if (write_sysfs(IIO_SYSFS "/in_temp_oversampling_ratio",
                    TEMP_OVERSAMPLING) < 0)
        return -1;

    if (write_sysfs(IIO_SYSFS "/in_pressure_oversampling_ratio",
                    PRESSURE_OVERSAMPLING) < 0)
        return -1;

    if (write_sysfs(IIO_SYSFS "/in_humidityrelative_oversampling_ratio",
                    HUMIDITY_OVERSAMPLING) < 0)
        return -1;

    if (write_sysfs(IIO_SYSFS "/filter_coefficient",
                    FILTER_COEFFICIENT) < 0)
        return -1;

    if (write_sysfs(IIO_SYSFS "/standby_time_us",
                    STANDBY_TIME_US) < 0)
        return -1;

    return 0;
}

static int configure_buffer(void) {
    char length[16];

    /* Buffer must be disabled while configuring scan elements. */
    if (write_sysfs(IIO_SYSFS "/buffer0/enable", "0") < 0)
        return -1;
    
    if (configure_normal_mode() < 0) 
        return -1;

    /* Enable the four scan elements defined by the current driver. */
    if (write_sysfs(IIO_SYSFS "/scan_elements/in_temp_en", "1") < 0)
        return -1;

    if (write_sysfs(IIO_SYSFS "/scan_elements/in_pressure_en", "1") < 0)
        return -1;

    if (write_sysfs(IIO_SYSFS "/scan_elements/in_humidityrelative_en", "1") < 0)
        return -1;

    if (write_sysfs(IIO_SYSFS "/scan_elements/in_timestamp_en", "1") < 0)
        return -1;

    snprintf(length, sizeof(length), "%d", BUFFER_LENGTH);

    if (write_sysfs(IIO_SYSFS "/buffer0/length", length) < 0)
        return -1;

    if (write_sysfs(IIO_SYSFS "/buffer0/enable", "1") < 0)
        return -1;

    return 0;
}


static int run_buffer_demo(unsigned int print_period_ms) {
    struct bme280_scan sample;
    int fd;
    ssize_t n;
    int64_t first_timestamp = 0;
    int64_t last_print_timestamp = 0;
    int64_t print_period_ns =
        (int64_t)print_period_ms * 1000000LL;

    if (configure_buffer() < 0)
        return -1;

    fd = open(IIO_DEVICE, O_RDONLY);

    if (fd < 0) {
        perror(IIO_DEVICE);
        write_sysfs(IIO_SYSFS "/buffer0/enable", "0");
        return -1;
    }

    signal(SIGINT, handle_sigint);

    printf("Buffered normal mode\n");
    printf("--------------------------------\n");
    printf("Temperature oversampling : x%s\n", TEMP_OVERSAMPLING);
    printf("Pressure oversampling    : x%s\n", PRESSURE_OVERSAMPLING);
    printf("Humidity oversampling    : x%s\n", HUMIDITY_OVERSAMPLING);
    printf("IIR filter coefficient   : %s\n", FILTER_COEFFICIENT);
    printf("Standby time             : %s us\n", STANDBY_TIME_US);
    printf("Buffer length            : %d samples\n", BUFFER_LENGTH);
    printf("Printing one sample every %u ms\n\n", print_period_ms);

    printf("%-12s %-16s %-16s %-16s\n",
           "time [s]",
           "temperature [C]",
           "pressure [Pa]",
           "humidity [%RH]");

    printf("---------------------------------------------------------------\n");

    while (!stop) {

        /* Keep consuming every sample produced by the driver.
         * Only limit how frequently samples are printed. */
        n = read(fd, &sample, sizeof(sample));

        if (n < 0) {
            if (errno == EINTR)
                continue;

            perror("read");
            break;
        }

        if (n != sizeof(sample)) {
            fprintf(stderr,
                    "Unexpected scan size: %zd bytes (expected %zu)\n",
                    n, sizeof(sample));
            break;
        }

        if (first_timestamp == 0) {
            first_timestamp = sample.timestamp;
            last_print_timestamp =
                sample.timestamp - print_period_ns;
        }

        if (sample.timestamp - last_print_timestamp >= print_period_ns) {

            double elapsed =
                (sample.timestamp - first_timestamp) / 1e9;

            printf("%-12.3f %-16.3f %-16u %-16.3f\n",
                   elapsed,
                   sample.temperature / 1000.0,
                   sample.pressure,
                   sample.humidity / 1000.0);

            last_print_timestamp = sample.timestamp;
        }
    }

    close(fd);

    write_sysfs(IIO_SYSFS "/buffer0/enable", "0");

    printf("\nBuffer disabled\n");

    return 0;
}


int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage:\n");
        printf("  %s forced\n", argv[0]);
        printf("  %s buffer [print_period_ms]\n", argv[0]);
        return EXIT_FAILURE;
    }

    if (strcmp(argv[1], "forced") == 0)
        return run_forced_demo() == 0
            ? EXIT_SUCCESS
            : EXIT_FAILURE;

    if (strcmp(argv[1], "buffer") == 0) {
        unsigned int period = DEFAULT_PRINT_PERIOD_MS;

        if (argc >= 3)
            period = atoi(argv[2]);

        return run_buffer_demo(period) == 0
            ? EXIT_SUCCESS
            : EXIT_FAILURE;
    }

    printf("Unknown mode: %s\n", argv[1]);
    return EXIT_FAILURE;
}
