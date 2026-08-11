
#define BME280_REG_CHIP_ID 0xD0
#define BME280_CHIP_ID 0x60

#define BME280_REG_CALIB1	0x88
#define BME280_CALIB1_LEN	26

#define BME280_REG_CALIB2	0xE1
#define BME280_CALIB2_LEN	7

/*
 * BME280 calibration register layout
 *
 * Calibration data is stored in two separate register blocks.
 * Multi-byte coefficients are little-endian: the least significant
 * byte is stored at the lower register address.
 *
 * Block 1: 0x88 - 0xA1
 *
 *   Address     Content
 *   -------     --------------------------
 *   0x88        T1[7:0]       \
 *   0x89        T1[15:8]       > u16 T1
 *   0x8A        T2[7:0]       \
 *   0x8B        T2[15:8]       > s16 T2
 *   0x8C        T3[7:0]       \
 *   0x8D        T3[15:8]       > s16 T3
 *
 *   0x8E        P1[7:0]       \
 *   0x8F        P1[15:8]       > u16 P1
 *   0x90        P2[7:0]       \
 *   0x91        P2[15:8]       > s16 P2
 *     ...           ...
 *   0x9E        P9[7:0]       \
 *   0x9F        P9[15:8]       > s16 P9
 *
 *   0xA0        reserved
 *   0xA1        H1[7:0]          u8 H1
 *
 *
 * Block 2: 0xE1 - 0xE7
 *
 *   Address        7                         0
 *                  +-------------------------+
 *   0xE1           |        H2[7:0]          |
 *   0xE2           |       H2[15:8]          |
 *                  +-------------------------+
 *   0xE3           |        H3[7:0]          |
 *                  +-------------------------+
 *   0xE4           |        H4[11:4]         |
 *                  +------------+------------+
 *   0xE5           |  H5[3:0]   |  H4[3:0]   |
 *                  +------------+------------+
 *   0xE6           |        H5[11:4]         |
 *                  +-------------------------+
 *   0xE7           |        H6[7:0]          |
 *                  +-------------------------+
 *
 * H4 and H5 are signed 12-bit values and require explicit
 * reconstruction and sign extension.
 */
struct bme280_calib {
	u16 t1;
	s16 t2;
	s16 t3;

	u16 p1;
	s16 p2;
	s16 p3;
	s16 p4;
	s16 p5;
	s16 p6;
	s16 p7;
	s16 p8;
	s16 p9;

	u8  h1;
	s16 h2;
	u8  h3;
	s16 h4;
	s16 h5;
	s8  h6;
};

struct bme280_data {
	struct i2c_client *client;
	struct bme280_calib calib;
};


