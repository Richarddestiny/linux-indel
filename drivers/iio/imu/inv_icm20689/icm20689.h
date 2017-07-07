/*
* Copyright (C) 2012 Invensense, Inc.
* Copyright (C) 2017 Indel AG
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/
#include <linux/i2c.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/platform_data/invensense_mpu6050.h>

/**
 *  struct icm20689_reg_map - Notable registers.
 *  @sample_rate_div:	Divider applied to gyro output rate.
 *  @user_ctrl:		Enables/resets the FIFO.
 *  @fifo_en:		Determines which data will appear in FIFO.
 *  @gyro_config:	gyro config register.
 *  @accl_config:	accel config register
 *  @fifo_count_h:	Upper byte of FIFO count.
 *  @fifo_r_w:		FIFO register.
 *  @raw_gyro:		Address of first gyro register.
 *  @raw_accl:		Address of first accel register.
 *  @temperature:	temperature register
 *  @int_enable:	Interrupt enable register.
 *  @pwr_mgmt_1:	Controls chip's power state and clock source.
 *  @pwr_mgmt_2:	Controls power state of individual sensors.
 */
struct icm20689_reg_map {
	u8 sample_rate_div;
	u8 config;
	u8 gyro_config;
	u8 accl_config_1;
	u8 accl_config_2;
	u8 lp_mode_config;
	u8 accl_wom_thr;
	u8 accl_wom_thr_x;
	u8 accl_wom_thr_y;
	u8 accl_wom_thr_z;
	u8 fifo_en;
	u8 fsync_int;
	u8 int_pin_cfg;
	u8 int_enable;
	u8 dmp_int_status;
	u8 int_status;
	u8 raw_gyro;
	u8 raw_accl;
	u8 temperature;
	u8 accl_intel;
	u8 user_ctrl;
	u8 pwr_mgmt_1;
	u8 pwr_mgmt_2;
	u8 fifo_count_h;
	u8 fifo_count_l;
	u8 fifo_r_w;

};

/*device enum */
enum inv_devices {
	ICM20689,
	INV_NUM_PARTS
};

/**
 *  struct icm20689_chip_config - Cached chip configuration data.
 *  @fsr:		Full scale range.
 *  @lpf:		Digital low pass filter frequency.
 *  @accl_fs:		accel full scale range.
 *  @enable:		master enable state.
 *  @accl_fifo_enable:	enable accel data output
 *  @gyro_fifo_enable:	enable gyro data output
 *  @fifo_rate:		FIFO update rate.
 */
struct icm20689_chip_config {
	unsigned int fsr:2;
	unsigned int lpf:3;
	unsigned int accl_fs:2;
	unsigned int enable:1;
	unsigned int accl_fifo_enable:1;
	unsigned int gyro_fifo_enable:1;
	u16 fifo_rate;
};

/**
 *  struct icm20689_hw - Other important hardware information.
 *  @num_reg:	Number of registers on device.
 *  @name:      name of the chip.
 *  @reg:   register map of the chip.
 *  @config:    configuration of the chip.
 */
struct icm20689_hw {
	u8 num_reg;
	u8 *name;
	const struct icm20689_reg_map *reg;
	const struct icm20689_chip_config *config;
};

/*
 *  struct icm20689_state - Driver state variables.
 *  @TIMESTAMP_FIFO_SIZE: fifo size for timestamp.
 *  @trig:          	IIO trigger.
 *  @chip_config:		Cached attribute information.
 *  @reg:				Map of important registers.
 *  @hw:				Other hardware-specific information.
 *  @chip_type:			chip type.
 *  @time_stamp_lock:	spin lock to time stamp.
 *  @client:			i2c client handle.
 *  @plat_data:			platform data.
 *  @timestamps:        kfifo queue to store time stamp.
 */
struct icm20689_state {
#define TIMESTAMP_FIFO_SIZE 16
	struct iio_trigger  *trig;
	struct icm20689_chip_config chip_config;
	const struct icm20689_reg_map *reg;
	const struct icm20689_hw *hw;
	enum   inv_devices chip_type;
	spinlock_t time_stamp_lock;
	struct mutex lock;
	struct i2c_client *client;
	struct device_node *dev_node;
	unsigned int powerup_count;
	u8 wom_thr;
	DECLARE_KFIFO(timestamps, long long, TIMESTAMP_FIFO_SIZE);
};

/*register and associated bit definition*/

#define ICM20689_REG_SAMPLE_RATE_DIV     0x19


/*CONFIGURATION*/
#define ICM20689_REG_CONFIG              0x1A

enum ICM20689_FILTER {
	ICM20689_FILTER_256HZ_NOLPF2 = 0,
	ICM20689_FILTER_188HZ,
	ICM20689_FILTER_98HZ,
	ICM20689_FILTER_42HZ,
	ICM20689_FILTER_20HZ,
	ICM20689_FILTER_10HZ,
	ICM20689_FILTER_5HZ,
	ICM20689_FILTER_2100HZ_NOLPF,
	NUM_MPU6050_FILTER
};
/*GYROSCOPE CONFIGURATION*/
#define ICM20689_REG_GYRO_CONFIG         0x1B

enum ICM20689_FS_SEL {
	ICM20689_FSR_250DPS 	= 0,
	ICM20689_FSR_500DPS 	= 1,
	ICM20689_FSR_1000DPS	= 2,
	ICM20689_FSR_2000DPS	= 3,
	NUM_MPU6050_FSR
};


#define ICM20689_REG_ACCEL_CONFIG_1      0x1C

enum ICM20689_ACCL_FS {
	ICM20689_FS_02G = 0,
	ICM20689_FS_04G,
	ICM20689_FS_08G,
	ICM20689_FS_16G,
	NUM_ACCL_FSR
};
#define ICM20689_REG_ACCEL_CONFIG_2		 0x1D

/*LOW POWER MODE CONFIGURATION*/
#define ICM20689_REG_LP_MODE_CFG		 0x1E




/*WAKE-ON MOTION THRESHOLD (ACCELEROMETER)*/
#define ICM20689_REG_ACCEL_WOM			 0x1F
#define ICM20689_REG_ACCEL_WOM_X		 0x20
#define ICM20689_REG_ACCEL_WOM_Y		 0x21
#define ICM20689_REG_ACCEL_WOM_Z		 0x22

#define ICM20689_REG_FIFO_EN             0x23

#define ICM20689_BIT_ACCEL_OUT           0x08
#define ICM20689_BITS_GYRO_OUT           0x70




#define ICM20689_REG_INT_PIN_CFG		 0x37
#define ICM20689_BITS_INT_PIN_CFG		 0xD0

#define ICM20689_REG_INT_ENABLE          0x38
#define ICM20689_BITS_INT_ENABLE		 0x01
#define ICM20689_BITS_WOM_ENABLE		 0xE0

#define ICM20689_REG_INT_STATUS			 0x3A
#define ICM20689_BIT_DATA_RDY_EN         0x01
#define ICM20689_BIT_DMP_INT_EN          0x02

#define ICM20689_REG_RAW_ACCEL           0x3B
#define ICM20689_REG_TEMPERATURE         0x41
#define ICM20689_REG_RAW_GYRO            0x43

#define ICM20689_REG_ACCL_INTEL_CTRL	 0x69
#define ICM20689_BIT_ACCL_INTEL_ENABLE   0xC0

/* USER CONTROL */
#define ICM20689_REG_USER_CTRL           0x6A
#define ICM20689_BIT_FIFO_RST            0x04
#define ICM20689_BIT_DMP_RST             0x08
#define ICM20689_BIT_I2C_MST_EN          0x20
#define ICM20689_BIT_FIFO_EN             0x40
#define ICM20689_BIT_DMP_EN              0x80

/* POWER MANAGEMENT 1 */
#define ICM20689_REG_PWR_MGMT_1          0x6B
#define ICM20689_BIT_H_RESET             0x80
#define ICM20689_BIT_SLEEP               0x40
#define ICM20689_BIT_CYCLE				 0x20
#define ICM20689_BIT_CLK_MASK            0x7
enum ICM20689_CLK_SEL {
	INV_CLK_INTERNAL = 0,
	INV_CLK_PLL,
	NUM_CLK
};

/*POWER MANAGEMENT 2*/
#define ICM20689_REG_PWR_MGMT_2          0x6C
#define ICM20689_BIT_PWR_ACCL_STBY       0x38
#define ICM20689_BIT_PWR_GYRO_STBY       0x07

#define ICM20689_REG_FIFO_COUNT_H        0x72
#define ICM20689_REG_FIFO_R_W            0x74


#define ICM20689_BYTES_PER_3AXIS_SENSOR   6
#define ICM20689_FIFO_COUNT_BYTE          2
#define ICM20689_FIFO_THRESHOLD           500
#define ICM20689_POWER_UP_TIME            100
#define ICM20689_TEMP_UP_TIME             100
#define ICM20689_SENSOR_UP_TIME           30
#define ICM20689_REG_UP_TIME              5

#define ICM20689_TEMP_OFFSET	             12421
#define ICM20689_TEMP_SCALE               2941
#define ICM20689_MAX_GYRO_FS_PARAM        3
#define ICM20689_MAX_ACCL_FS_PARAM        3
#define ICM20689_THREE_AXIS               3
#define ICM20689_GYRO_CONFIG_FSR_SHIFT    3
#define ICM20689_ACCL_CONFIG_FSR_SHIFT    3

/* 6 + 6 round up and plus 8 */
#define ICM20689_OUTPUT_DATA_SIZE         24


#define ICM20689_BIT_BYPASS_EN	0xF0

/* init parameters */
#define ICM20689_INIT_FIFO_RATE           50
#define ICM20689_TIME_STAMP_TOR           5
#define ICM20689_MAX_FIFO_RATE            1000
#define ICM20689_MIN_FIFO_RATE            4
#define ICM20689_ONE_K_HZ                 1000

/* scan element definition */
enum ICM20689_scan {
	ICM20689_SCAN_ACCL_X,
	ICM20689_SCAN_ACCL_Y,
	ICM20689_SCAN_ACCL_Z,
	ICM20689_SCAN_GYRO_X,
	ICM20689_SCAN_GYRO_Y,
	ICM20689_SCAN_GYRO_Z,
	ICM20689_SCAN_TIMESTAMP,
};



/* IIO attribute address */
enum ICM20689_IIO_ATTR_ADDR {
	ATTR_GYRO_MATRIX,
	ATTR_ACCL_MATRIX,
};


irqreturn_t icm20689_irq_handler(int irq, void *p);
irqreturn_t icm20689_read_fifo(int irq, void *p);
int icm20689_probe_trigger(struct iio_dev *indio_dev);
void icm20689_remove_trigger(struct icm20689_state *st);
int inv_reset_fifo(struct iio_dev *indio_dev);
int icm20689_switch_engine(struct icm20689_state *st, bool en, u32 mask);
int icm20689_write_reg(struct icm20689_state *st, int reg, u8 val);
int icm20689_set_power_itg(struct icm20689_state *st, bool power_on);


