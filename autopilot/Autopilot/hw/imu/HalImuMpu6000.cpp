/*
 * HalImuMpu6000.cpp
 *
 *  Created on: 7 juil. 2013
 *      Author: Aberzen
 */

#include <infra/rtos/Task.hpp>
#include "HalImuMpu6000.hpp"
#include <system/params/Nrd.hpp>
#include <hw/spi/SpiBus.hpp>
#include <Arduino.h>

#include <string.h>

#define BITS_EXTRACT(regval, mask, offset) \
	((regval & mask) >> offset)

#define MPU6000_CS_PIN       53        // APM pin connected to mpu6000's chip select pin
#define DMP_FIFO_BUFFER_SIZE 72        // DMP FIFO buffer size

// MPU 6000 registers
#define MPUREG_XG_OFFS_TC                               0x00
#define MPUREG_YG_OFFS_TC                               0x01
#define MPUREG_ZG_OFFS_TC                               0x02
#define MPUREG_X_FINE_GAIN                              0x03
#define MPUREG_Y_FINE_GAIN                              0x04
#define MPUREG_Z_FINE_GAIN                              0x05
#define MPUREG_XA_OFFS_H                                0x06    // X axis accelerometer offset (high byte)
#define MPUREG_XA_OFFS_L                                0x07    // X axis accelerometer offset (low byte)
#define MPUREG_YA_OFFS_H                                0x08    // Y axis accelerometer offset (high byte)
#define MPUREG_YA_OFFS_L                                0x09    // Y axis accelerometer offset (low byte)
#define MPUREG_ZA_OFFS_H                                0x0A    // Z axis accelerometer offset (high byte)
#define MPUREG_ZA_OFFS_L                                0x0B    // Z axis accelerometer offset (low byte)
#define MPUREG_PRODUCT_ID                               0x0C    // Product ID Register
#define MPUREG_XG_OFFS_USRH                     0x13    // X axis gyro offset (high byte)
#define MPUREG_XG_OFFS_USRL                     0x14    // X axis gyro offset (low byte)
#define MPUREG_YG_OFFS_USRH                     0x15    // Y axis gyro offset (high byte)
#define MPUREG_YG_OFFS_USRL                     0x16    // Y axis gyro offset (low byte)
#define MPUREG_ZG_OFFS_USRH                     0x17    // Z axis gyro offset (high byte)
#define MPUREG_ZG_OFFS_USRL                     0x18    // Z axis gyro offset (low byte)
#define MPUREG_SMPLRT_DIV                               0x19    // sample rate.  Fsample= 1Khz/(<this value>+1) = 200Hz
#       define MPUREG_SMPLRT_1000HZ                             0x00
#       define MPUREG_SMPLRT_500HZ                              0x01
#       define MPUREG_SMPLRT_250HZ                              0x03
#       define MPUREG_SMPLRT_200HZ                              0x04
#       define MPUREG_SMPLRT_100HZ                              0x09
#       define MPUREG_SMPLRT_50HZ                               0x13
#define MPUREG_CONFIG                                           0x1A
#define MPUREG_GYRO_CONFIG                                      0x1B
// bit definitions for MPUREG_GYRO_CONFIG
#       define BITS_GYRO_FS_250DPS                              0x00
#       define BITS_GYRO_FS_500DPS                              0x08
#       define BITS_GYRO_FS_1000DPS                             0x10
#       define BITS_GYRO_FS_2000DPS                             0x18
#       define BITS_GYRO_FS_MASK                                0x18    // only bits 3 and 4 are used for gyro full scale so use this to mask off other bits
#       define BITS_GYRO_ZGYRO_SELFTEST                 0x20
#       define BITS_GYRO_YGYRO_SELFTEST                 0x40
#       define BITS_GYRO_XGYRO_SELFTEST                 0x80
#define MPUREG_ACCEL_CONFIG                             0x1C
#define MPUREG_MOT_THR                                  0x1F    // detection threshold for Motion interrupt generation.  Motion is detected when the absolute value of any of the accelerometer measurements exceeds this
#define MPUREG_MOT_DUR                                  0x20    // duration counter threshold for Motion interrupt generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit of 1 LSB = 1 ms
#define MPUREG_ZRMOT_THR                                0x21    // detection threshold for Zero Motion interrupt generation.
#define MPUREG_ZRMOT_DUR                                0x22    // duration counter threshold for Zero Motion interrupt generation. The duration counter ticks at 16 Hz, therefore ZRMOT_DUR has a unit of 1 LSB = 64 ms.
#define MPUREG_FIFO_EN                                  0x23
#       define BIT_SLV0_FIFO_EN							0x01
#       define BIT_SLV1_FIFO_EN							0x02
#       define BIT_SLV2_FIFO_EN							0x04
#       define BIT_ACCEL_FIFO_EN						0x08
#       define BIT_ZG_FIFO_EN							0x10
#       define BIT_YG_FIFO_EN							0x20
#       define BIT_XG_FIFO_EN							0x40
#       define BIT_TEMP_FIFO_EN							0x80

#define MPUREG_INT_PIN_CFG                              0x37
#       define BIT_INT_RD_CLEAR                                 0x10    // clear the interrupt when any read occurs
#define MPUREG_INT_ENABLE                               0x38
// bit definitions for MPUREG_INT_ENABLE
#       define BIT_RAW_RDY_EN                                   0x01
#       define BIT_DMP_INT_EN                                   0x02    // enabling this bit (DMP_INT_EN) also enables RAW_RDY_EN it seems
#       define BIT_UNKNOWN_INT_EN                               0x04
#       define BIT_I2C_MST_INT_EN                               0x08
#       define BIT_FIFO_OFLOW_EN                                0x10
#       define BIT_ZMOT_EN                                              0x20
#       define BIT_MOT_EN                                               0x40
#       define BIT_FF_EN                                                0x80
#define MPUREG_INT_STATUS                               0x3A
// bit definitions for MPUREG_INT_STATUS (same bit pattern as above because this register shows what interrupt actually fired)
#       define BIT_RAW_RDY_INT                                  0x01
#       define BIT_DMP_INT                                              0x02
#       define BIT_UNKNOWN_INT                                  0x04
#       define BIT_I2C_MST_INT                                  0x08
#       define BIT_FIFO_OFLOW_INT                               0x10
#       define BIT_ZMOT_INT                                             0x20
#       define BIT_MOT_INT                                              0x40
#       define BIT_FF_INT                                               0x80
#define MPUREG_ACCEL_XOUT_H                             0x3B
#define MPUREG_ACCEL_XOUT_L                             0x3C
#define MPUREG_ACCEL_YOUT_H                             0x3D
#define MPUREG_ACCEL_YOUT_L                             0x3E
#define MPUREG_ACCEL_ZOUT_H                             0x3F
#define MPUREG_ACCEL_ZOUT_L                             0x40
#define MPUREG_TEMP_OUT_H                               0x41
#define MPUREG_TEMP_OUT_L                               0x42
#define MPUREG_GYRO_XOUT_H                              0x43
#define MPUREG_GYRO_XOUT_L                              0x44
#define MPUREG_GYRO_YOUT_H                              0x45
#define MPUREG_GYRO_YOUT_L                              0x46
#define MPUREG_GYRO_ZOUT_H                              0x47
#define MPUREG_GYRO_ZOUT_L                              0x48
#define MPUREG_USER_CTRL                                0x6A
// bit definitions for MPUREG_USER_CTRL
#       define BIT_USER_CTRL_SIG_COND_RESET             0x01            // resets signal paths and results registers for all sensors (gyros, accel, temp)
#       define BIT_USER_CTRL_I2C_MST_RESET              0x02            // reset I2C Master (only applicable if I2C_MST_EN bit is set)
#       define BIT_USER_CTRL_FIFO_RESET                 0x04            // Reset (i.e. clear) FIFO buffer
#       define BIT_USER_CTRL_DMP_RESET                  0x08            // Reset DMP
#       define BIT_USER_CTRL_I2C_IF_DIS                 0x10            // Disable primary I2C interface and enable SPI interface
#       define BIT_USER_CTRL_I2C_MST_EN                 0x20            // Enable MPU to act as the I2C Master to external slave sensors
#       define BIT_USER_CTRL_FIFO_EN                    0x40            // Enable FIFO operations
#       define BIT_USER_CTRL_DMP_EN                     0x80            // Enable DMP operations
#define MPUREG_PWR_MGMT_1                               0x6B
#       define BIT_PWR_MGMT_1_CLK_INTERNAL              0x00            // clock set to internal 8Mhz oscillator
#       define BIT_PWR_MGMT_1_CLK_XGYRO                 0x01            // PLL with X axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_YGYRO                 0x02            // PLL with Y axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_ZGYRO                 0x03            // PLL with Z axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_EXT32KHZ              0x04            // PLL with external 32.768kHz reference
#       define BIT_PWR_MGMT_1_CLK_EXT19MHZ              0x05            // PLL with external 19.2MHz reference
#       define BIT_PWR_MGMT_1_CLK_STOP                  0x07            // Stops the clock and keeps the timing generator in reset
#       define BIT_PWR_MGMT_1_TEMP_DIS                  0x08            // disable temperature sensor
#       define BIT_PWR_MGMT_1_CYCLE                             0x20            // put sensor into cycle mode.  cycles between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL
#       define BIT_PWR_MGMT_1_SLEEP                             0x40            // put sensor into low power sleep mode
#       define BIT_PWR_MGMT_1_DEVICE_RESET              0x80            // reset entire device
#define MPUREG_PWR_MGMT_2                               0x6C            // allows the user to configure the frequency of wake-ups in Accelerometer Only Low Power Mode
#define MPUREG_BANK_SEL                                 0x6D            // DMP bank selection register (used to indirectly access DMP registers)
#define MPUREG_MEM_START_ADDR                   0x6E            // DMP memory start address (used to indirectly write to dmp memory)
#define MPUREG_MEM_R_W                                  0x6F            // DMP related register
#define MPUREG_DMP_CFG_1                                0x70            // DMP related register
#define MPUREG_DMP_CFG_2                                0x71            // DMP related register
#define MPUREG_FIFO_COUNTH                              0x72
#define MPUREG_FIFO_COUNTL                              0x73
#define MPUREG_FIFO_R_W                                 0x74
#define MPUREG_WHOAMI                                   0x75

#define BITS_WHOAMI_MASK                                0x7E
#define BITS_WHOAMI_OFFSET                              1

// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BITS_DLPF_CFG_256HZ_NOLPF2              0x00
#define BITS_DLPF_CFG_188HZ                             0x01
#define BITS_DLPF_CFG_98HZ                              0x02
#define BITS_DLPF_CFG_42HZ                              0x03
#define BITS_DLPF_CFG_20HZ                              0x04
#define BITS_DLPF_CFG_10HZ                              0x05
#define BITS_DLPF_CFG_5HZ                               0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF              0x07
#define BITS_DLPF_CFG_MASK                              0x07

// Product ID Description for MPU6000
// high 4 bits  low 4 bits
// Product Name	Product Revision
#define MPU6000ES_UNDEF                         0x00    // 0000			0000
#define MPU6000ES_REV_C4                        0x14    // 0001			0100
#define MPU6000ES_REV_C5                        0x15    // 0001			0101
#define MPU6000ES_REV_D6                        0x16    // 0001			0110
#define MPU6000ES_REV_D7                        0x17    // 0001			0111
#define MPU6000ES_REV_D8                        0x18    // 0001			1000
#define MPU6000_REV_C4                          0x54    // 0101			0100
#define MPU6000_REV_C5                          0x55    // 0101			0101
#define MPU6000_REV_D6                          0x56    // 0101			0110
#define MPU6000_REV_D7                          0x57    // 0101			0111
#define MPU6000_REV_D8                          0x58    // 0101			1000
#define MPU6000_REV_D9                          0x59    // 0101			1001

// DMP output rate constants
#define MPU6000_200HZ                           0x00    // default value
#define MPU6000_100HZ                           0x01
#define MPU6000_66HZ                            0x02
#define MPU6000_50HZ                            0x03

// DMP FIFO constants
#define FIFO_PACKET_SIZE 18            // Default quaternion FIFO size (4*4) + Footer(2)
#define GYRO_BIAS_FROM_GRAVITY_RATE 4  // Rate of the gyro bias from gravity correction (200Hz/4) => 50Hz
#define DEFAULT_ACCEL_FUSION_GAIN       0x80    // Default gain for accel fusion (with gyros)

#ifndef IMU_MPU6000_SPI_CS_PIN
#define IMU_MPU6000_SPI_CS_PIN  (53)
#endif

namespace hw {

HalImuMpu6000::HalImuMpu6000(
		hw::SpiBus& spiBus,
		T_GYR_CNF gyrCnf,
		T_ACC_CNF accCnf,
		T_UPT_FREQ frequence)
: HalImu(),
  _spiBus(spiBus),
  _intRdyCnt(0),
  _gyrCnf(gyrCnf),
  _accCnf(accCnf),
  _frequence(frequence),
  _productId(0)
{

	/* Set gyro LSB */
	switch(_gyrCnf){
	case E_GYR_CNF_250DPS:
		_gyrLsb = GYRO_SCALE_250DPS;
		break;
	case E_GYR_CNF_500DPS:
		_gyrLsb = GYRO_SCALE_500DPS;
		break;
	case E_GYR_CNF_1000DPS:
		_gyrLsb = GYRO_SCALE_1000DPS;
		break;
	case E_GYR_CNF_2000DPS:
		_gyrLsb = GYRO_SCALE_2000DPS;
		break;
	}

	/* Set acco LSB */
	switch(_accCnf) {
	case E_ACC_CNF_2G:
		_accLsb = PHYSICS_GRAVITY/16364.;
		break;
	case E_ACC_CNF_4G:
		_accLsb = PHYSICS_GRAVITY/8192.;
		break;
	case E_ACC_CNF_8G:
		_accLsb = PHYSICS_GRAVITY/4096.;
		break;
	case E_ACC_CNF_16G:
		_accLsb = PHYSICS_GRAVITY/2048.;
		break;
	}

}

HalImuMpu6000::~HalImuMpu6000() {
}

/** @brief Init the process */
bool HalImuMpu6000::initialize()
{
	/* Set SS as OUTPUT */
	pinMode(IMU_MPU6000_SPI_CS_PIN, OUTPUT);

    // Wake up device and select GyroZ clock (better performance)
    register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO);

    // only used for wake-up in accelerometer only low power mode
    register_write(MPUREG_PWR_MGMT_2, 0x00);

    // FS & DLPF   de-activate the filtering
    register_write(MPUREG_CONFIG, BITS_DLPF_CFG_256HZ_NOLPF2);

    // Sample rate (since no filter activated, the sampling rate is
    // 8kHz instead of 1kHz)
    //register_write(MPUREG_SMPLRT_DIV, (_freq+1)*8-1);
//    register_write(MPUREG_SMPLRT_DIV, _freq);
//    register_write(MPUREG_SMPLRT_DIV, 0x4F);
    // 1KHz = 8KHz / (1+SMPLRT_DIV)
    // 1KHz * (1+SMPLRT_DIV) = 8KHz
    //  SMPLRT_DIV = (8KHz - 1KHz) / 1KHz = 7
    register_write(MPUREG_SMPLRT_DIV, 7);

    // Gyro scale
    register_write(MPUREG_GYRO_CONFIG, _gyrCnf);

    // read the product ID rev c has 1/2 the sensitivity of rev d
    _productId = register_read(MPUREG_PRODUCT_ID);

    // Accelerometer scale
    if ((_productId == MPU6000ES_REV_C4) || (_productId == MPU6000ES_REV_C5) ||
        (_productId == MPU6000_REV_C4)   || (_productId == MPU6000_REV_C5)) {
        // Rev C has different scaling than rev D
        register_write(MPUREG_ACCEL_CONFIG,_accCnf<<2);
    } else {
        register_write(MPUREG_ACCEL_CONFIG,_accCnf<<3);
    }

    // Configure FIFO buffer
    register_write(MPUREG_FIFO_EN, BIT_ACCEL_FIFO_EN | BIT_ZG_FIFO_EN | BIT_YG_FIFO_EN | BIT_XG_FIFO_EN);

    // Disable I2C bus (recommended on datasheet)
    // and enable FIFO
    register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_I2C_IF_DIS | BIT_USER_CTRL_FIFO_EN);

    // Disable all interrupt
    register_write(MPUREG_INT_ENABLE, 0);

    // Clear count
    _intRdyCnt = 0;

    // Attach interrupt
    HalImuMpu6000::_imu = this;
    attachInterrupt(6,handleReadyInterrupt,RISING);

    // clear interrupt on any status read
    register_write(MPUREG_INT_PIN_CFG, BIT_INT_RD_CLEAR);

//    // configure interrupt to fire when new data arrives
//    register_write(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);

    // configure interrupt to fire on FIFP overflow
    register_write(MPUREG_INT_ENABLE, BIT_FIFO_OFLOW_EN);

    return HalImu::initialize();
}
/** @brief Reset the process */
void HalImuMpu6000::reset()
{
    // Chip reset
    register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);

    // Wait 100 ms
    infra::Task::delay(configTICK_RATE_HZ / 10);

    return HalImu::reset();
}
/** @brief Execute the process */
bool HalImuMpu6000::sample (
		math::Vector3i& rate_U,
		math::Vector3i& acc_U,
		int16_t& temp)
{
	int16_t data[6];
	int32_t dataLong[6];
	uint16_t fifoCount = 0;
	int32_t read = 0;

	if(_intRdyCnt!=0)
	{
		uint8_t status = register_read(MPUREG_INT_STATUS);
		if (status & BIT_FIFO_OFLOW_INT)
		{
			register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_FIFO_RESET);
			register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_FIFO_EN);
			_intRdyCnt = 0;
			return false;
		}
	}

	memset(dataLong, 0, sizeof(int32_t[6]));

	_spiBus.read(IMU_MPU6000_SPI_CS_PIN, MPUREG_FIFO_COUNTH | 0x80, 1, &fifoCount);
	uint8_t addr = (MPUREG_FIFO_R_W | 0x80) ;
	while (fifoCount >= 12)
	{
		read++;
		fifoCount -= 12;
		uint32_t value;
		/* Read data */
		_spiBus.readn(IMU_MPU6000_SPI_CS_PIN,addr,12,(uint8_t*)&data[0]);

		dataLong[0] += (int32_t) data[0];
		dataLong[1] += (int32_t) data[1];
		dataLong[2] += (int32_t) data[2];
		dataLong[3] += (int32_t) data[3];
		dataLong[4] += (int32_t) data[4];
		dataLong[5] += (int32_t) data[5];

	}

	if (read != 0)
	{

		/* Store raw linear acceleration */
		acc_U(
				 (int16_t) (dataLong[4] / read),  //  Y
				 (int16_t) (dataLong[5] / read),  //  X
				-(int16_t) (dataLong[3] / read)); // -Z

		/* Store angular rate */
		rate_U(
				 (int16_t) (dataLong[1] / read),  //  Y
				 (int16_t) (dataLong[2] / read),  //  X
				-(int16_t) (dataLong[0] / read)); // -Z
	}

	/* temperature not used */
	temp = 0;

	return (read != 0);
}


void HalImuMpu6000::register_write(uint8_t reg, uint8_t val) const
{
    uint8_t dataIn[2] = {reg, val};
    _spiBus.transfer(IMU_MPU6000_SPI_CS_PIN, dataIn, 2, NULL);
}

uint8_t HalImuMpu6000::register_read( uint8_t reg ) const
{
    uint8_t return_value;

    _spiBus.transfer8(IMU_MPU6000_SPI_CS_PIN, reg | 0x80, return_value, true);

    return return_value;
}

/** @brief Imu for interrupt handling */
HalImuMpu6000* HalImuMpu6000::_imu = NULL;

/** @brief Handle data ready interrupt */
void HalImuMpu6000::handleReadyInterrupt(void)
{
	_imu->_intRdyCnt++;
}

void HalImuMpu6000::getGyrOffsets(math::Vector3i& gyrOffsets_U) const
{
	gyrOffsets_U(
			// Y
			(int16_t) (register_read(MPUREG_YG_OFFS_USRH)<<8 | register_read(MPUREG_YG_OFFS_USRL)),
			// X
			(int16_t) (register_read(MPUREG_XG_OFFS_USRH)<<8 | register_read(MPUREG_XG_OFFS_USRL)),
			// Z
			-(int16_t) (register_read(MPUREG_ZG_OFFS_USRH)<<8 | register_read(MPUREG_ZG_OFFS_USRL)));
}
void HalImuMpu6000::setGyrOffsets(const math::Vector3i& gyrOffsets_U)
{
    // Y
    register_write(MPUREG_YG_OFFS_USRH, (gyrOffsets_U.x >> 8) & 0xFF);
    register_write(MPUREG_YG_OFFS_USRL, (gyrOffsets_U.x >> 0) & 0xFF);

    // X
    register_write(MPUREG_XG_OFFS_USRH, (gyrOffsets_U.y >> 8) & 0xFF);
    register_write(MPUREG_XG_OFFS_USRL, (gyrOffsets_U.y >> 0) & 0xFF);

    // -Z
    register_write(MPUREG_ZG_OFFS_USRH, ((-gyrOffsets_U.z) >> 8) & 0xFF);
    register_write(MPUREG_ZG_OFFS_USRL, ((-gyrOffsets_U.z) >> 0) & 0xFF);
}

void HalImuMpu6000::getAccOffsets(math::Vector3i& accOffsets_U) const
{
	accOffsets_U(
			// Y
			(int16_t) (register_read(MPUREG_YA_OFFS_H)<<8 | register_read(MPUREG_YA_OFFS_L)),
			// X
			(int16_t) (register_read(MPUREG_XA_OFFS_H)<<8 | register_read(MPUREG_XA_OFFS_L)),
			// -Z
			- (int16_t) (register_read(MPUREG_ZA_OFFS_H)<<8 | register_read(MPUREG_ZA_OFFS_L)));
}

void HalImuMpu6000::setAccOffsets(const math::Vector3i& accOffsets_U)
{
    // Y
    register_write(MPUREG_YA_OFFS_H, (accOffsets_U.x >> 8) & 0xFF);
    register_write(MPUREG_YA_OFFS_L, (accOffsets_U.x >> 0) & 0xFF);

    // X
    register_write(MPUREG_XA_OFFS_H, (accOffsets_U.y >> 8) & 0xFF);
    register_write(MPUREG_XA_OFFS_L, (accOffsets_U.y >> 0) & 0xFF);

    // -Z
    register_write(MPUREG_ZA_OFFS_H, ((-accOffsets_U.z) >> 8) & 0xFF);
    register_write(MPUREG_ZA_OFFS_L, ((-accOffsets_U.z) >> 0) & 0xFF);
}


} /* namespace hw */
