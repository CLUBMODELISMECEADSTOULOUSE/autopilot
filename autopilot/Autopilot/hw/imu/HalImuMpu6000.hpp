/*
 * HalImuMpu6000.hpp
 *
 *  Created on: 7 juil. 2013
 *      Author: Aberzen
 */

#ifndef HALIMUMPU6000_HPP_
#define HALIMUMPU6000_HPP_

#include "HalImu.hpp"
#include <hw/spi/SpiBus.hpp>

/*
 *  RS-MPU-6000A-00.pdf, page 33, section 4.25 lists LSB sensitivity of
 *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
#define GYRO_SCALE_250DPS (0.0174532 / 131.07)
#define GYRO_SCALE_500DPS (0.0174532 / 65.536)
#define GYRO_SCALE_1000DPS (0.0174532 / 32.768)
#define GYRO_SCALE_2000DPS (0.0174532 / 16.384)
//#define GYRO_SCALE_2000DPS (1 / 16.384)

/*
 *  RS-MPU-6000A-00.pdf, page 31, section 4.23 lists LSB sensitivity of
 *  accel as 4096 LSB/mg at scale factor of +/- 8g (AFS_SEL==2)
 *
 *  See note below about accel scaling of engineering sample MPU6k
 *  variants however
 */
#define ACC_SCALE_2G (PHYSICS_GRAVITY / 16384.)
#define ACC_SCALE_4G (PHYSICS_GRAVITY / 8192.)
#define ACC_SCALE_8G (PHYSICS_GRAVITY / 4096.)
#define ACC_SCALE_16G (PHYSICS_GRAVITY / 2048.)

/*
 *  PS-MPU-6000A.pdf, page 14, section 6.3 LSB sensitivity of
 *  temperature equals to 340 LSB/°C
 */
#define TEMP_SCALE (1./340.)
#define TEMP_OFFSET (36.53)



namespace hw {

class HalImuMpu6000: public HalImu {
public:
	typedef enum {
		E_GYR_CNF_250DPS = 0x00,
		E_GYR_CNF_500DPS = 0x08,
		E_GYR_CNF_1000DPS = 0x10,
		E_GYR_CNF_2000DPS = 0x18
	} T_GYR_CNF;

	typedef enum {
		E_ACC_CNF_2G = 0,
		E_ACC_CNF_4G = 1,
		E_ACC_CNF_8G = 2,
		E_ACC_CNF_16G = 3
	} T_ACC_CNF;

	typedef enum {
		E_UPT_FREQ_1000HZ = 0x00,
		E_UPT_FREQ_500HZ = 0x01,
		E_UPT_FREQ_250HZ = 0x03,
		E_UPT_FREQ_200HZ = 0x04,
		E_UPT_FREQ_100HZ = 0x09,
		E_UPT_FREQ_50HZ = 0x13
	} T_UPT_FREQ;

public:
	HalImuMpu6000(
			hw::SpiBus& spiBus,
			T_GYR_CNF gyrCnf,
			T_ACC_CNF accCnf,
			T_UPT_FREQ frequence);
	virtual ~HalImuMpu6000();

	/** @brief Init the process */
	virtual bool initialize();

	virtual bool sample(
			math::Vector3i& rate_U,
			math::Vector3i& acc_U,
			int16_t& temp
			);

	virtual void getGyrOffsets(math::Vector3i& gyrOffsets_U) const;
	virtual void setGyrOffsets(const math::Vector3i& gyrOffsets_U);
	virtual void getAccOffsets(math::Vector3i& accOffsets_U) const;
	virtual void setAccOffsets(const math::Vector3i& accOffsets_U);

	/** @brief Reset the process */
	virtual void reset();

protected:

	/** @brief Write the register */
	void register_write(uint8_t reg, uint8_t val) const;

	/** @brief Read the register */
	uint8_t register_read( uint8_t reg ) const;

protected:
	hw::SpiBus& _spiBus;

	/** @brief Interrupt count */
	volatile uint8_t _intRdyCnt;

	/** @brief Parameters */
	T_GYR_CNF _gyrCnf;
	T_ACC_CNF _accCnf;
	T_UPT_FREQ _frequence;

    /** @brief Configured gyro LSB */
    float _gyrLsb;

	/** @brief Configured accel LSB */
    float _accLsb;

	/** @brief Product ID */
	uint8_t _productId;

	math::Vector3i _gyroBias_U;
	math::Vector3i _accoBias_U;

protected:
	/** @brief Imu for interrupt handling */
	static HalImuMpu6000* _imu;

	/** @brief Handle data ready interrupt */
	static void handleReadyInterrupt(void);
};

} /* namespace hw */
#endif /* HALIMUMPU6000_HPP_ */
