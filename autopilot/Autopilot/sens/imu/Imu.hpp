/*
 * Imu.hpp
 *
 *  Created on: 3 févr. 2016
 *      Author: AdministrateurLocal
 */

#ifndef SENS_IMU_IMU_HPP_
#define SENS_IMU_IMU_HPP_

#include <hw/imu/HalImu.hpp>

namespace sensors {

class Imu {
public:
	typedef enum
	{
		E_IMU_STATE_UNDEF = 0,
		E_IMU_STATE_INIT,
		E_IMU_STATE_CALIB,
		E_IMU_STATE_CALIB_VERIF,
		E_IMU_STATE_OPER,
		E_IMU_STATE_FAILED
	} State ;

	typedef struct
	{
		/** @brief Gyro noise variance to check calibration is ok */
		uint16_t gyroNoiseVar;
		/** @brief Gyro lsb*/
		float gyroLsb;
		/** @brief Accelerometer LSB */
		float accoLsb;
	} Parameter ;

public:
	Imu(hw::HalImu& hal, const Parameter& param);
	virtual ~Imu();

	/** @brief Get number of missed measurements */
	inline uint8_t getNbMissMeas();

	/** @brief Get current IMU state */
	inline State getState();

	/** @brief initialize IMU */
	virtual void initialize();

	/** @brief Execute IMU */
	virtual void execute();

protected:

	/** @brief Reset the calibration */
	void resetCalib();

	/** @brief Execute State Undefined */
	virtual void executeStateUndefined();

	/** @brief Execute State Init */
	virtual void executeStateInit();

	/** @brief Execute State calibration */
	virtual void executeStateCalib();

	/** @brief Execute State calibration verification */
	virtual void executeStateCalibVerif();

	/** @brief Execute State Operational */
	virtual void executeStateOper();

	/** @brief Execute State Failed */
	virtual void executeStateFailed();

	void processMeasurements();

protected:
	/** @brief IMU HAL */
	hw::HalImu& _hal;

	/** @brief Imu parameter */
	const Parameter& _param;

	/** @brief Number of missed measurements */
	uint8_t _nbMissMeas;

	/** @brief Current state */
	State _state;

	/** @brief Calibration samples number */
	uint8_t _calibNbSample;


	math::Vector3i _gyroBias_U;
};

/** @brief Get number of missed measurements */
uint8_t Imu::getNbMissMeas()
{
	return _nbMissMeas;
}

/** @brief Get number of missed measurements */
Imu::State Imu::getState()
{
	return _state;
}

} /* namespace sensors */

#endif /* SENS_IMU_IMU_HPP_ */
