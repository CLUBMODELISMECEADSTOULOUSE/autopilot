/*
 * Imu.cpp
 *
 *  Created on: 3 févr. 2016
 *      Author: AdministrateurLocal
 */

#include <sens/imu/Imu.hpp>
#include <system/system/System.hpp>

#define IMU_CALIB_MAX_NB_SAMPLE	(64)

namespace sensors {

Imu::Imu(hw::HalImu& hal, const Parameter& param)
: _hal(hal),
  _param(param),
  _nbMissMeas(0),
  _state(E_IMU_STATE_UNDEF),
  _gyroBias_U(0,0,0),
  _calibNbSample(0)
{
}

Imu::~Imu()
{
}

/** @brief initialize IMU */
void Imu::initialize()
{
	_nbMissMeas = 0;
	math::Vector3i nullVect(0,0,0);
	_hal.setGyrOffsets(nullVect);
	_hal.initialize();
	_calibNbSample = 0;
	_state = E_IMU_STATE_INIT;
}

/** @brief Execute IMU */
void Imu::execute()
{
	switch (_state)
	{
	case E_IMU_STATE_UNDEF:
		executeStateUndefined();
		break;

	case E_IMU_STATE_INIT:
		executeStateInit();
		break;

	case E_IMU_STATE_CALIB:
		executeStateCalib();
		break;

	case E_IMU_STATE_CALIB_VERIF:
		executeStateCalibVerif();
		break;

	case E_IMU_STATE_OPER:
		executeStateOper();
		break;

	case E_IMU_STATE_FAILED:
		executeStateFailed();
		break;

	default:
		initialize();
		break;
	}
}

/** @brief Execute State Undefined */
void Imu::executeStateUndefined()
{
	/* Initialize the IMU */
	initialize();
}

/** @brief Execute State Init */
void Imu::executeStateInit()
{
	/* Wait until the IMU hal has initialized */
	if (_hal.isInitialized())
	{
		system::DataPool& dataPool = system::system.dataPool;
		processMeasurements();

		if (!dataPool.imuIsMeasAvail)
		{
			return;
		}

		/* Set offsets to current measurement */
		_hal.setGyrOffsets(system::system.dataPool.imuRateRaw_U);

		/* Ready for Gyro calibration */
		_state = E_IMU_STATE_CALIB;

		/* Reset the calibration */
		resetCalib();
	}
}

/** @brief Execute State gyro calibration */
void Imu::executeStateCalib()
{
	system::DataPool& dataPool = system::system.dataPool;
	processMeasurements();

	if (!dataPool.imuIsMeasAvail)
	{
		/* Something goes wrong ? */
		return;
	}

	/* Sum measurements in datapool */
	_gyroBias_U += dataPool.imuRateRaw_U;

	/* Check end of calibration  */
	if ( IMU_CALIB_MAX_NB_SAMPLE <= (++_calibNbSample) )
	{
		/* The count has been reached, end the calibration */
		_gyroBias_U /= _calibNbSample;

		/* Set the offset in Hal */
		{
			math::Vector3i gyroBias_U;
			_hal.getGyrOffsets(gyroBias_U);
			_gyroBias_U += gyroBias_U;
			_hal.setGyrOffsets(_gyroBias_U);
		}

		/* Clear measurement */
		_gyroBias_U(0,0,0);

		/* Clear calibration counter */
		_calibNbSample = 0;

		/* Switch to calibration verification */
		_state = E_IMU_STATE_CALIB_VERIF;
	}
}

/** @brief Execute State Operational */
void Imu::executeStateCalibVerif()
{
	system::DataPool& dataPool = system::system.dataPool;
	processMeasurements();

	if (!dataPool.imuIsMeasAvail)
	{
		return;
	}

	/* Sum variance in datapool */
	_gyroBias_U(
			_gyroBias_U.x + dataPool.imuRateRaw_U.x * dataPool.imuRateRaw_U.x,
			_gyroBias_U.y + dataPool.imuRateRaw_U.y * dataPool.imuRateRaw_U.y,
			_gyroBias_U.z + dataPool.imuRateRaw_U.z * dataPool.imuRateRaw_U.z);

	/* Check end of calibration and increment calibration counter*/
	if ( IMU_CALIB_MAX_NB_SAMPLE <= (++_calibNbSample) )
	{

		/* The count has been reached, end the calibration */
		_gyroBias_U /= _calibNbSample;

		if (
				( math_abs(_gyroBias_U.x) < _param.gyroNoiseVar )
			 && ( math_abs(_gyroBias_U.y) < _param.gyroNoiseVar )
			 && ( math_abs(_gyroBias_U.z) < _param.gyroNoiseVar ) )
		{
			/* calibration is successful, switch to operational state */
			_state = E_IMU_STATE_OPER;
		}
		else
		{
			/* something goes wrong during the calibration, repeat calibration */
			resetCalib();

			/* State is back to calibration */
			_state = E_IMU_STATE_CALIB;
		}
	}
}

/** @brief Execute State Operational */
void Imu::executeStateOper()
{
	processMeasurements();
}

/** @brief Execute State Failed */
void Imu::executeStateFailed()
{
	/* Do nothing */
}


/** @brief Reset the calibration */
void Imu::resetCalib()
{
	/* Reset calibration sample count */
	_calibNbSample = 0;

	/* Clear bias measurement */
	_hal.getGyrOffsets(_gyroBias_U);
	_hal.setGyrOffsets(_gyroBias_U + system::system.dataPool.imuRateRaw_U);
	_gyroBias_U(0,0,0);
}

void Imu::processMeasurements()
{
	system::DataPool& dataPool = system::system.dataPool;

	/* Sample the Imu measurements */
	dataPool.imuIsMeasAvail = _hal.sample(
				dataPool.imuRateRaw_U,
				dataPool.imuAccRaw_U,
				dataPool.imuTemp);

	/* Convert the measurement from raw to IS unit */
	dataPool.imuRate_B(
			((float) dataPool.imuRateRaw_U.x) * _param.gyroLsb,
			((float) dataPool.imuRateRaw_U.y) * _param.gyroLsb,
			((float) dataPool.imuRateRaw_U.z) * _param.gyroLsb);

	dataPool.imuAcc_B(
			((float) dataPool.imuAccRaw_U.x) * _param.accoLsb,
			((float) dataPool.imuAccRaw_U.y) * _param.accoLsb,
			((float) dataPool.imuAccRaw_U.z) * _param.accoLsb);
}


} /* namespace sensors */
