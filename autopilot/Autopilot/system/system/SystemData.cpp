/*
 * SystemData.cpp
 *
 *  Created on: 8 nov. 2015
 *      Author: AdministrateurLocal
 */

#include "SystemData.hpp"
#include <system/params/Nrd.hpp>
#include <system/params/NrdGen.hpp>
#include <hw/imu/HalImuMpu6000.hpp>

namespace system {

const hw::Radio::Parameter paramRadio =
{
		/* reversed */
		{0},
		/* pwmZero */
		{(MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1},
		/* pwmMin */
		{MIN_PULSEWIDTH, MIN_PULSEWIDTH, MIN_PULSEWIDTH, MIN_PULSEWIDTH, MIN_PULSEWIDTH, MIN_PULSEWIDTH, MIN_PULSEWIDTH, MIN_PULSEWIDTH},
		/* pwmMax */
		{MAX_PULSEWIDTH, MAX_PULSEWIDTH, MAX_PULSEWIDTH, MAX_PULSEWIDTH, MAX_PULSEWIDTH, MAX_PULSEWIDTH, MAX_PULSEWIDTH, MAX_PULSEWIDTH}
};

const system::Dynamics::Parameter paramDyn = {
		{K_DYN_INER_XX,K_DYN_INER_YY,K_DYN_INER_ZZ},
		{0.000,0.000,0.000}
};

const autom::SimpleAttitudeKalmanFilter::Parameter paramEst =
{
		/* gainAttAngAcco */
		EST_GAIN_ACCO_ANGLE,
		/* gainAttDriftAcco */
		EST_GAIN_ACCO_DRIFT,
		/* gainAttAngCompass */
		EST_GAIN_COMPASS_ANGLE,
		/* gainAttDriftCompass */
		EST_GAIN_COMPASS_DRIFT
};

/** @brief Imu management parameter */
const sensors::Imu::Parameter imuMgtParam =
{
		/** @brief Gyro noise variance to check calibration is ok */
		50,
		/** @brief Gyro lsb*/
		GYRO_SCALE_250DPS,
		/** @brief Accelerometer LSB */
		ACC_SCALE_4G
};



} /* namespace system */

