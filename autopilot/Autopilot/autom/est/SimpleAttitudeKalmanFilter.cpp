/*
 * SimpleAttitudeKalmanFilter.cpp
 *
 *  Created on: 9 déc. 2013
 *      Author: Robotique
 */

#include <gcs/channel/mavlink_bridge.hpp>
#include <system/params/Nrd.hpp>
#include <autom/est/SimpleAttitudeKalmanFilter.hpp>
#include <system/system/System.hpp>

#define EST_MAX_CONSECUTIVE_MISSING_MEAS_GYRO (3)
#define EST_MAX_CONSECUTIVE_MISSING_MEAS_ACCO (3)
#define EST_MAX_CONSECUTIVE_MISSING_MEAS_COMPASS (3)
#define EST_MAX_CONSECUTIVE_MISSING_MEAS_BARO (3)
#define EST_MAX_CONSECUTIVE_MISSING_MEAS_GPS (3)

namespace autom {


SimpleAttitudeKalmanFilter::SimpleAttitudeKalmanFilter(const Parameter& param)
: _nbMissGyroMeas(0),
  _nbMissAccoMeas(0),
  _nbMissCompassMeas(0),
  _nbMissBaroMeas(0),
  _nbMissGpsMeas(0),
  _attEstInvNrmPrev(1.),
  _isAttInitialized(false),
  _isPosInitialized(false),
  _drift_B(0.,0.,0.),
  _magDir_I(1.,0.,0.),
  _param(param)
{
}

SimpleAttitudeKalmanFilter::~SimpleAttitudeKalmanFilter()
{
}

/** @brief Init the process */
void SimpleAttitudeKalmanFilter::initialize()
{
	system::DataPool& dataPool = system::system.dataPool;
	dataPool.estAttValid = false;
	dataPool.estPosValid = false;
	dataPool.estAtt_IB(1.,0.,0.,0.);
	dataPool.estRate_B(0.,0.,0.);
	dataPool.estPos_I(0.,0.,0.);
	dataPool.estVel_I(0.,0.,0.);
	_isAttInitialized = false;
	_isPosInitialized = false;
}

/** @brief Estimate */
void SimpleAttitudeKalmanFilter::update()
{
	system::DataPool& dataPool = system::system.dataPool;

	/* Check measurements */
	if (!_isAttInitialized)
		checkMeasurements();

	if (!_isAttInitialized)
		return;

	math::Quaternion quatAttPred_IB;
	math::Vector3f velPred_I;
	math::Vector3f posPred_I;
	math::Vector3f innovAngle(0.,0.,0.);
	math::Vector3f innovDrift(0.,0.,0.);

	/* Predict the attitude */
	{
		dataPool.estRate_B = dataPool.imuRate_B + _drift_B;
		math::Vector3f angInc = dataPool.estRate_B * (0.5 * FSW_TASK_CTRL_PERIOD_TICK_PER_SEC);;
		math::Quaternion dQ(
				1 - (angInc*angInc) * 0.5,
				angInc.x - angInc.x*angInc.x*angInc.x * 0.1666667,
				angInc.y - angInc.y*angInc.y*angInc.y * 0.1666667,
				angInc.z - angInc.z*angInc.z*angInc.z * 0.1666667);
		quatAttPred_IB = dataPool.estAtt_IB * dQ;
		_attEstInvNrmPrev = quatAttPred_IB.normalize(1, _attEstInvNrmPrev);
	}


	/* Predict the Position and Velocity in local inertial frame */
	{
		math::Vector3f acc_I = dataPool.estAtt_IB.rotateQVQconj(dataPool.imuAcc_B);
		acc_I.z -= 9.81;
		dataPool.estVel_I = dataPool.estVel_I + (acc_I * FSW_TASK_CTRL_PERIOD_TICK_PER_SEC);
		dataPool.estPos_I = dataPool.estPos_I + (velPred_I * FSW_TASK_CTRL_PERIOD_TICK_PER_SEC);
	}

	/* Compute attitude innovation from gravity (an accelerometer only see
	 * acceleration resulting from contact efforts */
	if (_nbMissAccoMeas == 0)
	{
		math::Vector3f accDir_B_pred = quatAttPred_IB.rotateQconjVQ(math::Vector3f(0., 0., -1.));
		math::Vector3f accDir_B(dataPool.imuAcc_B);
		accDir_B /= accDir_B.norm();
		math::Vector3f innov_B = (accDir_B % accDir_B_pred) ;
		innovAngle += innov_B * (0.5 * _param.gainAttAngAcco);
		innovDrift += innov_B * _param.gainAttDriftAcco;
	}

	/* Compute attitude innovation from compass */
	if (_nbMissCompassMeas == 0)
	{
		math::Vector3f magDirPred_B = quatAttPred_IB.rotateQconjVQ(_magDir_I);
		math::Vector3f innov_B = (dataPool.compassMag_B % magDirPred_B) / dataPool.compassMag_B.norm() ;
		innovAngle += innov_B * (0.5 * _param.gainAttAngCompass);
		innovDrift += innov_B * _param.gainAttDriftCompass;
	}

	/* Correct attitude using innovations */
	{
		math::Quaternion qCorr(1., innovAngle);
		dataPool.estAtt_IB = quatAttPred_IB*qCorr;
		_attEstInvNrmPrev = dataPool.estAtt_IB.normalize(1, _attEstInvNrmPrev);
		dataPool.estAtt_IB.to_dcm(dataPool.estDcm_IB);
	}

	/* Correct rate using innovations */
	{
		_drift_B += innovDrift;
		dataPool.estRate_B += innovDrift;
	}

	/* Correct Altitude using barometer */
	if (_nbMissBaroMeas == 0)
	{
		// TODO Implement altitude correction using barometer
	}
	/* Correct Position / Velocity using GPS */
	if (_nbMissGpsMeas == 0)
	{
		// TODO Implement Position / Vecolity correction using GPS
	}
}

/** @brief Init the attitude */
void SimpleAttitudeKalmanFilter::initAttitude()
{
	system::DataPool& dataPool = system::system.dataPool;
	math::Vector3f x_B;
	math::Vector3f y_B;
	math::Vector3f z_B;

	_attEstInvNrmPrev = 1.;

	z_B = -dataPool.imuAcc_B / dataPool.imuAcc_B.norm();
	x_B = z_B % (dataPool.compassMag_B % z_B);
	float x_B_norm = x_B.norm();

	if (x_B_norm == 0.)
		return;

	x_B /= x_B_norm;
	y_B = z_B % x_B;

	dataPool.estDcm_IB(x_B,y_B,z_B);
	_attEstInvNrmPrev = 1.;

	dataPool.estAtt_IB.from_dcm(dataPool.estDcm_IB);

	_magDir_I = dataPool.estAtt_IB.rotateQVQconj(dataPool.compassMag_B);
	_magDir_I /= _magDir_I.norm();

	_drift_B = -dataPool.imuRate_B;

	dataPool.estRate_B = dataPool.imuRate_B - _drift_B;

	dataPool.estAttValid = true;
	_isAttInitialized = true;
}

/** @brief Init the position */
void SimpleAttitudeKalmanFilter::initPosition()
{
}


/** @brief Check measurements */
void SimpleAttitudeKalmanFilter::checkMeasurements()
{
	system::DataPool& dataPool = system::system.dataPool;
	uint32_t now = micros();

	/* Check measurements from Imu */
	if (dataPool.imuLastMeasDateUsec + (FSW_TASK_CTRL_PERIOD_TICK_PER_MSEC*1000) < now)
	{
		if ((++_nbMissGyroMeas) > EST_MAX_CONSECUTIVE_MISSING_MEAS_GYRO)
			dataPool.estAttValid = false;

		if ((++_nbMissAccoMeas) > EST_MAX_CONSECUTIVE_MISSING_MEAS_ACCO)
			dataPool.estAttValid = false;
	}
	else
	{
		_nbMissGyroMeas = 0;
		_nbMissAccoMeas = 0;
	}

	/* Check compass */
	if ((dataPool.compassLastMeasDateUsec + (FSW_TASK_CTRL_PERIOD_TICK_PER_MSEC*1000)) < now)
	{
		if (++_nbMissCompassMeas > EST_MAX_CONSECUTIVE_MISSING_MEAS_COMPASS)
			dataPool.estAttValid = false;
	}
	else
	{
		_nbMissCompassMeas = 0;
	}

	/* Check barometer */
	if (dataPool.baroLastMeasDateUsec + (FSW_TASK_CTRL_PERIOD_TICK_PER_MSEC*1000) < now)
	{
		if (++_nbMissBaroMeas > EST_MAX_CONSECUTIVE_MISSING_MEAS_COMPASS)
			dataPool.estAttValid = false;
	}
	else
	{
		_nbMissCompassMeas = 0;
	}

	/* Check GPS */
	// TODO: add check for GPS

	/* Assess if reinitialization are required */
	if (   (!dataPool.estAttValid)
		&& (_nbMissGyroMeas==0)
		&& (_nbMissAccoMeas==0)
		&& (_nbMissCompassMeas==0))
	{
		/* Reinitialization of attitude */
		initAttitude();
	}
	if (   (!dataPool.estPosValid)
		&& (_nbMissGpsMeas==0)
		&& (_nbMissAccoMeas==0)
		&& (_nbMissBaroMeas==0))
	{
		/* Reinitialization of position */
		initPosition();
	}
}



} /* namespace autom */
