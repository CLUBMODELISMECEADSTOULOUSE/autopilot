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

#define EST_INIT_ATT_TIMER	(128)

#define EST_CST_C_to_K_OFFSET	(273.15)
#define EST_CST_C_to_K(C)	(C+EST_CST_C_to_K_OFFSET)

#define EST_CST_R	( 8.31451E+00 )
#define EST_CST_M	( 28.9660E-03  )
#define EST_CST_a	( 6.50000E-03  )
#define EST_CST_g	( 9.80665E-00  )
#define EST_CST_P0	( 101.325E-00  )
#define EST_CST_Ch	( 0.12000E+00 )
#define EST_CST_T0  EST_CST_C_to_K( 15 )

#define EST_CST_E00 ( 5.6402)
#define EST_CST_E01 (-0.0916)
#define EST_CST_E02 ( 0.0600)
#define EST_CST_E10 (18.2194)
#define EST_CST_E11 ( 1.0463)
#define EST_CST_E12 (-0.0666)

#define EST_CST_R_on_M 		(EST_CST_R/EST_CST_M)
#define EST_CST_Mg_on_Ra 	((EST_CST_M*EST_CST_g)/(EST_CST_R*EST_CST_a))
#define EST_CST_T0_on_a		(EST_CST_T0 / EST_CST_a)
#define EST_CST_a_on_T0		(EST_CST_a / EST_CST_T0)

namespace autom {


SimpleAttitudeKalmanFilter::SimpleAttitudeKalmanFilter(const Parameter& param)
: _timer(0),
  _attEstInvNrmPrev(1.),
  _drift_B(0.,0.,0.),
  _magDir_I(0.,0.,0.),
  _param(param),
  _state(E_STATE_INIT),
  _isRefPressureInit(false),
  _isRefPositionInit(false)
{
}

SimpleAttitudeKalmanFilter::~SimpleAttitudeKalmanFilter()
{
}

/** @brief Init the process */
void SimpleAttitudeKalmanFilter::initialize()
{
	system::DataPool& dataPool = system::system.dataPool;
	dataPool.estAtt_IB(1.,0.,0.,0.);
	dataPool.estDcm_IB.from_euler(0.,0.,0.);
	dataPool.estRate_B(0.,0.,0.);
	dataPool.estPos_I(0.,0.,0.);
	dataPool.estVel_I(0.,0.,0.);
	_isRefPressureInit = false;
	_isRefPositionInit = false;
	_timer = 0;
	switchToStateInit();
}

/** @brief Execute estimation function */
void SimpleAttitudeKalmanFilter::execute()
{
	switch (_state)
	{
	/* Initialization */
	case E_STATE_INIT:
		processStateInit();
		break;
	/* Attitude only */
	case E_STATE_OPER_ATT:
		processStateAtt();
		break;
	/* Attitude and altitude */
	case E_STATE_OPER_ATT_ALT:
		processStateAttAlt();
		break;
	/* Attitude and full navigation */
	case E_STATE_OPER_ATT_NAV:
		processStateAttAlt();
		break;
	default:
		/* Unexpected, force initialization */
		switchToStateInit();
		break;
	}
}

/** @brief Init the attitude */
void SimpleAttitudeKalmanFilter::initAttitude()
{
	/* If we are here, this mean that compass and imu measurements
	 * are available / correct
	 */

	/* Shortcut to data pool */
	system::DataPool& dataPool = system::system.dataPool;

	/* The vertical is given by the accelerometer */
	math::Vector3f z_B = -dataPool.imuAcc_B / dataPool.imuAcc_B.norm();

	/* The north is given by the compass */
	math::Vector3f x_B = z_B % (dataPool.compassMag_B % z_B);
	x_B /= x_B.norm();

	/* The third direction is deduced from the two others */
	math::Vector3f y_B = z_B % x_B;

	/* The DCM is composed of the three vectors computed above */
	dataPool.estDcm_IB(x_B,y_B,z_B);

	/* Compute the attitude quaternion using the dcm */
	dataPool.estAtt_IB.from_dcm(dataPool.estDcm_IB);

	/* Initialize the norm of the attitude to current norm */
	_attEstInvNrmPrev = dataPool.estAtt_IB.norm();
	dataPool.estAtt_IB /= _attEstInvNrmPrev;

	/* We assume that this operation can only be done on ground
	 * This mean that any rotation is due to drift and noise
	 * It should be noted that a constant pre-compensation of
	 * this drift can be set at imu HAL level.
	 */
	_drift_B = - dataPool.imuRate_B;

	/* Rate estimation is null... but is also the difference between
	 * current measurement and drift estimation */
	dataPool.estRate_B = dataPool.imuRate_B + _drift_B;
}

/** @brief Init the altitude */
void SimpleAttitudeKalmanFilter::initAltitude()
{
	/* When called, the barometer is available, and
	 * attitude is operational
	 */

	/* Shortcut to data pool */
	system::DataPool& dataPool = system::system.dataPool;

	/* Store current pressure in data pool */
	dataPool.estRefBaroPress = dataPool.baroPress;
	dataPool.estRefBaroTemp = dataPool.baroTemp;

	/* Initialize vertical position using simple baro formula */

}
/** @brief Init the navigation */
void SimpleAttitudeKalmanFilter::initNavigation()
{
}

/** @brief Process initialization state */
void SimpleAttitudeKalmanFilter::processStateInit()
{
	/* Initialize shortcuts :) */
	sensors::Imu& imu = system::system.getImuMgt();
	system::DataPool& dataPool = system::system.dataPool;
	dataPool.estAtt_IB(1.,0.,0.,0.);
	dataPool.estRate_B(0.,0.,0.);


	/* Check if Imu is in Oper Mode are available */
	if (imu.getState() != sensors::Imu::E_IMU_STATE_OPER)
	{
		return;
	}

	/* Check if measurement are available */
	if (!(dataPool.imuIsMeasAvail && dataPool.compassIsMeasAvail))
	{
		return;
	}

	/* All conditions are met to compute magnetic field in inertial frame */
	{
		/* Initialize vertical direction in body frame */
		math::Vector3f z_B = -(dataPool.imuAcc_B / dataPool.imuAcc_B.norm());

		/* compute mag in inertial frame */
		math::Vector3f mag_I (
					((z_B % (dataPool.compassMag_B % z_B))).norm(),
					0.,
					dataPool.compassMag_B * z_B);

		/* Sum the direction in internal variable in order to compute
		 * the mean when switching to operational modes */
		_magDir_I += (mag_I / mag_I.norm());
	}

	/* Increment and check the timer */
	if (EST_INIT_ATT_TIMER <= (++_timer))
	{
		/* Normalize the magnetic field direction
		 * It will be equivalent to performing the division
		 * by the number of summed elements with the additional
		 * property that the vector is correctly normalized at the end
		 */
		_magDir_I /= _magDir_I.norm();

		/* Initialize the attitude state */
		initAttitude();

		/* Time to switch to Attitude Only */
		switchToStateAtt();
	}
}

/** @brief Compute attitude (common to all oper mode) */
void SimpleAttitudeKalmanFilter::computeAttitude()
{
	/* Declare shortcut to dataPool to improve readability */
	system::DataPool& dataPool = system::system.dataPool;

	/* Declare local variables */
	/* Attitude prediction */
	math::Quaternion quatAttPred_IB;
	/* Innovation angle */
	math::Vector3f innovAngle(0.,0.,0.);
	/* Innovation drift */
	math::Vector3f innovDrift(0.,0.,0.);

	/* Predict the attitude */
	{
		/* Rate prediction (stored in rate estimation variable) */
		dataPool.estRate_B = dataPool.imuRate_B + _drift_B;

		/* Compute angular increment (small angle approximation) during
		 * one control period
		 */
		math::Vector3f angInc = dataPool.estRate_B * (0.5 * FSW_TASK_CTRL_PERIOD_TICK_PER_SEC);;

		/* Use simple limited development instead of sin/cos formula
		 * to compute the attitude increment
		 */
		math::Quaternion dQ(
				1 - (angInc*angInc) * 0.5,
				angInc.x - angInc.x*angInc.x*angInc.x * 0.1666667,
				angInc.y - angInc.y*angInc.y*angInc.y * 0.1666667,
				angInc.z - angInc.z*angInc.z*angInc.z * 0.1666667);

		/* Predict attitude */
		quatAttPred_IB = dataPool.estAtt_IB * dQ;

		/* Itterative normalization */
		_attEstInvNrmPrev = quatAttPred_IB.normalize(1, _attEstInvNrmPrev);
	}


	/* Compute attitude innovation from gravity (an accelerometer only see
	 * acceleration resulting from contact efforts */
	if (dataPool.imuIsMeasAvail)
	{
		/* Acceleration direction prediction */
		math::Vector3f accDir_B_pred = quatAttPred_IB.rotateQconjVQ(math::Vector3f(0., 0., -1.));

		/* Acceleration direction measurement */
		math::Vector3f accDir_B(dataPool.imuAcc_B);
		accDir_B /= accDir_B.norm();

		/* Compute innovation (small angle approximation */
		math::Vector3f innov_B = (accDir_B % accDir_B_pred) ;

		/* Sum correction that would be performed on predictions */
		innovAngle += innov_B * (0.5 * _param.gainAttAngAcco);
		innovDrift += innov_B * _param.gainAttDriftAcco;
	}

	/* Compute attitude innovation from compass */
	if (dataPool.compassIsMeasAvail)
	{
		/* Predicted compass direction */
		math::Vector3f magDirPred_B = quatAttPred_IB.rotateQconjVQ(_magDir_I);

		/* Compute innovation */
		math::Vector3f innov_B = (dataPool.compassMag_B % magDirPred_B) / dataPool.compassMag_B.norm() ;

		/* Sum correction that would be performed on predictions */
		/* Limit compass usage to yaw */
		innovAngle.z += innov_B.z * (0.5 * _param.gainAttAngCompass);
		innovDrift.z += innov_B.z * _param.gainAttDriftCompass;
	}

	/* Correct attitude using innovations */
	{
		/* Computed the attitude correction using precomputed
		 * angular innovation
		 */
		math::Quaternion qCorr(1., innovAngle);

		/* Correct attitude */
		dataPool.estAtt_IB = quatAttPred_IB*qCorr;

		/* Iterative normalization */
		_attEstInvNrmPrev = dataPool.estAtt_IB.normalize(1, _attEstInvNrmPrev);

		/* Compute the dcm (more efficient for vector transformation) */
		dataPool.estAtt_IB.to_dcm(dataPool.estDcm_IB);
	}

	/* Correct drift and predicted rate using innovations */
	{
		_drift_B += innovDrift;
		dataPool.estRate_B += innovDrift;
	}
}

/** @brief Process Attitude state */
void SimpleAttitudeKalmanFilter::processStateAtt()
{
	/* Declare shortcut to dataPool to improve readability */
	system::DataPool& dataPool = system::system.dataPool;

	/* Estimate attitude */
	computeAttitude();

//	/* Check if the state machine can switch to other states.
//	 * Priority given to altitude in order to ensure that the
//	 * ground pressure (more precisely the reference pressure)
//	 * is known when in full navigation
//	 */
//	if (!_isRefPressureInit)
//	{
//		/* Check if barometer measurements are available */
//		if (dataPool.baroIsMeasAvail)
//		{
//			/* Initialize reference altitude */
//			initAltitude();
//
//			/* Then switch to attitude and altitude mode */
//			switchToStateAttAlt();
//
//			/* Return since nothing to do after that */
//			return;
//		}
//	}
//	else
//	{
//		/* Priority to given to nav */
//		if (dataPool.gpsIsMeasAvail)
//		{
//			/* If not initialized, init before switching to */
//			if (!_isRefPositionInit)
//			{
//				initNavigation();
//			}
//			/* Switch to attitude and navigation */
//			switchToStateAttNav();
//
//			/* Return since nothing to do after that */
//			return;
//		}
//		/* If nav not available, try altitude */
//		if (dataPool.baroIsMeasAvail)
//		{
//			/* already initialized, just have to switch to */
//			switchToStateAttAlt();
//
//			/* Return since nothing to do after that */
//			return;
//		}
//	}

}

/** @brief Process Attitude and Altitude */
void SimpleAttitudeKalmanFilter::processStateAttAlt()
{
	/* Declare shortcut to dataPool to improve readability */
	system::DataPool& dataPool = system::system.dataPool;
	float pressurePred;

	/* Estimate attitude */
	computeAttitude();

	/* Predict the Position and Velocity in Inertial frame
	 * (consider only z component / altitude)
	 */
	{
		math::Vector3f acc_I = dataPool.estAtt_IB.rotateQVQconj(dataPool.imuAcc_B);
		acc_I.z -= 9.81;
		dataPool.estVel_I.z = dataPool.estVel_I.z + (acc_I.z * FSW_TASK_CTRL_PERIOD_TICK_PER_SEC);
		dataPool.estPos_I.z = dataPool.estPos_I.z + (dataPool.estVel_I.z * FSW_TASK_CTRL_PERIOD_TICK_PER_SEC);
	}

	/* Use predicted altitude to predict the atmospheric */
	getPressure(
			pressurePred,
			dataPool.estPos_I.z,
			dataPool.baroTemp,
			dataPool.estRefBaroPress);

	/*  */
}

/** @brief Process Attitude and Navigation */
void SimpleAttitudeKalmanFilter::processStateAttNav()
{
	/* Declare shortcut to dataPool to improve readability */
	system::DataPool& dataPool = system::system.dataPool;

	/* Estimate attitude */
	computeAttitude();

	/* Predict the Position and Velocity in local inertial frame */
	{
		math::Vector3f acc_I = dataPool.estAtt_IB.rotateQVQconj(dataPool.imuAcc_B);
		acc_I.z -= 9.81;
		dataPool.estVel_I = dataPool.estVel_I + (acc_I * FSW_TASK_CTRL_PERIOD_TICK_PER_SEC);
		dataPool.estPos_I = dataPool.estPos_I + (dataPool.estVel_I * FSW_TASK_CTRL_PERIOD_TICK_PER_SEC);
	}
}

/** @brief Switch to initialization state */
void SimpleAttitudeKalmanFilter::switchToStateInit()
{
	/* Reset the magnetic direction */
	_magDir_I(0.,0.,0.);

	/* Reset the time */
	_timer=0;

	/* Set state to initialize */
	_state = E_STATE_INIT;
}

/** @brief Switch to Attitude state */
void SimpleAttitudeKalmanFilter::switchToStateAtt()
{
	/* Reset the time */
	_timer = 0;

	/* Set state to attitude */
	_state = E_STATE_OPER_ATT;
}

/** @brief Switch to Attitude and Altitude */
void SimpleAttitudeKalmanFilter::switchToStateAttAlt()
{
	/* Reset the time */
	_timer=0;

	/* Set state to attitude and altitude */
	_state = E_STATE_OPER_ATT_ALT;
}

/** @brief Switch to Attitude and Navigation */
void SimpleAttitudeKalmanFilter::switchToStateAttNav()
{
	/* Reset the time */
	_timer=0;

	/* Set state to attitude and navigation */
	_state = E_STATE_OPER_ATT_NAV;
}

/** @brief Get altitude from pressure and temperature */
void SimpleAttitudeKalmanFilter::getAltitude(float& alt, const float& pressure, const float& temperature, const float& pressure0) const
{
	/*
	 * Formula extracted from WikiPedia:
	 * https://fr.wikipedia.org/wiki/Formule_du_nivellement_barom%C3%A9trique
	 *
	 * p0 = p(h)*e^x
	 *
	 *                   g0*h
	 * with x = -----------------------
	 *          R* (T(h) + Ch.E + ah/2)
	 *
	 * Knowing the ground pressure, the pressure becomes:
	 * p(h) = p0*e^(-x)
	 *
	 * where:
	 * - p0: the atmospheric pressure at sea level [hPa]
	 * - p(h): the atmospheric pressure at altitude h [hPa]
	 * - g0: the standard gravity acceleration on Earth [m/s^2]
	 * - R*: the specific constant of dry air (R/M) [m^2/(s^2K)]
	 * - h: altitude (m)
	 * - a: reference vertical temperature gradient [K/m]
	 * - E: Partial pressure of water vapour [hPa]
	 * - Ch: Partial pressure correction coefficient [K/h/Pa]
	 *
	 * The altitude can be deduced from current temperature using
	 * the formula:
	 *       (R*(T + Ch*E)) * log(p/p0)
	 * h = - --------------------------
	 *           g + R*a/2*log(p/p0)
	 *
	 * E can be approximated by:
	 * 		E0 = E00 * (E01 + exp(E02*t(h)) if t(h) < 9.1°C
	 * 		E1 = E10 * (E11 - exp(E12*t(h)) if t(h) >= 9.1°C
	 *
	 * for simplification purpose and discontinuity avoidance when
	 * close to 9.1°C, we fix the solution to E1.
	 *
	 */

	float E = EST_CST_E10 * (EST_CST_E11-expf(EST_CST_E12*temperature));
	float log_p_on_p0 = logf(pressure/pressure0);
	float x = EST_CST_R_on_M * (EST_CST_C_to_K(temperature) + EST_CST_Ch*E) * log_p_on_p0;
	float y = EST_CST_g + EST_CST_R_on_M*EST_CST_a/2. * log_p_on_p0;
	alt = x / y;
}

/** @brief Get pressure from altitude and temperature */
void SimpleAttitudeKalmanFilter::getPressure(float& pressure, const float& altitude, const float& temperature, const float& pressure0) const
{
	/*
	 * Formula extracted from WikiPedia:
	 * https://fr.wikipedia.org/wiki/Formule_du_nivellement_barom%C3%A9trique
	 *
	 * p0 = p(h)*e^x
	 *
	 *                   g0*h
	 * with x = -----------------------
	 *          R* (T(h) + Ch.E + ah/2)
	 *
	 * Knowing the ground pressure, the pressure becomes:
	 * p(h) = p0*e^(-x)
	 *
	 * where:
	 * - p0: the atmospheric pressure at sea level [hPa]
	 * - p(h): the atmospheric pressure at altitude h [hPa]
	 * - g0: the standard gravity acceleration on Earth [m/s^2]
	 * - R*: the specific constant of dry air (R/M) [m^2/(s^2K)]
	 * - h: altitude (m)
	 * - a: reference vertical temperature gradient [K/m]
	 * - E: Partial pressure of water vapour [hPa]
	 * - Ch: Partial pressure correction coefficient [K/h/Pa]
	 *
	 * The altitude can be deduced from current temperature using
	 * the formula:
	 *       (R*(T + Ch*E)) * log(p/p0)
	 * h = - --------------------------
	 *           g + R*a/2*log(p/p0)
	 *
	 * E can be approximated by:
	 * 		E0 = E00 * (E01 + exp(E02*t(h)) if t(h) < 9.1°C
	 * 		E1 = E10 * (E11 - exp(E12*t(h)) if t(h) >= 9.1°C
	 *
	 * for simplification purpose and discontinuity avoidance when
	 * close to 9.1°C, we fix the solution to E1.
	 *
	 */

	float E = EST_CST_E10 * (EST_CST_E11-expf(EST_CST_E12*temperature));
	float x = EST_CST_g * altitude
			/
			(EST_CST_R_on_M * (EST_CST_C_to_K(temperature) + EST_CST_Ch*E + (EST_CST_a/2.)*altitude));
	pressure = pressure0 * expf(-x);
}


/** @brief Get ground pressure from current pressure, altitude and temperature */
void SimpleAttitudeKalmanFilter::getGroundPressure(float& pressure0, const float& alt, const float& pressure, const float& temperature) const
{
	/*
	 * Formula extracted from WikiPedia:
	 * https://fr.wikipedia.org/wiki/Formule_du_nivellement_barom%C3%A9trique
	 *
	 * p0 = p(h)*e^x
	 *
	 *                   g0*h
	 * with x = -----------------------
	 *          R* (T(h) + Ch.E + ah/2)
	 *
	 * Knowing the ground pressure, the pressure becomes:
	 * p(h) = p0*e^(-x)
	 *
	 * where:
	 * - p0: the atmospheric pressure at sea level [hPa]
	 * - p(h): the atmospheric pressure at altitude h [hPa]
	 * - g0: the standard gravity acceleration on Earth [m/s^2]
	 * - R*: the specific constant of dry air (R/M) [m^2/(s^2K)]
	 * - h: altitude (m)
	 * - a: reference vertical temperature gradient [K/m]
	 * - E: Partial pressure of water vapour [hPa]
	 * - Ch: Partial pressure correction coefficient [K/h/Pa]
	 *
	 * The altitude can be deduced from current temperature using
	 * the formula:
	 *       (R*(T + Ch*E)) * log(p/p0)
	 * h = - --------------------------
	 *           g + R*a/2*log(p/p0)
	 *
	 * E can be approximated by:
	 * 		E0 = E00 * (E01 + exp(E02*t(h)) if t(h) < 9.1°C
	 * 		E1 = E10 * (E11 - exp(E12*t(h)) if t(h) >= 9.1°C
	 *
	 * for simplification purpose and discontinuity avoidance when
	 * close to 9.1°C, we fix the solution to E1.
	 *
	 */

	float E = EST_CST_E10 * (EST_CST_E11-expf(EST_CST_E12*temperature));
	float x = EST_CST_g * alt
			/
			(EST_CST_R_on_M * (EST_CST_C_to_K(temperature) + EST_CST_Ch*E + (EST_CST_a/2.)*alt));
	pressure0 = pressure * expf(x);
}

} /* namespace autom */
