/*
 * SimpleAttitudeKalmanFilter.hpp
 *
 *  Created on: 9 déc. 2013
 *      Author: Robotique
 */

#ifndef SIMPLEATTITUDEKALMANFILTER_HPP_
#define SIMPLEATTITUDEKALMANFILTER_HPP_

#include <math/Quaternion.hpp>

namespace autom {

class SimpleAttitudeKalmanFilter {
public:
	typedef struct
	{
		float gainAttAngAcco;
		float gainAttDriftAcco;
		float gainAttAngCompass;
		float gainAttDriftCompass;
	} Parameter ;

	typedef enum
	{
		E_STATE_INIT = 0,
		E_STATE_OPER_ATT,
		E_STATE_OPER_ATT_ALT,
		E_STATE_OPER_ATT_NAV
	} State;

public:
	SimpleAttitudeKalmanFilter(const Parameter& param);
	virtual ~SimpleAttitudeKalmanFilter();

	/** @brief Init the process */
	void initialize() ;

	/** @brief Execute estimation */
	void execute() ;

	/** @brief Get current state */
	inline State getState();

protected:

	/** @brief Compute attitude (common to all oper mode) */
	void computeAttitude();

	/** @brief Process initialization state */
	void processStateInit();

	/** @brief Process Attitude state */
	void processStateAtt();

	/** @brief Process Attitude and Altitude */
	void processStateAttAlt();

	/** @brief Process Attitude and Navigation */
	void processStateAttNav();

	/** @brief Switch to initialization state */
	void switchToStateInit();

	/** @brief Switch to Attitude state */
	void switchToStateAtt();

	/** @brief Switch to Attitude and Altitude */
	void switchToStateAttAlt();

	/** @brief Switch to Attitude and Navigation */
	void switchToStateAttNav();

	/** @brief Init the attitude */
	void initAttitude() ;

	/** @brief Init the attitude */
	void initAltitude() ;

	/** @brief Init the navigation */
	void initNavigation() ;

	/** @brief Convert pressure to altitude */
	void convertPressureToAltitude(float& alt, const float& pressure, const float& temperature) const;


	/** @brief Get ground pressure from current pressure, altitude and temperature */
	void getGroundPressure(float& pressure0, const float& alt, const float& pressure, const float& temperature) const;

	/** @brief Get altitude from pressure and temperature */
	void getAltitude(float& alt, const float& pressure, const float& temperature, const float& pressure0) const;

	/** @brief Get pressure from altitude and temperature */
	void getPressure(float& pressure, const float& altitude, const float& temperature, const float& pressure0) const;

protected:

	/** @brief Counter used for time out */
	uint8_t _timer;

	/** @brief Inverse of attitude norm incremental computation */
	float _attEstInvNrmPrev;

	/** @brief Imu rate drift */
	math::Vector3f _drift_B;

	/** @brief Compass direction in inertial frame */
	math::Vector3f _magDir_I;

	/** @brief Parameters values */
	const Parameter& _param;

	/** @brief Current state */
	State _state;

	/** @brief Is reference atmospheric pressure initialized */
	bool _isRefPressureInit;

	/** @brief Is reference position initialized */
	bool _isRefPositionInit;
};

/** @brief Get current state */
SimpleAttitudeKalmanFilter::State SimpleAttitudeKalmanFilter::getState()
{
	return _state;
}


} /* namespace autom */

#endif /* SIMPLEATTITUDEKALMANFILTER_HPP_ */
