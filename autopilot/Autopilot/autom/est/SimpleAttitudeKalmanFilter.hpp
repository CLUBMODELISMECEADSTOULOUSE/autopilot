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
	} Step;

public:
	SimpleAttitudeKalmanFilter(const Parameter& param);
	virtual ~SimpleAttitudeKalmanFilter();

	/** @brief Init the process */
	virtual void initialize() ;

	/** @brief Estimate */
	virtual void update() ;

protected:

	/** @brief Check measurements */
	virtual void checkMeasurements() ;

	/** @brief Init the attitude */
	virtual void initAttitude() ;

	/** @brief Init the position */
	virtual void initPosition() ;

	/** @brief Consecutive missing gyro measurements */
	uint8_t _nbMissGyroMeas;

	/** @brief Consecutive missing acco measurements */
	uint8_t _nbMissAccoMeas;

	/** @brief Consecutive missing compass measurements */
	uint8_t _nbMissCompassMeas;

	/** @brief Consecutive missing Baro measurements */
	uint8_t _nbMissBaroMeas;

	/** @brief Consecutive missing Gps measurements */
	uint8_t _nbMissGpsMeas;

	float _attEstInvNrmPrev;
	bool _isAttInitialized;
	bool _isPosInitialized;

	math::Vector3f _drift_B;
	math::Vector3f _magDir_I;

	const Parameter& _param;
};

} /* namespace autom */

#endif /* SIMPLEATTITUDEKALMANFILTER_HPP_ */
