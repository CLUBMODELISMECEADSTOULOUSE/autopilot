/*
 * ModeStabilized.hpp
 *
 *  Created on: 2 janv. 2014
 *      Author: Robotique
 */

#ifndef MODESTABILIZED_HPP_
#define MODESTABILIZED_HPP_

#include <autom/mgt/include/Mode.hpp>
#include <autom/gen/include/GenericParameters.hpp>
#include <autom/proc/include/ProcDetectContact.hpp>

namespace autom {

class ModeStabilized: public autom::Mode {
public:
	/*
	 * PWM scale is computed as PWM*num*2^exp
	 */
	typedef struct
	{
		ControllerPid3Axes::Param attCtrl;
		int8_t rollPwmScale;
		int8_t rollPwmScaleExp; /* Scale and exp (typically 11 and -13 to obtain +/-45deg of inclination) */
		int8_t pitchPwmScale;
		int8_t pitchPwmScaleExp; /* Scale and exp (typically 11 and -13 to obtain +/-45deg of inclination) */
		int8_t yawRatePwmScale;
		int8_t yawRatePwmScaleExp; /* Scale and exp (typically 11 and -13 to obtain +/-45deg/s of rate) */
		int8_t thrustPwmScale; /* */
		int8_t thrustPwmScaleExp; /* Scale and exp (typically 1 and -6 for [0 2g]) */
		float thrustDir_B_x;
		float thrustDir_B_y;
		float thrustDir_B_z;
	} Param ;
public:
	ModeStabilized(
			/* Input */
			const Estimator::Estimations& est,
			const ProcDetectContact::Output& groundDetect,
			/* Outputs */
			AttGuid::Output& attGuid,
			::math::Vector3f& force_B,
			/* Parameters */
			const float& dt,
			const autom::ModeStabilized::Param& param,
			const GenericParam& paramGen,
			/* Dependencies */
			AttitudeController& attCtrl
			);
	virtual ~ModeStabilized();

	/** @brief Init the process */
	virtual ::infra::status initialize();

	/** @brief Execute the process */
	virtual ::infra::status execute();

protected:

	/* @brief Ground detection */
	const ProcDetectContact::Output& _groundDetect;

	/** @brief Time step duration */
	const float& _dt;

	/** @brief Parameters */
	const autom::ModeStabilized::Param& _param;

	/** @brief Generic parameters */
	AttitudeController& _attCtrl;

	/** @brief Generic parameters */
	const autom::GenericParam& _paramGen;

	/** @brief Rotation around z */
	math::Quaternion _rotZ;

	/** @brief Previous inverse of rotZ norm */
	float _prevInvNormRotZ;

	/** @brief Previous inverse of guidance norm */
	float _prevInvNormGuid;

	/** @brief Previous roll angle */
	float _angleRollPrev;

	/** @brief Previous pitch angle */
	float _anglePitchPrev;

	/** @brief Previous thrust */
	float _thrustPrev;
};

} /* namespace autom */

#endif /* MODESTABILIZED_HPP_ */