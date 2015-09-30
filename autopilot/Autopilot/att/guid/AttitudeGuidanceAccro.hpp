/*
 * AttitudeGuidanceAccro.hpp
 *
 *  Created on: 27 ao�t 2015
 *      Author: AdministrateurLocal
 */

#ifndef ATT_GUID_ATTITUDEGUIDANCEACCRO_HPP_
#define ATT_GUID_ATTITUDEGUIDANCEACCRO_HPP_

#include <hw/radio/Radio.hpp>
#include "AttitudeGuidance.hpp"

#define ATTITUDE_GUIDANCE_MODE_RATE_PWM_SCALE_ROLL 		(0.0034907) /* rad/s/pwm */
#define ATTITUDE_GUIDANCE_MODE_RATE_PWM_SCALE_PITCH 	(0.0034907) /* rad/s/pwm */
#define ATTITUDE_GUIDANCE_MODE_RATE_PWM_SCALE_YAW	 	(0.0013090) /* rad/s/pwm */

namespace attitude {

class AttitudeGuidanceAccro : public AttitudeGuidance {
public:
	AttitudeGuidanceAccro();
	virtual ~AttitudeGuidanceAccro();

	/** @brief Initialize internal variable */
	virtual void initialize(
			const math::Quaternion& quat_IB,
			const math::Vector3f& rate_B);

	/** @brief Compute the guidance */
	void calcGuidance(
			hw::Radio& radio,
			math::Quaternion& guidQuat_IB,
			math::Vector3f& guidRate_B);

protected:

	/** @brief Scale parameter in Rate mode */
	float _paramScalePwm[3];

	/** @brief Norm */
	float _attNormInv;
};

} /* namespace attitude */

#endif /* ATT_GUID_ATTITUDEGUIDANCEACCRO_HPP_ */
