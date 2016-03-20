/*
 * AttitudeGuidanceAccro.hpp
 *
 *  Created on: 27 août 2015
 *      Author: AdministrateurLocal
 */

#ifndef ATT_GUID_ATTITUDEGUIDANCEACCRO_HPP_
#define ATT_GUID_ATTITUDEGUIDANCEACCRO_HPP_

#include <hw/radio/Radio.hpp>
#include "AttitudeGuidanceMode.hpp"

#define ATTITUDE_GUIDANCE_MODE_RATE_PWM_SCALE_ROLL 		(0.0034907) /* rad/s/pwm */
#define ATTITUDE_GUIDANCE_MODE_RATE_PWM_SCALE_PITCH 	(0.0034907) /* rad/s/pwm */
#define ATTITUDE_GUIDANCE_MODE_RATE_PWM_SCALE_YAW	 	(0.0013090) /* rad/s/pwm */

namespace attitude {

class AttitudeGuidanceAccro : public AttitudeGuidanceMode {
public:
	AttitudeGuidanceAccro();
	virtual ~AttitudeGuidanceAccro();

protected:
	/** @brief Initialize internal variable */
	virtual void initialize(
			const math::Quaternion& quat_IB,
			const math::Vector3f& rate_B);

	/** @brief Execute current step */
	virtual void execute() ;

	/** @brief Activated on leaving the mode by mode manager state */
	virtual void onLeave() ;

	/** @brief Activated on entering the mode by mode manager */
	virtual void onEnter() ;

protected:

	/** @brief Scale parameter in Rate mode */
	float _paramScalePwm[3];

	/** @brief Norm */
	float _attNormInv;
};

} /* namespace attitude */

#endif /* ATT_GUID_ATTITUDEGUIDANCEACCRO_HPP_ */
