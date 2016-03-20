/*
 * AttitudeGuidanceMode.hpp
 *
 *  Created on: 21 août 2015
 *      Author: AdministrateurLocal
 */

#ifndef ATT_GUID_ATTITUDEGUIDANCEMODE_HPP_
#define ATT_GUID_ATTITUDEGUIDANCEMODE_HPP_

#include <math/Quaternion.hpp>
#include <hw/pwm/Pwm.hpp>
#include <infra/mode/Mode.hpp>

#define ATTITUDE_GUIDANCE_IDX_ROLL   (0)
#define ATTITUDE_GUIDANCE_IDX_PITCH  (1)
#define ATTITUDE_GUIDANCE_IDX_YAW    (2)

namespace attitude {

class AttitudeGuidanceMode : public infra::Mode {
public:
	AttitudeGuidanceMode();
	virtual ~AttitudeGuidanceMode();

	/** @brief Execute current step */
	virtual void execute() = 0 ;

	/** @brief Activated on leaving the mode by mode manager state */
	virtual void onLeave() = 0 ;

	/** @brief Activated on entering the mode by mode manager */
	virtual void onEnter() = 0 ;

};


} /* namespace attitude */

#endif /* ATT_GUID_ATTITUDEGUIDANCEMODE_HPP_ */
