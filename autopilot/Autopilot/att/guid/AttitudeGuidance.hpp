/*
 * AttitudeGuidance.hpp
 *
 *  Created on: 21 août 2015
 *      Author: AdministrateurLocal
 */

#ifndef ATT_GUID_ATTITUDEGUIDANCE_HPP_
#define ATT_GUID_ATTITUDEGUIDANCE_HPP_

#include <math/Quaternion.hpp>
#include <hw/pwm/Pwm.hpp>
#include <infra/mode/Mode.hpp>

#define ATTITUDE_GUIDANCE_IDX_ROLL   (0)
#define ATTITUDE_GUIDANCE_IDX_PITCH  (1)
#define ATTITUDE_GUIDANCE_IDX_YAW    (2)

namespace attitude {

class AttitudeGuidance : public infra::Mode {
public:
	typedef enum {
		E_MODE_NONE = 0,
		E_MODE_AUTOSTAB,
		E_MODE_AUTOSTAB_NOYAW,
		E_MODE_ACCRO
	} Mode;

public:

	/** @brief Initialize mode manager */
	static void initialize();

	/** @brief Set new mode */
	static void setMode(Mode mode);

	/** @brief Execute current step */
	static void executeStateMachine();

protected:

	/** @brief Current mode identifier */
	static Mode _currentModeIdentifier;

	/** @brief Current mode */
	static AttitudeGuidance* _currentMode;

public:
	AttitudeGuidance();
	virtual ~AttitudeGuidance();

	/** @brief Verify if transition to this mode is possible */
	virtual bool isReady() ;

protected:

	/** @brief Execute current step */
	virtual void execute() = 0 ;

	/** @brief Activated on leaving the mode by mode manager state */
	virtual void onLeave() = 0 ;

	/** @brief Activated on entering the mode by mode manager */
	virtual void onEnter() = 0 ;

};


} /* namespace attitude */

#endif /* ATT_GUID_ATTITUDEGUIDANCE_HPP_ */
