/*
 * ModeControlManager.hpp
 *
 *  Created on: 8 nov. 2015
 *      Author: AdministrateurLocal
 */

#ifndef MODE_CONTROL_MODECONTROLMANAGER_HPP_
#define MODE_CONTROL_MODECONTROLMANAGER_HPP_

#include <mode/control/ModeIdle.hpp>
#include <mode/control/ModeAutoStab.hpp>

namespace system {

class ModeControlManager {
public:
	typedef enum
	{
		E_MODE_IDLE,
		E_MODE_AUTOSTAB
	} Mode ;

public:

	ModeControlManager();
	virtual ~ModeControlManager();

	/** @brief Set new mode */
	void setMode(Mode mode);

	/** @brief Execute current step */
	void execute();

protected:

	/** @brief Reset step */
	void resetStep();

	/** @brief Get current step */
	ModeControl::Step getStep();

	/** @brief Update current step */
	void updateStep();

protected:

	/** @brief Mode Idle */
	ModeIdle _modeIdle;

	/** @brief Mode Autostab */
	ModeAutoStab _modeAutoStab;

	/** @brief Current mode identifier */
	Mode _currentModeIdentifier;

	/** @brief Current mode */
	ModeControl& _currentMode;

	/** @brief Current step */
	uint8_t _currentStep;

	/**  */
};

} /* namespace system */

#endif /* MODE_CONTROL_MODECONTROLMANAGER_HPP_ */
