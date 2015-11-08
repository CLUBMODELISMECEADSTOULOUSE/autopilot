/*
 * ModeControlManager.cpp
 *
 *  Created on: 8 nov. 2015
 *      Author: AdministrateurLocal
 */

#include <mode/control/ModeControlManager.hpp>

#define STEP_PERIOD	(5)

namespace system {

ModeControlManager::ModeControlManager()
: _modeIdle(),
  _modeAutoStab(),
  _currentModeIdentifier(E_MODE_IDLE),
  _currentMode(_modeIdle),
  _currentStep(0)
{
	/* Enter modeIdle */
	_currentMode.onEnter();

	/* Reset step */
	resetStep();
}

ModeControlManager::~ModeControlManager()
{
}

/** @brief Set new mode */
void ModeControlManager::setMode(Mode mode)
{
	if (mode != _currentModeIdentifier)
	{
		/* Next mode */
		ModeControl& next = _currentMode;

		switch (mode)
		{
		case E_MODE_AUTOSTAB:
			next = _modeAutoStab;
			break;
		case E_MODE_IDLE:
			next = _modeIdle;
			break;
		default:
			/* Invalid control mode */
			// TODO Signal invalid mode to FDIR

			/* Ignore command */
			return;

			break;
		}
		/* Leave current mode */
		_currentMode.onLeave();

		/* Set new mode */
		_currentMode = next;

		/* Enter new mode */
		_currentMode.onEnter();

		/* Reset step to ensure that next time
		 * navigation will be computed */
		resetStep();
	}
}

/** @brief Execute current step */
void ModeControlManager::execute()
{
	/* Execute current mode step */
	_currentMode.execute(getStep());

	/* Update step */
	updateStep();
}

/** @brief Update step */
ModeControl::Step ModeControlManager::getStep()
{
	/* Default set to attitude */
	ModeControl::Step result = ModeControl::E_STEP_ATTITUDE;

	/* If counter is zero, then time to process navigation */
	if (_currentStep == 0)
		result = ModeControl::E_STEP_NAVIGATION;

	/* return result */
	return result;
}

/** @brief Reset step */
void ModeControlManager::resetStep()
{
	/* Set current step  */
	_currentStep = 0;
}

/** @brief Update step */
void ModeControlManager::updateStep()
{
	/* Update counter modulo STEP_PERIOD */
	if (++_currentStep == STEP_PERIOD)
		_currentStep = 0;
}

} /* namespace system */
