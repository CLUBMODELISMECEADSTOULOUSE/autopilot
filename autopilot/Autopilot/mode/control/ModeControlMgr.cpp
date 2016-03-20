/*
 * ModeControlMgr.cpp
 *
 *  Created on: 20 mars 2016
 *      Author: AdministrateurLocal
 */

#include <mode/control/ModeControlMgr.hpp>
#include "ModeControlData.hpp"
#include <system/system/System.hpp>

namespace system {

ModeControlMgr::ModeControlMgr()
: _modulator(paramMod),
  _attCtrl(paramAttCtrl),
  _navCtrl(paramNavCtrl),
  _attGuidMgr(),
  _navGuidMgr(),
  _modeIdle(),
  _modeAutoStab(),
  _currentModeIdentifier(E_MODE_IDLE),
  _currentMode(&_modeIdle),
  _currentStep(0)
{
}

ModeControlMgr::~ModeControlMgr()
{
}

/** @brief Initialize mode manager */
void ModeControlMgr::initialize()
{
	/* Initialize modulator */
	_modulator.initialize();

	/* Set current mode */
	_currentMode = &_modeIdle;

	/* Enter modeIdle */
	_currentMode->onEnter();

	/* Reset step */
	resetStep();
}

/** @brief Set new mode */
void ModeControlMgr::setMode(Mode mode)
{
	if (mode != _currentModeIdentifier)
	{
		/* Next mode */
		ModeControl* next = _currentMode;

		/* Check mode */
		switch (mode)
		{
		case E_MODE_AUTOSTAB:
			next = &_modeAutoStab;
			break;

		case E_MODE_IDLE:
			next = &_modeIdle;
			break;

		default:
			/* Invalid control mode */
			// TODO Signal invalid mode to FDIR

			/* Ignore command */
			return;

			break;
		}
		/* Leave current mode */
		_currentMode->onLeave();

		/* Set new mode */
		_currentMode = next;
		_currentModeIdentifier = mode;

		/* Enter new mode */
		_currentMode->onEnter();

		/* Reset step to ensure that next time
		 * navigation will be computed */
		resetStep();
	}
}

/** @brief Execute current step */
void ModeControlMgr::execute()
{
	system::DataPool& datapool = system::system.dataPool;

	/* Execute current mode step */
	_currentMode->execute(getStep());

	/* Update modulator */
	_modulator.calcMotorCommand(
			datapool.ctrlFrcDemB,
			datapool.ctrlTrqDemB,
			datapool.pwm_outputs);

	/* Update estimation of efforts produced by the modulator */
	_modulator.calcTorsor(
			datapool.pwm_outputs,
			datapool.estForce_B,
			datapool.estTorque_B);

	/* Update step */
	updateStep();
}

/** @brief Reset step */
void ModeControlMgr::resetStep()
{
	_currentStep = 0 ;
}

/** @brief Get current step */
ModeControl::Step ModeControlMgr::getStep()
{
	/* Default is attitude control */
	ModeControl::Step result = ModeControl::E_STEP_ATTITUDE;

	/* If current step is zero, then navigation */
	if (_currentStep == 0)
		result = ModeControl::E_STEP_NAVIGATION;

	/* Return result */
	return result;
}

/** @brief Update current step */
void ModeControlMgr::updateStep()
{
	if (++_currentStep >= CNF_CONTROL_MODE_NAV_PERIOD)
		_currentStep = 0;
}


} /* namespace system */
