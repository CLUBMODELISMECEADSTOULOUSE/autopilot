/*
 * ModeControl.cpp
 *
 *  Created on: 8 nov. 2015
 *      Author: AdministrateurLocal
 */

#include <mode/control/ModeControl.hpp>
#include <system/system/System.hpp>

#include "ModeIdle.hpp"
#include "ModeAutoStab.hpp"

#include "ModeControlData.hpp"

namespace system {

/* Instantiation of attitude controller */
attitude::AttitudeController ModeControl::_attCtrl(paramAttCtrl);

/* Instantiation of navigation controller */
navigation::NavigationController ModeControl::_navCtrl(paramNavCtrl);

/* Instantiation of attitude guidance manager */
attitude::AttitudeGuidanceManager ModeControl::_attGuidStateMachine;

/* Instantiation of navigation guidance manager */
navigation::NavigationGuidanceManager ModeControl::_navGuidStateMachine;

/* Instantiation of modulator */
autom::ModulatorLut ModeControl::_modulator(paramMod);

/** @brief Mode Idle */
static ModeIdle _modeIdle;

/** @brief Mode Autostab */
static ModeAutoStab _modeAutoStab;

/* Initialize current mode variable */
ModeControl& ModeControl::_currentMode = _modeIdle;

/* Set current identifier as E_MODE_IDLE */
ModeControl::Mode ModeControl::_currentModeIdentifier = E_MODE_IDLE;

/* Set current step to zero */
uint8_t ModeControl::_currentStep = 0;

/** @brief Initialize mode manager */
void ModeControl::initialize()
{
	/* Initialize modulator */
	_modulator.initialize();

	/* Enter modeIdle */
	_currentMode.onEnter();

	/* Reset step */
	resetStep();
}

/** @brief Set new mode */
void ModeControl::setMode(Mode mode)
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
void ModeControl::execute()
{
	/* Execute current mode step */
	_currentMode.execute(getStep());

	/* Update step */
	updateStep();
}

/** @brief Reset step */
void ModeControl::resetStep()
{
	_currentStep = 0 ;
}

/** @brief Get current step */
ModeControl::Step ModeControl::getStep()
{
	/* Default is attitude control */
	ModeControl::Step result = E_STEP_ATTITUDE;

	/* If current step is zero, then navigation */
	if (_currentStep == 0)
		result = E_STEP_NAVIGATION;

	/* Return result */
	return result;
}

/** @brief Update current step */
void ModeControl::updateStep()
{
	if (++_currentStep >= CNF_CONTROL_MODE_NAV_PERIOD)
		_currentStep = 0;
}


ModeControl::ModeControl()
: system::Mode()
{
}

ModeControl::~ModeControl()
{
}

/** @brief Activated on entering the mode by mode manager */
void ModeControl::onEnter()
{
	/* Reset all controllers to avoid side effect due to internal state
	 * mind that some transitory effect may result from this */
	_attCtrl.reset();
	_navCtrl.reset();
}

/** @brief Execute current step */
void ModeControl::execute(Step step)
{
	switch (step)
	{
	case E_STEP_NONE:
		/* Nothing to do */
		break;

	case E_STEP_NAVIGATION:

		/* Process navigation step */
		stepNavigation();

		/* and then ... */

	case E_STEP_ATTITUDE:

		/* Process attitude step */
		stepAttitude();

		break;

	default:

		/* unexpected */
		// TODO: add error report
		break;

	}
}

/** @brief Activated on leaving the mode by mode manager state */
void ModeControl::onLeave()
{
	/* Nothing foreseen for the moment ! */
}

/** @brief Execute attitude step */
void ModeControl::stepAttitude()
{
	/* Local reference to datapool */
	system::DataPool& datapool = system::system.dataPool;

	/* In this step process attitude controller only
	 * assuming guidance is done at lower rate (nav
	 * guidance step) */

	/* Update guidance */
	_attGuidStateMachine.execute();

	/* Update Contol */
	_attCtrl.execute();

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
}

/** @brief Execute navigation step */
void ModeControl::stepNavigation()
{
	/* Update navigation guidance */
	_navGuidStateMachine.execute();

	/* Update navigation control */
	_navCtrl.execute();

	/* Update attitude control
	 * (low frequency / may require nav info */
	_attGuidStateMachine.execute();
}



} /* namespace system */
