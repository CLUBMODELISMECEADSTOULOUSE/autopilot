/*
 * ModeControl.cpp
 *
 *  Created on: 8 nov. 2015
 *      Author: AdministrateurLocal
 */

#include <mode/control/ModeControl.hpp>
#include <system/system/System.hpp>

namespace system {

ModeControl::ModeControl()
: Mode()
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
	/* Local reference to datapool */
	system::DataPool& datapool = system::system.dataPool;

	switch (step)
	{
	case E_STEP_NONE:
		/* Nothing to do */
		break;

	case E_STEP_NAVIGATION:

		/* Update navigation guidance */
		_navGuidStateMachine.execute();

		/* Update navigation control */
		_navCtrl.execute();

		/* Update attitude control
		 * (low frequency / may require nav info */
		_attGuidStateMachine.execute();

	case E_STEP_ATTITUDE:

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

} /* namespace system */
