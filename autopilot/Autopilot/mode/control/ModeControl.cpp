/*
 * ModeControl.cpp
 *
 *  Created on: 8 nov. 2015
 *      Author: AdministrateurLocal
 */

#include <mode/control/ModeControl.hpp>

namespace system {

ModeControl::ModeControl()
: Mode()
{
}

ModeControl::~ModeControl()
: Mode ()
{
}

/** Activated on entering the mode by mode manager */
void ModeControl::onEnter()
{
	/* Reset all controllers to avoid side effect due to internal state
	 * mind that some transitory effect may result from this */
	_attCtrl.reset();
	_navCtrl.reset();
}

/** Execute current step */
void ModeControl::execute(E_STEP step)
{
	switch (step)
	{
	case E_STEP_NONE:
		/* Nothing to do */
		break;

	case E_STEP_ATTITUDE:

		/* In this step process attitude controller only
		 * assuming guidance is done at lower rate (nav
		 * guidance step) */

		/* Update guidance */
		_attGuidStateMachine.execute();

		/* Update Contol */
		_attCtrl.execute();

		break;

	case E_STEP_NAVIGATION:

		/* Update navigation guidance */
		_navGuidStateMachine.execute();

		/* Update navigation control */
		_navCtrl.execute();

		/* Update attitude control
		 * (low frequency / may require nav info */
		_attGuidStateMachine.execute();

		break;

	default:

		/* unexpected */
		// TODO: add error report
		break;

	}
}

/** Activated on leaving the mode by mode manager state */
void ModeControl::onLeave()
{
	/* Nothing foreseen for the moment ! */
}

} /* namespace system */
