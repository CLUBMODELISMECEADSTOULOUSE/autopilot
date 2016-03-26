/*
 * ModeControl.cpp
 *
 *  Created on: 8 nov. 2015
 *      Author: AdministrateurLocal
 */

#include <mode/control/ModeControlMgr.hpp>
#include <system/system/System.hpp>



namespace system {


ModeControl::ModeControl()
: infra::Mode()
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
	system::system.getModeCtrlMgr().getAttCtrl().reset();
	system::system.getModeCtrlMgr().getNavCtrl().reset();
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

		/* Process attitude step */
		stepAttitude();

		break;

	case E_STEP_ATTITUDE:

		/* Process attitude step only */
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
	system::ModeControlMgr& modeMgr = system::system.getModeCtrlMgr();

	/* In this step process attitude controller only
	 * assuming guidance is done at lower rate (nav
	 * guidance step) */

	/* Update Contol */
	modeMgr.getAttCtrl().execute();
}

/** @brief Execute navigation step */
void ModeControl::stepNavigation()
{
	system::ModeControlMgr& modeMgr = system::system.getModeCtrlMgr();

	/* Update navigation guidance */
	modeMgr.getNavGuidMgr().execute();

	/* Update navigation control */
	modeMgr.getNavCtrl().execute();

	/* Update attitude control
	 * (low frequency / may require nav info */
	modeMgr.getAttGuidMgr().execute();
}



} /* namespace system */
