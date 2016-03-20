/*
 * AttitudeGuidance.cpp
 *
 *  Created on: 21 août 2015
 *      Author: AdministrateurLocal
 */

#include <stddef.h>

#include "AttitudeGuidance.hpp"
#include "AttitudeGuidanceAutoStab.hpp"
#include "AttitudeGuidanceAccro.hpp"

#include <system/system/System.hpp>

namespace attitude {

/** @brief Mode accro */
AttitudeGuidanceAccro modeAccro;

/** @brief Mode auto stabilized */
AttitudeGuidanceAutoStab modeAutoStab;

/** @brief Current mode identifier */
AttitudeGuidance::Mode AttitudeGuidance::_currentModeIdentifier = E_MODE_NONE;

/** @brief Current mode */
AttitudeGuidance* AttitudeGuidance::_currentMode = NULL;


/** @brief Initialize mode manager */
void AttitudeGuidance::initialize()
{
	_currentMode = NULL;
	_currentModeIdentifier = E_MODE_NONE;
}

/** @brief Set new mode */
void AttitudeGuidance::setMode(AttitudeGuidance::Mode mode)
{
	if (_currentModeIdentifier == mode)
		return;

	AttitudeGuidance* next = NULL;
	switch (mode)
	{
	case E_MODE_NONE:
		break;
	case E_MODE_ACCRO:
		next = &modeAccro;
		break;
	case E_MODE_AUTOSTAB:
		next = &modeAutoStab;
		modeAutoStab.setYawUpdate(true);
		break;
	case E_MODE_AUTOSTAB_NOYAW:
		next = &modeAutoStab;
		modeAutoStab.setYawUpdate(false);
		break;
	default:
		// TODO signal invalid mode before returning
		return;
		break;
	}

	/* Leave old mode */
	if (_currentMode != NULL)
	{
		_currentMode->onLeave();
	}
	else
	{
		system::DataPool& datapool = system::system.dataPool;
		datapool.guidAtt_IB(datapool.estAtt_IB);
		datapool.guidRate_B(datapool.estRate_B);
	}

	/* Switch mode */
	_currentMode = next;
	_currentModeIdentifier = mode;

	/* Enter new mode */
	if (_currentMode != NULL)
	{
		_currentMode->onEnter();
	}
}

/** @brief Execute current step */
void AttitudeGuidance::executeStateMachine()
{
	if (_currentMode != NULL)
	{
		_currentMode->execute();
	}
}



AttitudeGuidance::AttitudeGuidance()
{
}

AttitudeGuidance::~AttitudeGuidance()
{
}

/** @brief Verify if transition to this mode is possible */
bool AttitudeGuidance::isReady()
{
	autom::SimpleAttitudeKalmanFilter& est = system::system.getEstimator();
	autom::SimpleAttitudeKalmanFilter::State state = est.getState();
	return (state == autom::SimpleAttitudeKalmanFilter::E_STATE_OPER_ATT)
		|| (state == autom::SimpleAttitudeKalmanFilter::E_STATE_OPER_ATT_ALT)
		|| (state == autom::SimpleAttitudeKalmanFilter::E_STATE_OPER_ATT_NAV);
}

} /* namespace attitude */
