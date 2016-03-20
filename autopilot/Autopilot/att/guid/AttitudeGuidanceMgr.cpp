/*
 * AttitudeGuidanceMgr.cpp
 *
 *  Created on: 20 mars 2016
 *      Author: AdministrateurLocal
 */

#include <stddef.h>
#include <att/guid/AttitudeGuidanceMgr.hpp>

namespace attitude {

AttitudeGuidanceMgr::AttitudeGuidanceMgr()
: _currentModeIdentifier(E_MODE_NONE),
  _currentMode(NULL),
  _modeAccro(),
  _modeAutoStab()
{
}

AttitudeGuidanceMgr::~AttitudeGuidanceMgr()
{
	/* Consider that current mode is left */
	if (_currentMode != NULL)
		_currentMode->onLeave();
}


/** @brief Initialize mode manager */
void AttitudeGuidanceMgr::initialize()
{
	_currentMode = NULL;
	_currentModeIdentifier = E_MODE_NONE;
}

/** @brief Set new mode */
void AttitudeGuidanceMgr::setMode(AttitudeGuidanceMgr::Mode mode)
{
	AttitudeGuidanceMode* next = NULL;

	/* Check if demanded mode is current */
	if (_currentModeIdentifier == mode)
		/* Then, nothing to do */
		return;

	/* Check current mode */
	switch (mode)
	{
	/* NONE */
	case E_MODE_NONE:
		next = NULL;
		break;

	/* ACCRO */
	case E_MODE_ACCRO:
		next = &_modeAccro;
		break;

	/* AUTO STABILIZED without yaw control */
	case E_MODE_AUTOSTAB:
		next = &_modeAutoStab;
		_modeAutoStab.setYawUpdate(true);
		break;

	/* AUTO STABILIZED withou yaw control */
	case E_MODE_AUTOSTAB_NOYAW:
		next = &_modeAutoStab;
		_modeAutoStab.setYawUpdate(false);
		break;

	/* default (error management) */
	default:
		/* TODO signal invalid mode before returning */
		return;
		break;
	}

	/* Leave old mode */
	if (_currentMode != NULL)
	{
		_currentMode->onLeave();
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
void AttitudeGuidanceMgr::execute()
{
	if (_currentMode != NULL)
	{
		_currentMode->execute();
	}
}

} /* namespace infra */
