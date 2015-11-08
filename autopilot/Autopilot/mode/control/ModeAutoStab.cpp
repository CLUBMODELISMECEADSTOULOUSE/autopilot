/*
 * ModeAutoStab.cpp
 *
 *  Created on: 6 nov. 2015
 *      Author: AdministrateurLocal
 */

#include "ModeAutoStab.hpp"

namespace system {

ModeAutoStab::ModeAutoStab()
: ModeControl()
{
}

ModeAutoStab::~ModeAutoStab()
{
}

/** Activated on entering the mode by mode manager */
void ModeAutoStab::onEnter()
{
	/* Call super */
	ModeControl::onEnter();

	/* Change Attitude guidance mode */
	ModeControl::_attGuidStateMachine.setMode(attitude::AttitudeGuidanceManager::E_MODE_AUTOSTAB_NOYAW);
}

} /* namespace system */
