/*
 * ModeAutoStab.cpp
 *
 *  Created on: 6 nov. 2015
 *      Author: AdministrateurLocal
 */

#include "ModeAutoStab.hpp"
#include <system/system/Dynamics.hpp>
#include <system/system/System.hpp>

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

/** @brief Execute navigation step */
void ModeAutoStab::stepNavigation()
{
	/* Direct thrust compute such that to have compensation
	 * of gravity at middle throttle */
	hw::Radio& radio = system::system.getRadio();
	float mass;
	Dynamics::getMass(mass);

	int32_t thrustCmd = (int32_t) radio.getUnsigned(hw::Radio::E_RADIO_CHANNEL_THRUST);
	int32_t throttleMiddle = - (int32_t) radio.getSignedMinVal(hw::Radio::E_RADIO_CHANNEL_THRUST);
	/* cmdMiddle <=> - mass * g
	 *       pwm <=> - mass * g * pwm / cmdMiddle
	 */
	system::system.dataPool.ctrlFrcDemB(
			0,
			0,
			(((int32_t) ldexpf(- mass * PHYSICS_GRAVITY, SCALE_TORSOR)) * thrustCmd) / throttleMiddle );

	/* Update attitude control
	 * (low frequency / may require nav info */
	_attGuidStateMachine.execute();

}


} /* namespace system */
