/*
 * ModeIdle.cpp
 *
 *  Created on: 6 nov. 2015
 *      Author: AdministrateurLocal
 */

#include <mode/control/ModeIdle.hpp>
#include <system/system/System.hpp>

namespace system {

ModeIdle::ModeIdle()
: ModeControl()
{
}

ModeIdle::~ModeIdle()
{
}

/** Execute current step */
void ModeIdle::execute(ModeControl::Step step)
{
	/* Don't mind, this mode command null torque / force */
	system::system.dataPool.ctrlFrcDemB(0,0,0);
	system::system.dataPool.ctrlTrqDemB(0,0,0);

	/* In addition, it force PWM to MIN value */
	for (uint8_t iMotor = 1 ; iMotor < CNF_NB_MOTORS ; iMotor++)
	{
		system::system.dataPool.pwm_outputs[iMotor] = MIN_PULSEWIDTH;
	}
}

/** Activated on entering the mode by mode manager */
void ModeIdle::onEnter()
{
	/* Call super */
	ModeControl::onEnter();

	/* Change Attitude guidance mode */
	attitude::AttitudeGuidance::setMode(attitude::AttitudeGuidance::E_MODE_NONE);
}

} /* namespace system */
