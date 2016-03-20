/*
 * AttitudeGuidanceAccro.cpp
 *
 *  Created on: 27 août 2015
 *      Author: AdministrateurLocal
 */

#include "AttitudeGuidanceAccro.hpp"
#include <system/system/System.hpp>

namespace attitude {

AttitudeGuidanceAccro::AttitudeGuidanceAccro()
: AttitudeGuidance(),
  _attNormInv(1.)
{
	_paramScalePwm[ATTITUDE_GUIDANCE_IDX_ROLL]  = ATTITUDE_GUIDANCE_MODE_RATE_PWM_SCALE_ROLL;
	_paramScalePwm[ATTITUDE_GUIDANCE_IDX_PITCH] = ATTITUDE_GUIDANCE_MODE_RATE_PWM_SCALE_PITCH;
	_paramScalePwm[ATTITUDE_GUIDANCE_IDX_YAW]   = ATTITUDE_GUIDANCE_MODE_RATE_PWM_SCALE_YAW;
}

AttitudeGuidanceAccro::~AttitudeGuidanceAccro() {
}

/** @brief Initialize internal variable */
void AttitudeGuidanceAccro::initialize(
		const math::Quaternion& quat_IB,
		const math::Vector3f& rate_B)
{
	_attNormInv = 1. / quat_IB.norm();
}

/** @brief Execute current step */
void AttitudeGuidanceAccro::execute()
{
	hw::Radio& radio = system::system.getRadio();
	system::DataPool& datapool = system::system.dataPool;
	math::Vector3f& guidRate_B = datapool.guidRate_B;
	math::Quaternion& guidQuat_IB = datapool.guidAtt_IB;

	/* Build rate from pwm inputs */
	guidRate_B.x = ((float) radio.getSigned(hw::Radio::E_RADIO_CHANNEL_ROLL) )
			* _paramScalePwm[ATTITUDE_GUIDANCE_IDX_ROLL];
	guidRate_B.y = ((float) radio.getSigned(hw::Radio::E_RADIO_CHANNEL_PITCH) )
			* _paramScalePwm[ATTITUDE_GUIDANCE_IDX_PITCH];
	guidRate_B.z = ((float) radio.getSigned(hw::Radio::E_RADIO_CHANNEL_YAW) )
			*_paramScalePwm[ATTITUDE_GUIDANCE_IDX_YAW];

	/* Integrate rate */
	math::Vector3f angInc = guidRate_B * (0.5 * FSW_TASK_CTRL_PERIOD_TICK_PER_SEC);;
	math::Quaternion dQ(
			1 - (angInc*angInc) * 0.5,
			angInc.x - angInc.x*angInc.x*angInc.x * 0.1666667,
			angInc.y - angInc.y*angInc.y*angInc.y * 0.1666667,
			angInc.z - angInc.z*angInc.z*angInc.z * 0.1666667);
	guidQuat_IB(guidQuat_IB * dQ);
	_attNormInv = guidQuat_IB.normalize(1,_attNormInv);
}

/** @brief Activated on leaving the mode by mode manager state */
void AttitudeGuidanceAccro::onLeave()
{

}

/** @brief Activated on entering the mode by mode manager */
void AttitudeGuidanceAccro::onEnter()
{
	system::DataPool& dataPool = system::system.dataPool;

	/* Then initialize internal data using current estimation */
	initialize(dataPool.estAtt_IB, dataPool.estRate_B);
}

} /* namespace attitude */
