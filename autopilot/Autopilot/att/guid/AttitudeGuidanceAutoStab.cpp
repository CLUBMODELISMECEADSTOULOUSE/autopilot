/*
 * AttitudeGuidanceAutoStab.cpp
 *
 *  Created on: 27 août 2015
 *      Author: AdministrateurLocal
 */
#include <stdio.h>
#include <system/system/System.hpp>

#include <att/guid/AttitudeGuidanceAutoStab.hpp>
#include <system/params/Nrd.hpp>

namespace attitude {

AttitudeGuidanceAutoStab::AttitudeGuidanceAutoStab()
: AttitudeGuidance(),
  _angRollPrev(0.),
  _angPitchPrev(0.),
  _angYawPrev(0.),
  _attNormInv(1.)
{
	_paramScalePwm[ATTITUDE_GUIDANCE_IDX_ROLL]  = ATTITUDE_GUIDANCE_MODE_AUTOSTAB_PWM_SCALE_ROLL;
	_paramScalePwm[ATTITUDE_GUIDANCE_IDX_PITCH] = ATTITUDE_GUIDANCE_MODE_AUTOSTAB_PWM_SCALE_PITCH;
	_paramScalePwm[ATTITUDE_GUIDANCE_IDX_YAW]   = ATTITUDE_GUIDANCE_MODE_AUTOSTAB_PWM_SCALE_YAW;
}

AttitudeGuidanceAutoStab::~AttitudeGuidanceAutoStab()
{
}

/** @brief Initialize internal variable */
void AttitudeGuidanceAutoStab::initialize(
		const math::Quaternion& quat_IB,
		const math::Vector3f& rate_B)
{
	_attNormInv = 1./quat_IB.norm();

	/* Initialize the Yaw angle from current estimate */
	math::Vector3f z(0.,0.,1.);
	math::Vector3f z_B = quat_IB.rotateQconjVQ(z);

	float dotProd = z * z_B;
	math::Vector3f crossProd = z % z_B;
	float crossProdNorm = crossProd.norm();
	if (crossProdNorm != 0.)
		crossProd /= crossProdNorm;
	else
		crossProd(1.,0.,0.);

	float angle = atan2f(crossProdNorm,dotProd);

	math::Quaternion q_BB1(
		cosf(angle/2),
		-crossProd*sinf(angle/2));

	math::Quaternion q_IB1 = quat_IB*q_BB1;


	_angRollPrev = -crossProd.x;
	_angPitchPrev = -crossProd.y;
	_angYawPrev = ldexpf(atan2f(q_IB1.vector.z,q_IB1.scalar),1);
}

/** @brief Compute the guidance */
void AttitudeGuidanceAutoStab::execute()
{
	hw::Radio& radio = system::system.getRadio();
	system::DataPool& datapool = system::system.dataPool;
	math::Vector3f& guidRate_B = datapool.guidRate_B;
	math::Quaternion& guidQuat_IB = datapool.guidAtt_IB;

	/* Compute yaw rate from PWM */
	float rateYaw = ((float)radio.getSigned(hw::Radio::E_RADIO_CHANNEL_YAW))*_paramScalePwm[ATTITUDE_GUIDANCE_IDX_YAW];

	/* Case no yaw */
	if (!_updateYaw)
		rateYaw = 0.;

	/* Build yaw angle by integrating pwm input */
	_angYawPrev = _angYawPrev + rateYaw*FSW_TASK_CTRL_PERIOD_TICK_PER_SEC;
	if (_angYawPrev > M_PI)
		_angYawPrev -= M_PI + M_PI;
	else if (-180. > _angYawPrev)
		_angYawPrev += M_PI + M_PI;

	/* Yaw rotation */
	math::Quaternion guidQuat_IBYaw(
			cosf(ldexpf(_angYawPrev,-1)),
			0.,
			0.,
			sinf(ldexpf(_angYawPrev, -1)));

	/* Build seat rotation */
	float angRoll = ((float)radio.getSigned(hw::Radio::E_RADIO_CHANNEL_ROLL))*_paramScalePwm[ATTITUDE_GUIDANCE_IDX_ROLL];
	float angPitch = ((float)radio.getSigned(hw::Radio::E_RADIO_CHANNEL_PITCH))*_paramScalePwm[ATTITUDE_GUIDANCE_IDX_PITCH];

//	math::Quaternion guidQuat_BYawB(
//			1 - ldexpf(angRoll*angRoll + angPitch*angPitch, -1),
//			angRoll - angRoll*angRoll*angRoll * 0.1666667,
//			angPitch - angPitch*angPitch*angPitch * 0.1666667,
//			0.);
	math::Quaternion guidQuat_BYawB(
			1,
			angRoll,
			angPitch,
			0.);
	guidQuat_BYawB /= guidQuat_BYawB.norm();

	/* Combine rotations */
	guidQuat_IB(guidQuat_IBYaw * guidQuat_BYawB);
//	_attNormInv = guidQuat_IB.normalize(1,_attNormInv);

	/* Compute rate */
	guidRate_B(
			(angRoll - _angRollPrev) * 20.,
			(angPitch - _angPitchPrev) * 20.,
			rateYaw);

	_angRollPrev = angRoll;
	_angPitchPrev = angPitch;
}

/** @brief Activated on entering the mode by mode manager */
void AttitudeGuidanceAutoStab::onEnter()
{
	system::DataPool& dataPool = system::system.dataPool;

	/* Then initialize internal data using current estimation */
	initialize(dataPool.estAtt_IB, dataPool.estRate_B);
}

/** @brief Activated on leaving the mode by mode manager state */
void AttitudeGuidanceAutoStab::onLeave()
{
}




} /* namespace attitude */
