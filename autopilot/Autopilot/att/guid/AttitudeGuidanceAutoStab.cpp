/*
 * AttitudeGuidanceAutoStab.cpp
 *
 *  Created on: 27 ao�t 2015
 *      Author: AdministrateurLocal
 */
#include <stdio.h>
#include <system/system/System.hpp>

#include <att/guid/AttitudeGuidanceAutoStab.hpp>
#include <system/params/Nrd.hpp>

namespace attitude {

const autom::SecondOrderFilter::Parameter paramSecOrderFiltAttGuid = {
		1.98250832e-02,
		3.96501664e-02,
		1.98250832e-02,
		1.56731055e+00,
		-6.46610882e-01
};

AttitudeGuidanceAutoStab::AttitudeGuidanceAutoStab()
: AttitudeGuidance(),
  _angRollPrev(0.),
  _angPitchPrev(0.),
  _angYawPrev(0.),
  _attNormInv(1.),
  _chanFiltersRoll(paramSecOrderFiltAttGuid),
  _chanFiltersPitch(paramSecOrderFiltAttGuid),
  _chanFiltersYaw(paramSecOrderFiltAttGuid)

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
	_chanFiltersRoll.reset(_angRollPrev);
	_chanFiltersPitch.reset(_angPitchPrev);
	_chanFiltersYaw.reset(0.);
}

/** @brief Compute the guidance */
void AttitudeGuidanceAutoStab::calcGuidance(
		hw::Radio& radio,
		math::Quaternion& guidQuat_IB,
		math::Vector3f& guidRate_B,
		bool updateYaw)
{
	/* Compute yaw rate from PWM */
	float rateYaw = _chanFiltersYaw.apply(((float)radio.getSigned(hw::Radio::E_RADIO_CHANNEL_YAW))*_paramScalePwm[ATTITUDE_GUIDANCE_IDX_YAW]);

	/* Case no yaw */
	if (!updateYaw)
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
	float angRoll = _chanFiltersRoll.apply(((float)radio.getSigned(hw::Radio::E_RADIO_CHANNEL_ROLL))*_paramScalePwm[ATTITUDE_GUIDANCE_IDX_ROLL]);
	float angPitch = _chanFiltersPitch.apply(((float)radio.getSigned(hw::Radio::E_RADIO_CHANNEL_PITCH))*_paramScalePwm[ATTITUDE_GUIDANCE_IDX_PITCH]);

	math::Quaternion guidQuat_BYawB(
			1 - ldexpf(angRoll*angRoll + angPitch*angPitch, -1),
			angRoll - angRoll*angRoll*angRoll * 0.1666667,
			angPitch - angPitch*angPitch*angPitch * 0.1666667,
			0.);

	/* Combine rotations */
	guidQuat_IB(guidQuat_IBYaw * guidQuat_BYawB);
	_attNormInv = guidQuat_IB.normalize(1,_attNormInv);

	/* Compute rate */
	guidRate_B(
			(angRoll - _angRollPrev) * 20.,
			(angPitch - _angPitchPrev) * 20.,
			rateYaw);

	_angRollPrev = angRoll;
	_angPitchPrev = angPitch;
}


} /* namespace attitude */
