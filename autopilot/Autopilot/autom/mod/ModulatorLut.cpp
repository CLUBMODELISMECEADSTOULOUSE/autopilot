/*
 * ModulatorLut.cpp
 *
 *  Created on: 26 juin 2015
 *      Author: AdministrateurLocal
 */


#include "ModulatorLut.hpp"

namespace autom {

ModulatorLut::ModulatorLut(
		const ModulatorLut::Parameter& param)
: Modulator::Modulator(param.mod),
  _param(param)
{
}

ModulatorLut::~ModulatorLut() {
}

/** @brief Update the PWM value */
void ModulatorLut::calcMotorCommandPriv(
		const math::Vector3l& force_B,
		const math::Vector3l& torque_B,
		hw::pwm_t* command)
{
	uint8_t idx;
	int32_t commandLong[CNF_NB_MOTORS];
	uint8_t signIdxTrq[3];
	uint8_t signIdxFrc[3];
	int32_t maxScaled = (MODULATOR_MOTOR_MAX << SCALE_TORSOR);
	int32_t scale = 0;
	math::Vector3l forceRemaining_B(0,0,0);

	/* 1) Select the LUT idx w.r.t. sign of the demanded effort */
	for (idx=0 ; idx<3 ; idx++)
	{
		if (torque_B[(const uint8_t)idx] < 0)
			signIdxTrq[idx]=0;
		else
			signIdxTrq[idx]=1;
	}

	/* 2) Compute the requested motor square of PWM ratio using pseudo inverse */
	for (idx=0 ; idx<CNF_NB_MOTORS ; idx++)
	{
		commandLong[idx]  = math_abs(torque_B.x) * _param.lutTrq[signIdxTrq[0]][0][idx];
		commandLong[idx] += math_abs(torque_B.y) * _param.lutTrq[signIdxTrq[1]][1][idx];
		commandLong[idx] += math_abs(torque_B.z) * _param.lutTrq[signIdxTrq[2]][2][idx];

		int32_t scaledCmd = (commandLong[idx] >> SCALE_TORSOR);

		forceRemaining_B.x += Modulator::_param.inflMat[idx][0] * scaledCmd;
		forceRemaining_B.y += Modulator::_param.inflMat[idx][1] * scaledCmd;
		forceRemaining_B.z += Modulator::_param.inflMat[idx][2] * scaledCmd;
	}

	forceRemaining_B.x = force_B.x - (forceRemaining_B.x >> SCALE_INFLUENCE_MATRIX);
	forceRemaining_B.y = force_B.y - (forceRemaining_B.y >> SCALE_INFLUENCE_MATRIX);
	forceRemaining_B.z = force_B.z - (forceRemaining_B.z >> SCALE_INFLUENCE_MATRIX);

	for (idx=0 ; idx<3 ; idx++)
	{
		if (forceRemaining_B[idx] < 0)
			signIdxFrc[idx]=0;
		else
			signIdxFrc[idx]=1;
	}

	/* Compute the contribution for the force, taking into account parasitic force */
	for (idx=0 ; idx<CNF_NB_MOTORS ; idx++)
	{
		commandLong[idx] +=
			 math_abs(forceRemaining_B.x) * _param.lutFrc[signIdxFrc[0]][0][idx]
		   + math_abs(forceRemaining_B.y) * _param.lutFrc[signIdxFrc[1]][1][idx]
		   + math_abs(forceRemaining_B.z) * _param.lutFrc[signIdxFrc[2]][2][idx];

		// Simple security!
		commandLong[idx] = math_max(0, commandLong[idx]);

		// Check saturation
		if (commandLong[idx] > maxScaled)
		{
			// saturation AND scale
			scale = math_max(scale, commandLong[idx]/MODULATOR_MOTOR_MAX);
		}
	}

	if (scale != 0)
	{
		// Saturation, thus scale such that to have MODULATOR_MOTOR_MAX
		for (idx=0 ; idx<CNF_NB_MOTORS ; idx++)
		{
			command[idx] = ((uint16_t) math_min(MODULATOR_MOTOR_MAX, commandLong[idx] / scale)) + Modulator::_param.minPwm[idx];
		}
	}
	else
	{
		// no saturation, thus scale ONLY
		for (idx=0 ; idx<CNF_NB_MOTORS ; idx++)
		{
			command[idx] = ((uint16_t) math_min(MODULATOR_MOTOR_MAX, commandLong[idx] >> SCALE_TORSOR)) + Modulator::_param.minPwm[idx];
		}
	}
}

} /* namespace autom */
