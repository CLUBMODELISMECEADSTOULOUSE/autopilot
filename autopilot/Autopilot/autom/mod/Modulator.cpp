/*
 * Modulator.cpp
 *
 *  Created on: 26 juin 2015
 *      Author: AdministrateurLocal
 */

#include "Modulator.hpp"

namespace autom {

Modulator::Modulator(const Modulator::Parameter& param)
: _param(param)
{
}

Modulator::~Modulator() {
}


/** @brief Init the process */
void Modulator::initialize()
{
}

/** @brief Init the process */
void Modulator::calcMotorCommand(
		const math::Vector3l& force_B,
		const math::Vector3l& torque_B,
		hw::pwm_t* command)
{
	math::Vector3l forceSaturated_B(
			math_min(math_max(force_B.x, _param.frcMaxNeg_B[MODULATOR_AXIS_X]), _param.frcMaxPos_B[MODULATOR_AXIS_X]),
			math_min(math_max(force_B.y, _param.frcMaxNeg_B[MODULATOR_AXIS_Y]), _param.frcMaxPos_B[MODULATOR_AXIS_Y]),
			math_min(math_max(force_B.z, _param.frcMaxNeg_B[MODULATOR_AXIS_Z]), _param.frcMaxPos_B[MODULATOR_AXIS_Z])
			);
	math::Vector3l torqueSaturated_B(
			math_min(math_max(torque_B.x, _param.trqMaxNeg_B[MODULATOR_AXIS_X]), _param.trqMaxPos_B[MODULATOR_AXIS_X]),
			math_min(math_max(torque_B.y, _param.trqMaxNeg_B[MODULATOR_AXIS_Y]), _param.trqMaxPos_B[MODULATOR_AXIS_Y]),
			math_min(math_max(torque_B.z, _param.trqMaxNeg_B[MODULATOR_AXIS_Z]), _param.trqMaxPos_B[MODULATOR_AXIS_Z])
			);

	/* Compute PWM */
	calcMotorCommandPriv(
				forceSaturated_B,
				torqueSaturated_B,
				command);
}
/** @brief Update the produced torsor */
void Modulator::calcTorsor(
		const hw::pwm_t* command,
		math::Vector3l& force_B,
		math::Vector3l& torque_B)
{
	uint8_t iMotor;

	/* Initialize to zero */
	force_B(0,0,0);
	torque_B(0,0,0);

	/* Compute the produced torsor */
	for (iMotor=0 ; iMotor<CNF_NB_MOTORS ; iMotor++)
	{
		force_B.x += _param.inflMat[iMotor][0] * ((int32_t)(command[iMotor]-_param.minPwm[iMotor]));
		force_B.y += _param.inflMat[iMotor][1] * ((int32_t)(command[iMotor]-_param.minPwm[iMotor]));
		force_B.z += _param.inflMat[iMotor][2] * ((int32_t)(command[iMotor]-_param.minPwm[iMotor]));
		torque_B.x += _param.inflMat[iMotor][3] * ((int32_t)(command[iMotor]-_param.minPwm[iMotor]));
		torque_B.y += _param.inflMat[iMotor][4] * ((int32_t)(command[iMotor]-_param.minPwm[iMotor]));
		torque_B.z += _param.inflMat[iMotor][5] * ((int32_t)(command[iMotor]-_param.minPwm[iMotor]));
	}

	force_B(
			force_B.x >> SCALE_INFLUENCE_MATRIX,
			force_B.y >> SCALE_INFLUENCE_MATRIX,
			force_B.z >> SCALE_INFLUENCE_MATRIX);
	torque_B(
			torque_B.x >> SCALE_INFLUENCE_MATRIX,
			torque_B.y >> SCALE_INFLUENCE_MATRIX,
			torque_B.z >> SCALE_INFLUENCE_MATRIX);
}

} /* namespace autom */
