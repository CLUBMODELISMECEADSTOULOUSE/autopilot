/*
 * Modulator.hpp
 *
 *  Created on: 24 déc. 2013
 *      Author: Robotique
 */

#ifndef MODULATOR_HPP_
#define MODULATOR_HPP_

#include <hw/pwm/Pwm.hpp>

#include <system/params/Nrd.hpp>

#include <math/Vector3.hpp>


#define MODULATOR_MOTOR_MAX ((int32_t) (MAX_PULSEWIDTH - MIN_PULSEWIDTH))
#define MODULATOR_AXIS_X (0)
#define MODULATOR_AXIS_Y (1)
#define MODULATOR_AXIS_Z (2)
#define MODULATOR_AXIS_NB (3)


namespace autom {

class Modulator {
public:
	typedef struct {
		int32_t inflMat[CNF_NB_MOTORS][6];
		int32_t frcMaxPos_B[MODULATOR_AXIS_NB];
		int32_t frcMaxNeg_B[MODULATOR_AXIS_NB];
		int32_t trqMaxPos_B[MODULATOR_AXIS_NB];
		int32_t trqMaxNeg_B[MODULATOR_AXIS_NB];
		uint16_t minPwm[CNF_NB_MOTORS];
	} Parameter;

public:

	Modulator(const Parameter& param);
	virtual ~Modulator();

	/** @brief Init the process */
	virtual void initialize();

	/** @brief Execute the process */
	virtual void calcMotorCommand(
			const math::Vector3l& force_B,
			const math::Vector3l& torque_B,
			hw::pwm_t* command) ;

	/** @brief Execute the process */
	virtual void calcTorsor(
			const hw::pwm_t* command,
			math::Vector3l& force_B,
			math::Vector3l& torque_B) ;

protected:

	/** @brief Calc motor command*/
	virtual void calcMotorCommandPriv(
			const math::Vector3l& force_B,
			const math::Vector3l& torque_B,
			hw::pwm_t command[CNF_NB_MOTORS]) = 0 ;

	/** @brief Parameter */
	const Parameter& _param;
};

} /* namespace autom */

#endif /* MODULATOR_HPP_ */
