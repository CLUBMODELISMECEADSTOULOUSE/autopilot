/*
 * ModulatorLut.hpp
 *
 *  Created on: 24 déc. 2013
 *      Author: Robotique
 */

#ifndef MODULATORLUT_HPP_
#define MODULATORLUT_HPP_

#include <autom/mod/Modulator.hpp>


namespace autom {

class ModulatorLut : public Modulator {
public:
	typedef struct {
		Modulator::Parameter mod;
		int32_t lutTrq[2][3][CNF_NB_MOTORS];
		int32_t lutFrc[2][3][CNF_NB_MOTORS];
	} Parameter;

public:

	ModulatorLut(
			const ModulatorLut::Parameter& param);
	virtual ~ModulatorLut();

protected:
	/** @brief Calc motor command*/
	virtual void calcMotorCommandPriv(
			const math::Vector3l& force_B,
			const math::Vector3l& torque_B,
			hw::pwm_t* command) ;

protected:

	/** @brief Parameter */
	const Parameter& _param;

};

} /* namespace autom */

#endif /* MODULATORLUT_HPP_ */
