/*
 * AttitudeController.hpp
 *
 *  Created on: 19 août 2015
 *      Author: AdministrateurLocal
 */

#ifndef ATT_CTRL_ATTITUDECONTROLLER_HPP_
#define ATT_CTRL_ATTITUDECONTROLLER_HPP_

#include <math/Quaternion.hpp>
#include <autom/ctrl/ControllerPid3Axes.hpp>
#include <autom/filt/SecondOrderFilter.hpp>

namespace attitude {

class AttitudeController {
public:
	typedef struct
	{
		autom::ControllerPid3Axes<float>::Parameter ctrl;
		float maxCosAngOverTwoErr;
		float maxSinAngOverTwoErr;
		autom::SecondOrderFilter::Parameter filterX;
		autom::SecondOrderFilter::Parameter filterY;
		autom::SecondOrderFilter::Parameter filterZ;
	} Parameter;

public:
	AttitudeController(
			const Parameter& param);
	virtual ~AttitudeController();

	/** @brief */
	void reset();

	/** @brief Execute the service */
	void execute();

protected:

	/** @brief 3 Axes PID controller */
	autom::ControllerPid3Axes<float> _ctrl;

	/** @brief Previous rate */
	math::Vector3f _rateDem_B_prev;

	/** @brief Parameter */
	const AttitudeController::Parameter& _param;

	/** @brief Filter for X axis */
	autom::SecondOrderFilter _filterX;

	/** @brief Filter for Y axis */
	autom::SecondOrderFilter _filterY;

	/** @brief Filter for Z axis */
	autom::SecondOrderFilter _filterZ;
};

} /* namespace attitude */

#endif /* ATT_CTRL_ATTITUDECONTROLLER_HPP_ */
