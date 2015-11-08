/*
 * Dynamics.hpp
 *
 *  Created on: 3 oct. 2015
 *      Author: AdministrateurLocal
 */

#ifndef SYSTEM_PARAMS_DYNAMICS_HPP_
#define SYSTEM_PARAMS_DYNAMICS_HPP_

#include <math/Vector3.hpp>

namespace system {

class Dynamics {
public:
	/** @brief Parameter type */
	typedef struct
	{
		/** @brief Diagonal inertias */
		float diagInertia[3];

		/** @brief Center of mass position in body frame */
		float comPositionB[3];

		/** @brief Mass */
		float mass;

	} Parameter ;

public:
	Dynamics();
	virtual ~Dynamics();

public:
	/** @brief Get diagonal inertia */
	static void getDiagInertia(math::Vector3f& inertia) ;

	/** @brief Get Center of Mass position in body frame */
	static void getCenterOfMass(math::Vector3f& position_B);

	/** @brief Get mass */
	static void getMass(float& mass);

};

} /* namespace system */

#endif /* SYSTEM_PARAMS_DYNAMICS_HPP_ */
