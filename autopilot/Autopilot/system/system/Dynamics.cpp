/*
 * Dynamics.cpp
 *
 *  Created on: 3 oct. 2015
 *      Author: AdministrateurLocal
 */

#include <system/system/Dynamics.hpp>
#include <system/system/SystemData.hpp>

namespace system {

Dynamics::Dynamics()
{
}

Dynamics::~Dynamics()
{
}

/** @brief Get diagonal inertia */
void Dynamics::getDiagInertia(math::Vector3f& inertia)
{
	inertia(
			paramDyn.diagInertia[0],
			paramDyn.diagInertia[1],
			paramDyn.diagInertia[2]);
}

/** @brief Get Center of Mass position in body frame */
void Dynamics::getCenterOfMass(math::Vector3f& position_B)
{
	position_B(
			paramDyn.comPositionB[0],
			paramDyn.comPositionB[1],
			paramDyn.comPositionB[2]);
}

/** @brief Get Center of Mass position in body frame */
void Dynamics::getMass(float& mass)
{
	mass = paramDyn.mass;
}


} /* namespace system */
