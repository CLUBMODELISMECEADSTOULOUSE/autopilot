/*
 * NavigationController.cpp
 *
 *  Created on: 8 sept. 2015
 *      Author: AdministrateurLocal
 */

#include <nav/ctrl/NavigationController.hpp>
#include <system/params/Nrd.hpp>
#include <system/system/System.hpp>

namespace navigation {

NavigationController::NavigationController(const Parameter& param)
: _param(param),
  _ctrl(param.paramCtrl)
{
}

NavigationController::~NavigationController() {
	// TODO Auto-generated destructor stub
}

/** @brief Exeute the service */
void NavigationController::execute()
{
	/* Compute control errors */
	system::system.dataPool.ctrlNavPosErrI =
			system::system.dataPool.guidPos_I - system::system.dataPool.estPos_I;
	system::system.dataPool.ctrlNavVelErrI =
			system::system.dataPool.guidVel_I - system::system.dataPool.estVel_I;


	/* Compute error performed on the command realization */
	math::Vector3f errCmdB(
			ldexpf((float)(system::system.dataPool.ctrlFrcDemB.x - system::system.dataPool.estForce_B.x), -SCALE_TORSOR),
			ldexpf((float)(system::system.dataPool.ctrlFrcDemB.y - system::system.dataPool.estForce_B.y), -SCALE_TORSOR),
			ldexpf((float)(system::system.dataPool.ctrlFrcDemB.z - system::system.dataPool.estForce_B.z), -SCALE_TORSOR));
	math::Vector3f errCmdI = system::system.dataPool.estAtt_IB.rotateQVQconj(errCmdB);

	/* Compute control force */
	_ctrl.computeCommand(
			system::system.dataPool.ctrlNavPosErrI,
			system::system.dataPool.ctrlNavVelErrI,
			errCmdI,
			system::system.dataPool.ctrlFrcDemI);

	/* Add feedforward guidance force to the force */
	system::system.dataPool.ctrlFrcDemI += (system::system.dataPool.guidVel_I * _param.mass);

	/* Add feedforward gravity force to the force */
	system::system.dataPool.ctrlFrcDemI += (math::Vector3f(0.,0.,-PHYSICS_GRAVITY) * _param.mass);
}

/** @brief Reset controller */
void NavigationController::reset()
{
	/* Reinitialize the controller */
	_ctrl.initialize();
}



} /* namespace navigation */
