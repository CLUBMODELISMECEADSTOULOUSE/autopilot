/*
 * AttitudeController.cpp
 *
 *  Created on: 19 août 2015
 *      Author: AdministrateurLocal
 */

#include <system/system/System.hpp>
#include "AttitudeController.hpp"

namespace attitude {

AttitudeController::AttitudeController(const AttitudeController::Parameter& param)
: _ctrl(param.ctrl),
  _rateDem_B_prev(0.,0.,0.),
  _param(param),
  _filterX(param.filterX),
  _filterY(param.filterY),
  _filterZ(param.filterZ)//,
{
}

AttitudeController::~AttitudeController() {
}

/** @brief Compute torque */
void AttitudeController::execute()
{
	const math::Quaternion& qEst_IB = system::system.dataPool.estAtt_IB;
	const math::Vector3f& rateEst_B = system::system.dataPool.estRate_B;
	const math::Quaternion& qDem_IB = system::system.dataPool.guidAtt_IB;
	const math::Vector3f& rateDem_B = system::system.dataPool.guidRate_B;

	/* Attitude error computed as the conj(qEst)*qDem */
	{
		math::Quaternion dQ = (~qEst_IB) * qDem_IB;
		if (dQ.scalar < 0)
		{
			system::system.dataPool.ctrlAttAngErrB(
					-ldexpf(dQ.vector.x,1),
					-ldexpf(dQ.vector.y,1),
					-ldexpf(dQ.vector.z,1));
		}
		else
		{
			system::system.dataPool.ctrlAttAngErrB(
					ldexpf(dQ.vector.x,1),
					ldexpf(dQ.vector.y,1),
					ldexpf(dQ.vector.z,1));
		}
	}

	/* Set angular control error */
	system::system.dataPool.ctrlAttRateErrB = rateDem_B - rateEst_B;

	{
		/* Compute controller */
		math::Vector3f ctrlTrqDemB;
		math::Vector3f nullVect;
		_ctrl.computeCommand(
				system::system.dataPool.ctrlAttAngErrB,
				system::system.dataPool.ctrlAttRateErrB,
				nullVect,
				ctrlTrqDemB);

		/* filter the control torque */
		ctrlTrqDemB(
				_filterX.apply(ctrlTrqDemB.x),
				_filterY.apply(ctrlTrqDemB.y),
				_filterZ.apply(ctrlTrqDemB.z));

		/* Set the commanded torque */
		system::system.dataPool.ctrlTrqDemB(
				(int32_t) ldexpf(ctrlTrqDemB.x, SCALE_TORSOR),
				(int32_t) ldexpf(ctrlTrqDemB.y, SCALE_TORSOR),
				(int32_t) ldexpf(ctrlTrqDemB.z, SCALE_TORSOR));

	}
}

void AttitudeController::reset()
{
	_ctrl.initialize();
	_filterX.reset(0.);
	_filterY.reset(0.);
	_filterZ.reset(0.);
}

} /* namespace attitude */
