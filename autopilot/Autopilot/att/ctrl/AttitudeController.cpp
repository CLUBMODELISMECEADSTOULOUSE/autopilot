/*
 * AttitudeController.cpp
 *
 *  Created on: 19 ao�t 2015
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
  _filterZ(param.filterZ)
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
//	const math::Quaternion qDem_IB(1.,0.,0.,0.);
//	const math::Vector3f rateDem_B(0.,0.,0.);

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
		_ctrl.computeCommand(
				system::system.dataPool.ctrlAttAngErrB,
				system::system.dataPool.ctrlAttRateErrB,
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

//		system::system.dataPool.ctrlTrqDemB.z = (int32_t) (1024. * sinf(pulse*time));
//		time += 0.01;
//		system::system.dataPool.ctrlTrqDemB(
//				(int32_t) ldexpf(system::system.dataPool.ctrlAttAngErrB.x, 12),
//				(int32_t) ldexpf(system::system.dataPool.ctrlAttAngErrB.y, 12),
//				(int32_t) ldexpf(system::system.dataPool.ctrlAttAngErrB.z, 12));
	}

//	math::Vector3f dynInertia;
//	system::system.getDynamics().getDiagInertia(dynInertia);
//	math::Vector3l trqDemGuidB(
//			((int32_t) ldexpf(((rateDem_B.x-_rateDem_B_prev.x)*FSW_TASK_CTRL_FREQ*dynInertia.x),SCALE_TORSOR)),
//			((int32_t) ldexpf(((rateDem_B.y-_rateDem_B_prev.y)*FSW_TASK_CTRL_FREQ*dynInertia.y),SCALE_TORSOR)),
//			((int32_t) ldexpf(((rateDem_B.z-_rateDem_B_prev.z)*FSW_TASK_CTRL_FREQ*dynInertia.z),SCALE_TORSOR)));
//
//	system::system.dataPool.ctrlTrqDemB += trqDemGuidB;
}

void AttitudeController::reset()
{
	_ctrl.initialize();
	_filterX.reset(0.);
	_filterY.reset(0.);
	_filterZ.reset(0.);
}

} /* namespace attitude */
