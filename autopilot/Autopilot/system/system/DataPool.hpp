/*
 * DataPool.hpp
 *
 *  Created on: 16 juin 2015
 *      Author: AdministrateurLocal
 */

#ifndef SYSTEM_SYSTEM_DATAPOOL_HPP_
#define SYSTEM_SYSTEM_DATAPOOL_HPP_

#include <math/Quaternion.hpp>
#include <hw/pwm/Pwm.hpp>

#include <system/params/Nrd.hpp>

namespace system {

class DataPool {
public:
	DataPool();
	virtual ~DataPool();

	math::Vector3i imuRateRaw_U;
	math::Vector3i imuAccRaw_U;
	math::Vector3i imuRateRawBias_U;
	math::Vector3f imuRate_B;
	math::Vector3f imuAcc_B;
	int16_t imuTemp;
	uint32_t imuLastMeasDateUsec;

	math::Vector3i compassMagRaw_U;
	math::Vector3f compassMag_B;
	uint32_t compassLastMeasDateUsec;

	int32_t baroPressRaw_U;
	int16_t baroTempRaw_U;
	float baroPress_B;
	uint32_t baroLastMeasDateUsec;

	bool estAttValid;
	math::Quaternion estAtt_IB;
	math::Matrix3f estDcm_IB;
	math::Vector3f estRate_B;
	bool estPosValid;
	math::Vector3f estPos_I;
	math::Vector3f estVel_I;

	math::Quaternion guidAtt_IB;
	math::Matrix3f guidDcm_IB;
	math::Vector3f guidRate_B;
	math::Vector3f guidPos_I;
	math::Vector3f guidVel_I;

	hw::pwm_t pwm_inputs[CNF_PWM_NUM_FROM_DEVICE];
	hw::pwm_t pwm_outputs[CNF_PWM_NUM_TO_DEVICE];

	math::Vector3f ctrlAttAngErrB;
	math::Vector3f ctrlAttRateErrB;
	math::Vector3f ctrlNavPosErrI;
	math::Vector3f ctrlNavVelErrI;
	math::Vector3f ctrlFrcDemI;

	math::Vector3l ctrlFrcDemB;
	math::Vector3l ctrlTrqDemB;

	math::Vector3l estForce_B;
	math::Vector3l estTorque_B;

};

} /* namespace system */

#endif /* SYSTEM_SYSTEM_DATAPOOL_HPP_ */
