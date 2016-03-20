/*
 * AttitudeGuidanceAutoStab.hpp
 *
 *  Created on: 27 août 2015
 *      Author: AdministrateurLocal
 */

#ifndef ATT_GUID_ATTITUDEGUIDANCEAUTOSTAB_HPP_
#define ATT_GUID_ATTITUDEGUIDANCEAUTOSTAB_HPP_

#include "AttitudeGuidanceMode.hpp"

namespace attitude {

class AttitudeGuidanceAutoStab : public AttitudeGuidanceMode {
public:
	AttitudeGuidanceAutoStab();
	virtual ~AttitudeGuidanceAutoStab();

	/** @brief Set update yaw */
	inline void setYawUpdate(bool updateYaw) ;

protected:
	/** @brief Activated on leaving the mode by mode manager state */
	virtual void onLeave() ;

	/** @brief Activated on entering the mode by mode manager */
	virtual void onEnter() ;

	/** @brief Execute current step */
	virtual void execute() ;

	/** @brief Initialize internal state */
	virtual void initialize(
			const math::Quaternion& quat_IB,
			const math::Vector3f& rate_B) ;

protected:
	/** @brief Scale parameter in Autostab mode */
	float _paramScalePwm[3];

	/** @brief Prev Roll angle */
	float _angRollPrev;

	/** @brief Prev Pitch angle */
	float _angPitchPrev;

	/** @brief Prev Yaw angle */
	float _angYawPrev;

	/** @brief Norm */
	float _attNormInv;

	/** @brief Update yaw */
	bool _updateYaw;
};

/** @brief Set update yaw */
void AttitudeGuidanceAutoStab::setYawUpdate(bool updateYaw)
{
	_updateYaw = updateYaw;
}


} /* namespace attitude */

#endif /* ATT_GUID_ATTITUDEGUIDANCEAUTOSTAB_HPP_ */
