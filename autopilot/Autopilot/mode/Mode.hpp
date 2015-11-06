/*
 * Mode.hpp
 *
 *  Created on: 17 juin 2015
 *      Author: AdministrateurLocal
 */

#ifndef GCS_MODE_MODE_HPP_
#define GCS_MODE_MODE_HPP_

#include <system/system/System.hpp>
#include <att/guid/AttitudeGuidance.hpp>
#include <att/ctrl/AttitudeController.hpp>
#include <nav/guid/NavigationGuidance.hpp>
#include <nav/ctrl/NavigationController.hpp>

namespace system {

class Mode {
public:
	typedef enum
	{
		E_STEP_NONE,
		E_STEP_ATTITUDE,
		E_STEP_NAVIGATION,
	} E_STEP;
public:
	Mode();
	virtual ~Mode();

	/** Activated on entering the mode by mode manager */
	virtual void onEnter();

	/** Execute current step */
	virtual void execute(E_STEP step);

	/** Activated on leaving the mode by mode manager state */
	virtual void onLeave();

protected:


protected:
	/** @bief Attitude controller: (intended to be) common to all modes */
	static attitude::AttitudeController _attCtrl;

	/** @bief Attitude guidance state machine: (intended to be) common to all modes */
	static attitude::AttitudeGuidanceManager _attGuidStateMachine;

	/** @bief Navigation controller: (intended to be) common to all modes */
	static navigation::NavigationController _navCtrl;

	/** @bief Navigation guidance state machine: (intended to be) common to all modes */
	static navigation::NavigationGuidanceManager _navGuidStateMachine;
};

} /* namespace system */

#endif /* GCS_MODE_MODE_HPP_ */
