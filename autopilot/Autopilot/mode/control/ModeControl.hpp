/*
 * ModeControl.hpp
 *
 *  Created on: 8 nov. 2015
 *      Author: AdministrateurLocal
 */

#ifndef MODE_CONTROL_MODECONTROL_HPP_
#define MODE_CONTROL_MODECONTROL_HPP_

#include <mode/Mode.hpp>
#include <system/system/System.hpp>
#include <att/guid/AttitudeGuidance.hpp>
#include <att/ctrl/AttitudeController.hpp>
#include <nav/guid/NavigationGuidance.hpp>
#include <nav/ctrl/NavigationController.hpp>

namespace system {

class ModeControl: public Mode {
public:
	typedef enum
	{
		E_STEP_NONE,
		E_STEP_ATTITUDE,
		E_STEP_NAVIGATION,
	} E_STEP;

public:
	ModeControl();
	virtual ~ModeControl();

	/** Activated on entering the mode by mode manager */
	virtual void onEnter();

	/** Execute current step */
	virtual void execute(E_STEP step);

	/** Activated on leaving the mode by mode manager state */
	virtual void onLeave();

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

#endif /* MODE_CONTROL_MODECONTROL_HPP_ */
