/*
 * ModeControl.hpp
 *
 *  Created on: 8 nov. 2015
 *      Author: AdministrateurLocal
 */

#ifndef MODE_CONTROL_MODECONTROL_HPP_
#define MODE_CONTROL_MODECONTROL_HPP_

#include <mode/Mode.hpp>
#include <autom/mod/ModulatorLut.hpp>
#include <att/guid/AttitudeGuidanceManager.hpp>
#include <att/ctrl/AttitudeController.hpp>
#include <nav/guid/NavigationGuidanceManager.hpp>
#include <nav/ctrl/NavigationController.hpp>

namespace system {

class ModeControl : public Mode {
public:
	typedef enum
	{
		E_STEP_NONE,
		E_STEP_ATTITUDE,
		E_STEP_NAVIGATION,
	} Step;

public:
	ModeControl();
	virtual ~ModeControl();

	/** Activated on entering the mode by mode manager */
	virtual void onEnter();

	/** Execute current step */
	virtual void execute(Step step);

	/** Activated on leaving the mode by mode manager state */
	virtual void onLeave();

protected:
	/** @brief Attitude controller: (intended to be) common to all modes */
	static attitude::AttitudeController _attCtrl;

	/** @brief Attitude guidance state machine: (intended to be) common to all modes */
	static attitude::AttitudeGuidanceManager _attGuidStateMachine;

	/** @brief Navigation controller: (intended to be) common to all modes */
	static navigation::NavigationController _navCtrl;

	/** @brief Navigation guidance state machine: (intended to be) common to all modes */
	static navigation::NavigationGuidanceManager _navGuidStateMachine;

	/** @brief Navigation guidance state machine: (intended to be) common to all modes */
	static autom::ModulatorLut _modulator;
};

} /* namespace system */

#endif /* MODE_CONTROL_MODECONTROL_HPP_ */
