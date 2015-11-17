/*
 * ModeControl.hpp
 *
 *  Created on: 8 nov. 2015
 *      Author: AdministrateurLocal
 */

#ifndef MODE_CONTROL_MODECONTROL_HPP_
#define MODE_CONTROL_MODECONTROL_HPP_

#include <infra/mode/Mode.hpp>
#include <autom/mod/ModulatorLut.hpp>
#include <att/guid/AttitudeGuidanceManager.hpp>
#include <att/ctrl/AttitudeController.hpp>
#include <nav/guid/NavigationGuidanceManager.hpp>
#include <nav/ctrl/NavigationController.hpp>

namespace system {

class ModeControl : public infra::Mode {
public:
	typedef enum
	{
		E_STEP_NONE,
		E_STEP_ATTITUDE,
		E_STEP_NAVIGATION,
	} Step;

	typedef enum
	{
		E_MODE_IDLE,
		E_MODE_AUTOSTAB
	} Mode ;

public:

	/** @brief Initialize mode manager */
	static void initialize();

	/** @brief Set new mode */
	static void setMode(Mode mode);

	/** @brief Execute current step */
	static void execute();

protected:

	/** @brief Reset step */
	static void resetStep();

	/** @brief Get current step */
	static Step getStep();

	/** @brief Update current step */
	static void updateStep();

protected:

	/** @brief Current mode identifier */
	static Mode _currentModeIdentifier;

	/** @brief Current mode */
	static ModeControl* _currentMode;

	/** @brief Current step */
	static uint8_t _currentStep;


public:
	ModeControl();
	virtual ~ModeControl();

	/** Activated on entering the mode by mode manager */
	virtual void onEnter();

	/** Execute current step */
	virtual void execute(Step step);

	/** @brief Verify if transition to this mode is possible */
	virtual bool isReady();

	/** Activated on leaving the mode by mode manager state */
	virtual void onLeave();

protected:

	/** @brief Execute attitude step */
	virtual void stepAttitude();

	/** @brief Execute navigation step */
	virtual void stepNavigation();

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
