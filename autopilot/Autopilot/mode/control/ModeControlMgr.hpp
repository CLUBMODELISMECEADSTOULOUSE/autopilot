/*
 * ModeControlMgr.hpp
 *
 *  Created on: 20 mars 2016
 *      Author: AdministrateurLocal
 */

#ifndef MODE_CONTROL_MGR_HPP_
#define MODE_CONTROL_MGR_HPP_

#include <att/guid/AttitudeGuidanceMgr.hpp>
#include <autom/mod/ModulatorLut.hpp>
#include <att/ctrl/AttitudeController.hpp>
#include <nav/guid/NavigationGuidanceManager.hpp>
#include <nav/ctrl/NavigationController.hpp>

#include "ModeIdle.hpp"
#include "ModeAutoStab.hpp"

namespace system {

class ModeControlMgr {
public:

	/** @brief Mode */
	typedef enum
	{
		E_MODE_IDLE,
		E_MODE_AUTOSTAB
	} Mode ;

public:
	ModeControlMgr();
	virtual ~ModeControlMgr();

	/** @brief Initialize mode manager */
	void initialize();

	/** @brief Set new mode */
	void setMode(Mode mode);

	/** @brief Execute current step */
	void execute();

	/** @brief Get attitude guidance manager */
	inline attitude::AttitudeGuidanceMgr& getAttGuidMgr();

	/** @brief Get attitude guidance manager */
	inline navigation::NavigationGuidanceManager& getNavGuidMgr();

	/** @brief Get attitude controller */
	inline attitude::AttitudeController& getAttCtrl();

	/** @brief Get navigation controller */
	inline navigation::NavigationController& getNavCtrl();

protected:

	/** @brief Reset step */
	void resetStep();

	/** @brief Get current step */
	ModeControl::Step getStep();

	/** @brief Update current step */
	void updateStep();

protected:

	/** @brief Navigation guidance state machine: (intended to be) common to all modes */
	autom::ModulatorLut _modulator;

	/** @brief Attitude controller: (intended to be) common to all modes */
	attitude::AttitudeController _attCtrl;

	/** @brief Navigation controller: (intended to be) common to all modes */
	navigation::NavigationController _navCtrl;

	/** @brief Attitude guidance manager */
	attitude::AttitudeGuidanceMgr _attGuidMgr;

	/** @brief Navigation guidance state machine: (intended to be) common to all modes */
	navigation::NavigationGuidanceManager _navGuidMgr;

	/** @brief Mode IDLE */
	ModeIdle _modeIdle;

	/** @brief Mode Auto Stabilized */
	ModeAutoStab _modeAutoStab;

	/** @brief Current mode identifier */
	Mode _currentModeIdentifier;

	/** @brief Current mode */
	ModeControl* _currentMode;

	/** @brief Current step */
	uint8_t _currentStep;

};

/** @brief Get attitude guidance manager */
attitude::AttitudeGuidanceMgr& ModeControlMgr::getAttGuidMgr()
{
	return _attGuidMgr;
}

/** @brief Get attitude guidance manager */
navigation::NavigationGuidanceManager& ModeControlMgr::getNavGuidMgr()
{
	return _navGuidMgr;
}

/** @brief Get attitude controller */
attitude::AttitudeController& ModeControlMgr::getAttCtrl()
{
	return _attCtrl;
}

/** @brief Get navigation controller */
navigation::NavigationController& ModeControlMgr::getNavCtrl()
{
	return _navCtrl;
}

} /* namespace system */

#endif /* MODE_CONTROL_MGR_HPP_ */
