/*
 * ModeControl.hpp
 *
 *  Created on: 8 nov. 2015
 *      Author: AdministrateurLocal
 */

#ifndef MODE_CONTROL_MODECONTROL_HPP_
#define MODE_CONTROL_MODECONTROL_HPP_

#include <infra/mode/Mode.hpp>

namespace system {

class ModeControl : public infra::Mode {
public:
	/** @brief Execution step */
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

	/** @brief Execute attitude step */
	virtual void stepAttitude();

	/** @brief Execute navigation step */
	virtual void stepNavigation();

};

} /* namespace system */

#endif /* MODE_CONTROL_MODECONTROL_HPP_ */
