/*
 * ModeIdle.hpp
 *
 *  Created on: 6 nov. 2015
 *      Author: AdministrateurLocal
 */

#ifndef MODE_MODEIDLE_HPP_
#define MODE_MODEIDLE_HPP_

#include <mode/control/ModeControl.hpp>

namespace system {

class ModeIdle : public ModeControl {
public:
	ModeIdle();
	virtual ~ModeIdle();

	/** Execute current step */
	virtual void execute(ModeControl::Step step);

	/** Activated on entering the mode by mode manager */
	virtual void onEnter();

};

} /* namespace system */

#endif /* MODE_MODEIDLE_HPP_ */
