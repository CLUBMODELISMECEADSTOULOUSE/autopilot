/*
 * ModeAutoStab.hpp
 *
 *  Created on: 6 nov. 2015
 *      Author: AdministrateurLocal
 */

#ifndef MODEAUTOSTAB_HPP_
#define MODEAUTOSTAB_HPP_

#include <mode/control/ModeControl.hpp>

namespace system {

class ModeAutoStab : public ModeControl {
public:
	ModeAutoStab();
	virtual ~ModeAutoStab();

	/** Activated on entering the mode by mode manager */
	virtual void onEnter();

protected:

	/** @brief Execute navigation step */
	virtual void stepNavigation();

};

} /* namespace system */

#endif /* MODEAUTOSTAB_HPP_ */
