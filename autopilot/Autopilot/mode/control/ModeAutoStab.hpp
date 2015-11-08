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
};

} /* namespace system */

#endif /* MODEAUTOSTAB_HPP_ */
