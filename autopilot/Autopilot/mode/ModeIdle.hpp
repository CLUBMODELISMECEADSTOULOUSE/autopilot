/*
 * ModeIdle.hpp
 *
 *  Created on: 6 nov. 2015
 *      Author: AdministrateurLocal
 */

#ifndef MODE_MODEIDLE_HPP_
#define MODE_MODEIDLE_HPP_

#include <mode/Mode.hpp>

namespace system {

class ModeIdle: public Mode {
public:
	ModeIdle();
	virtual ~ModeIdle();
};

} /* namespace system */

#endif /* MODE_MODEIDLE_HPP_ */
