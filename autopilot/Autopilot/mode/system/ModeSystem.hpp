/*
 * ModeSystem.hpp
 *
 *  Created on: 8 nov. 2015
 *      Author: AdministrateurLocal
 */

#ifndef MODE_SYSTEM_MODESYSTEM_HPP_
#define MODE_SYSTEM_MODESYSTEM_HPP_

#include <mode/Mode.hpp>

namespace system {

class ModeSystem : public Mode {
public:
	ModeSystem();
	virtual ~ModeSystem();

	/** Activated on entering the mode by mode manager */
	virtual void onEnter() = 0;

	/** Activated on leaving the mode by mode manager state */
	virtual void onLeave() = 0;
};

} /* namespace system */

#endif /* MODE_SYSTEM_MODESYSTEM_HPP_ */
