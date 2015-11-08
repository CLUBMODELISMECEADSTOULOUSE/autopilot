/*
 * ModeInit.hpp
 *
 *  Created on: 8 nov. 2015
 *      Author: AdministrateurLocal
 */

#ifndef MODE_SYSTEM_MODEINIT_HPP_
#define MODE_SYSTEM_MODEINIT_HPP_

#include <mode/system/ModeSystem.hpp>

namespace system {

class ModeInit {
public:
	ModeInit();
	virtual ~ModeInit();

	/** Activated on entering the mode by mode manager */
	virtual void onEnter() = 0;

	/** Activated on leaving the mode by mode manager state */
	virtual void onLeave() = 0;

};

} /* namespace system */

#endif /* MODE_SYSTEM_MODEINIT_HPP_ */
