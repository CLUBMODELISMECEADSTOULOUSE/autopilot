/*
 * Mode.hpp
 *
 *  Created on: 17 juin 2015
 *      Author: AdministrateurLocal
 */

#ifndef GCS_MODE_MODE_HPP_
#define GCS_MODE_MODE_HPP_

namespace infra {

class Mode {
public:
	Mode();
	virtual ~Mode();

	/** @brief Activated on entering the mode by mode manager */
	virtual void onEnter() = 0;

	/** @brief Activated on leaving the mode by mode manager state */
	virtual void onLeave() = 0;
};

} /* namespace infra */

#endif /* GCS_MODE_MODE_HPP_ */
