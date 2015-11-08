/*
 * NavigationController.hpp
 *
 *  Created on: 8 sept. 2015
 *      Author: AdministrateurLocal
 */

#ifndef NAVIGATION_CTRL_NAVIGATIONCONTROLLER_HPP_
#define NAVIGATION_CTRL_NAVIGATIONCONTROLLER_HPP_

#include <autom/ctrl/ControllerPid3Axes.hpp>

namespace navigation {

class NavigationController {
public:
	typedef struct {
		autom::ControllerPid3Axes<float>::Parameter paramCtrl;
		float mass;
	} Parameter ;

public:
	NavigationController(const Parameter& param);
	virtual ~NavigationController();

	/** @brief Exeute the service */
	void execute();

	/** @brief Reset controller */
	void reset();

protected:

	/** @brief Parameter */
	const Parameter& _param;

	/** @brief Controller */
	autom::ControllerPid3Axes<float> _ctrl;

};

} /* namespace navigation */

#endif /* NAVIGATION_CTRL_NAVIGATIONCONTROLLER_HPP_ */
