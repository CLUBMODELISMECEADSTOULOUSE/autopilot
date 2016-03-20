/*
 * ModeControlData.hpp
 *
 *  Created on: 8 nov. 2015
 *      Author: AdministrateurLocal
 */

#ifndef MODE_CONTROL_MODECONTROLDATA_HPP_
#define MODE_CONTROL_MODECONTROLDATA_HPP_

#include "ModeControlMgr.hpp"

namespace system {

/** @brief Attitude controller parameter */
extern const attitude::AttitudeController::Parameter paramAttCtrl;

/** @brief Navigation controller parameter */
extern const navigation::NavigationController::Parameter paramNavCtrl;

/** @brief Modulator setting */
extern const autom::ModulatorLut::Parameter paramMod;

} /* namespace system */

#endif /* MODE_CONTROL_MODECONTROLDATA_HPP_ */
