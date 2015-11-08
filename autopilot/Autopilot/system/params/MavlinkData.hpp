/*
 * MavlinkData.hpp
 *
 *  Created on: 8 nov. 2015
 *      Author: AdministrateurLocal
 */

#ifndef SYSTEM_PARAMS_MAVLINKDATA_HPP_
#define SYSTEM_PARAMS_MAVLINKDATA_HPP_

#include <avr/pgmspace.h>
#include <gcs/param/ParameterMgt.hpp>

namespace system {

extern PROGMEM const mavlink::ParameterMgt::ParamInfo info[];
extern uint16_t paramCount;

} /* namespace system */

#endif /* SYSTEM_PARAMS_MAVLINKDATA_HPP_ */
