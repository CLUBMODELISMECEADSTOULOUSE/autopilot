/*
 * SystemData.hpp
 *
 *  Created on: 8 nov. 2015
 *      Author: AdministrateurLocal
 */

#ifndef SYSTEM_SYSTEM_SYSTEMDATA_HPP_
#define SYSTEM_SYSTEM_SYSTEMDATA_HPP_

#include <hw/radio/Radio.hpp>
#include <system/system/Dynamics.hpp>
#include <autom/est/SimpleAttitudeKalmanFilter.hpp>

namespace system {

/** @brief Radio parameters */
extern const hw::Radio::Parameter paramRadio;

/** @brief Dynamics parameters */
extern const system::Dynamics::Parameter paramDyn;

/** @brief Estimator parameter */
extern const autom::SimpleAttitudeKalmanFilter::Parameter paramEst;

} /* namespace system */


#endif /* SYSTEM_SYSTEM_SYSTEMDATA_HPP_ */
