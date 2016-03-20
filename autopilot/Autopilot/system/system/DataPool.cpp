/*
 * DataPool.cpp
 *
 *  Created on: 16 juin 2015
 *      Author: AdministrateurLocal
 */

#include <system/system/DataPool.hpp>

namespace system {

DataPool::DataPool()
: compassIsMeasAvail(false),
  imuIsMeasAvail(false),
  baroIsMeasAvail(false),
  gpsIsMeasAvail(false)
{
	for (uint8_t idx = 0 ; idx<CNF_PWM_NUM_TO_DEVICE ; idx++)
	{
		pwm_outputs[idx] = MIN_PULSEWIDTH;
	}

	for (uint8_t idx = 0 ; idx<CNF_PWM_NUM_FROM_DEVICE ; idx++)
	{
		pwm_inputs[idx] = MIN_PULSEWIDTH;
	}
}

DataPool::~DataPool() {
	// TODO Auto-generated destructor stub
}

} /* namespace system */
