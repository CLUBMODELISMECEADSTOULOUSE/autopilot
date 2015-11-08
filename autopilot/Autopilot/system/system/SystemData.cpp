/*
 * SystemData.cpp
 *
 *  Created on: 8 nov. 2015
 *      Author: AdministrateurLocal
 */

#include "SystemData.hpp"
#include <system/params/Nrd.hpp>
#include <system/params/NrdGen.hpp>

namespace system {

const hw::Radio::Parameter paramRadio =
{
		/* reversed */
		{0},
		/* pwmZero */
		{(MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1},
		/* pwmMin */
		{MIN_PULSEWIDTH, MIN_PULSEWIDTH, MIN_PULSEWIDTH, MIN_PULSEWIDTH, MIN_PULSEWIDTH, MIN_PULSEWIDTH, MIN_PULSEWIDTH, MIN_PULSEWIDTH},
		/* pwmMax */
		{MAX_PULSEWIDTH, MAX_PULSEWIDTH, MAX_PULSEWIDTH, MAX_PULSEWIDTH, MAX_PULSEWIDTH, MAX_PULSEWIDTH, MAX_PULSEWIDTH, MAX_PULSEWIDTH}
};

const system::Dynamics::Parameter paramDyn = {
		{K_DYN_INER_XX,K_DYN_INER_YY,K_DYN_INER_ZZ},
		{0.000,0.000,0.000}
};

const autom::SimpleAttitudeKalmanFilter::Parameter paramEst =
{
		/* gainAttAngAcco */
		EST_GAIN_ACCO_ANGLE,
		/* gainAttDriftAcco */
		EST_GAIN_ACCO_DRIFT,
		/* gainAttAngCompass */
		EST_GAIN_COMPASS_ANGLE,
		/* gainAttDriftCompass */
		EST_GAIN_COMPASS_DRIFT
};

} /* namespace system */

