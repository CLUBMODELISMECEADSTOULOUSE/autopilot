/*
 * ModeControlData.cpp
 *
 *  Created on: 8 nov. 2015
 *      Author: AdministrateurLocal
 */

#include "ModeControlData.hpp"
#include <system/params/Nrd.hpp>
#include <system/params/NrdGen.hpp>

namespace system {

const attitude::AttitudeController::Parameter paramAttCtrl =
{
		/* ctrl */
		{
				/* axes */
				{
						{
								/* Kp */
								K_ATTCTRL_KP_0,
								/* Kd */
								K_ATTCTRL_KD_0,
								/* Ki; */
								K_ATTCTRL_KI_0,
								/* maxI */
								K_ATTCTRL_MAXI_0
						},
						{
								/* Kp */
								K_ATTCTRL_KP_1,
								/* Kd */
								K_ATTCTRL_KD_1,
								/* Ki; */
								K_ATTCTRL_KI_1,
								/* maxI */
								K_ATTCTRL_MAXI_1
						},
						{
								/* Kp */
								K_ATTCTRL_KP_2,
								/* Kd */
								K_ATTCTRL_KD_2,
								/* Ki; */
								K_ATTCTRL_KI_2,
								/* maxI */
								K_ATTCTRL_MAXI_2
						}
				}
		},
		/* maxCosAngOverTwoErr */
		K_ATTCTRL_MAXERRCOS,
		/* maxSinAngOverTwoErr */
		K_ATTCTRL_MAXERRSIN,
		/* filterX */
		{
				/* a0 */
				K_ATTCTRL_FILT_X_NUM_0,
				/* a1 */
				K_ATTCTRL_FILT_X_NUM_1,
				/* a2 */
				K_ATTCTRL_FILT_X_NUM_2,
				/* -b1 */
				-K_ATTCTRL_FILT_X_DEN_1,
				/* -b2 */
				-K_ATTCTRL_FILT_X_DEN_2,
		},
		/* filterY */
		{
				/* a0 */
				K_ATTCTRL_FILT_Y_NUM_0,
				/* a1 */
				K_ATTCTRL_FILT_Y_NUM_1,
				/* a2 */
				K_ATTCTRL_FILT_Y_NUM_2,
				/* -b1 */
				-K_ATTCTRL_FILT_Y_DEN_1,
				/* -b2 */
				-K_ATTCTRL_FILT_Y_DEN_2,
		},
		/* filterZ */
		{
				/* a0 */
				K_ATTCTRL_FILT_Z_NUM_0,
				/* a1 */
				K_ATTCTRL_FILT_Z_NUM_1,
				/* a2 */
				K_ATTCTRL_FILT_Z_NUM_2,
				/* -b1 */
				-K_ATTCTRL_FILT_Z_DEN_1,
				/* -b2 */
				-K_ATTCTRL_FILT_Z_DEN_2,
		}
} ;

const navigation::NavigationController::Parameter paramNavCtrl =
{
		/* paramCtrl */
		{
				/* axes */
				{
						{
								/* Kp */
								K_NAVCTRL_KP_0,
								/* Kd */
								K_NAVCTRL_KD_0,
								/* Ki; */
								K_NAVCTRL_KI_0,
								/* maxI */
								K_NAVCTRL_MAXI_0
						},
						{
								/* Kp */
								K_NAVCTRL_KP_1,
								/* Kd */
								K_NAVCTRL_KD_1,
								/* Ki; */
								K_NAVCTRL_KI_1,
								/* maxI */
								K_NAVCTRL_MAXI_1
						},
						{
								/* Kp */
								K_NAVCTRL_KP_2,
								/* Kd */
								K_NAVCTRL_KD_2,
								/* Ki; */
								K_NAVCTRL_KI_2,
								/* maxI */
								K_NAVCTRL_MAXI_2
						}
				}
		},
		/* mass */
		1.6
} ;

const autom::ModulatorLut::Parameter paramMod =
{
	/* mod */
	{
		K_MOD_INFLMAT,
		/* frcMaxPos_B */
		K_MOD_FMAXPOS,
		/* frcMaxNeg_B */
		K_MOD_FMAXNEG,
		/* trqMaxPos_B */
		K_MOD_TMAXPOS,
		/* trqMaxNeg_B */
		K_MOD_TMAXNEG,
		/* minPwm */
		K_MOD_MIN
	},
	/* lutTrq */
	K_MOD_LUTTRQ,
	/* lutFrc */
	K_MOD_LUTFRC
};

} /* namespace system */

