/*
 * nrd.h
 *
 *  Created on: 17 janv. 2013
 *      Author: Aberzen
 */


//#include <math/Matrix3.hpp>

#ifndef NRD_H_
#define NRD_H_

#define SCALE_INFLUENCE_MATRIX  (10)
#define SCALE_TORSOR            (10)
#define SCALE_ATT_CTRL_GAIN     (10)

#define CNF_PWM_NUM_FROM_DEVICE     (8)
#define CNF_PWM_NUM_TO_DEVICE       (11)

#define CNF_NB_MOTORS     (4)

#define CNF_TIMER_ARM     (300)

#define CNF_CONTROL_MODE_NAV_PERIOD     (5) // number of time steps between processing two navigation steps


#define MIN_PULSEWIDTH     900
#define MAX_PULSEWIDTH    2100


/* **************************************************** */
/* Physics */

#define PHYSICS_GRAVITY 		(9.81) 	/* [m/s^2] gravity acceleration norm */
#define K_DYN_MASS 			(1.600) /* [Kg] mass */
#define K_DYN_INER_XX 		(0.0229623) /* [Kg.m^2] Diagonal inertia XX */
#define K_DYN_INER_YY 		(0.0448513) /* [Kg.m^2] Diagonal inertia YY */
#define K_DYN_INER_ZZ 		(0.0617154) /* [Kg.m^2] Diagonal inertia ZZ */
#define K_DYN_COM_X 			(0.0000000) /* [m] Center of Mass position (X axis) */
#define K_DYN_COM_Y 			(0.0000000) /* [m] Center of Mass position (Y axis)*/
#define K_DYN_COM_Z 			(0.0000000) /* [m] Center of Mass position (Z axis)*/

/* **************************************************** */
/* Global FSW parameter */

#define FSW_SYSTEM_TICK_PER_MSEC            (1) 	/* tick/ms */
#define FSW_TASK_CTRL_PERIOD_TICK           (10)	/* tick */
#define FSW_TASK_CTRL_PERIOD_TICK_PER_MSEC  (FSW_TASK_CTRL_PERIOD_TICK*FSW_SYSTEM_TICK_PER_MSEC)	/* tick/period/ms */
#define FSW_TASK_CTRL_PERIOD_TICK_PER_SEC   (((float)(FSW_TASK_CTRL_PERIOD_TICK*FSW_SYSTEM_TICK_PER_MSEC))/1000.)	/* tick/period/ms */
#define FSW_TASK_CTRL_FREQ   (1000./((float)(FSW_TASK_CTRL_PERIOD_TICK*FSW_SYSTEM_TICK_PER_MSEC)))	/* tick/period/ms */

/* **************************************************** */
/* Attitude controller: angular attitude with rate bias */

#define CTRL_ATT_X_Kp		(2.9967466)
#define CTRL_ATT_X_Kd		(0.3147850)
#define CTRL_ATT_X_Ki		(0.)
#define CTRL_ATT_X_maxI		(0.)

#define CTRL_ATT_Y_Kp		(5.8534217)
#define CTRL_ATT_Y_Kd		(0.6148567)
#define CTRL_ATT_Y_Ki		(0.)
#define CTRL_ATT_Y_maxI		(0.)

#define CTRL_ATT_Z_Kp		(0.2707139)
#define CTRL_ATT_Z_Kd		(0.1809589)
#define CTRL_ATT_Z_Ki		(0.)
#define CTRL_ATT_Z_maxI		(0.)

/* **************************************************** */
/* Estimator: gain and other parameters for Kalman filter */

#define EST_GAIN_ACCO_ANGLE	( 2.9321531433e-02)  /* gainAcco / angle */
#define EST_GAIN_ACCO_DRIFT	( 4.3864908449e-10)  /* gainAcco / drift */
#define EST_GAIN_COMPASS_ANGLE	( 2.9321531433e-02)  /* gainAcco / angle */
#define EST_GAIN_COMPASS_DRIFT	( 4.3864908449e-10)  /* gainAcco / drift */

#define EST_MAX_CONSECUTIVE_MISSING_MEAS_GYRO (3)
#define EST_MAX_CONSECUTIVE_MISSING_MEAS_ACCO (3)
#define EST_MAX_CONSECUTIVE_MISSING_MEAS_COMPASS (3)
#define EST_MAX_CONSECUTIVE_MISSING_MEAS_BARO (3)
#define EST_MAX_CONSECUTIVE_MISSING_MEAS_GPS (3)

/* **************************************************** */
/* Attitude Guidance */

#define ATTITUDE_GUIDANCE_MODE_AUTOSTAB_PWM_SCALE_ROLL 		(0.0013090) /* rad/pwm */
#define ATTITUDE_GUIDANCE_MODE_AUTOSTAB_PWM_SCALE_PITCH 	(0.0013090) /* rad/pwm */
#define ATTITUDE_GUIDANCE_MODE_AUTOSTAB_PWM_SCALE_YAW	 	(0.0013090) /* rad/s/pwm */



/* **************************************************** */
/* MAVLINK configuration */

#define CONFIG_MAVLINK_EEPROMREV	(0x0012)

#endif /* NRD_H_ */
