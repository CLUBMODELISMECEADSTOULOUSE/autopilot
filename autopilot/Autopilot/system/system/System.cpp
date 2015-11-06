/*
 * System.cpp
 *
 *  Created on: 23 mai 2015
 *      Author: Modélisme
 */


#include <gcs/channel/mavlink_bridge.hpp>

#include "System.hpp"

#include <infra/rtos/Task.hpp>
#include <system/params/NrdGen.hpp>


#define PARAM(type, name, addr, defVal) { MAV_PARAM_TYPE_##type, name, addr, {type : defVal} }
#define USB_MUX_PIN 23


namespace system {

#ifdef MODULATORPINV
autom::ModulatorPinv::DescentVector paramModModDesc = {};
autom::ModulatorPinv::PInvInflMat paramModModPInv = {};
#else
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

#endif

const attitude::AttitudeManager::Parameter attMgrParam =
{
		/* ctrl */
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
		}
} ;

const navigation::NavigationManager::Parameter navMgrParam =
{
		/* paramCtrl */
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
				1.6,
		},
		/* paramDirectThrust */
		{
				/* unitThrust */
				{K_NAV_DIRTHR_X, K_NAV_DIRTHR_Y, K_NAV_DIRTHR_Z}
		},
} ;

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
		{0.100,0.100,0.200},
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

/** @brief Handlers for COM0 interrupts */
SerialHandler(0);


System::System()
: dataPool(),
  _mode(E_SYS_MODE_NONE),
  _buffRx0(&_buffRx0_buffer[0], SERIAL0_RX_LEN),
  _buffTx0(&_buffTx0_buffer[0], SERIAL0_TX_LEN),
  _spiBus(),
  _i2c(),
  _imu(
		  _spiBus,
		  hw::HalImuMpu6000::E_GYR_CNF_1000DPS,
		  hw::HalImuMpu6000::E_ACC_CNF_4G,
		  hw::HalImuMpu6000::E_UPT_FREQ_500HZ),
  _baro(_spiBus),
  _compass(_i2c),
  SerialPortConstParam(_com0, 0, _buffRx0, _buffTx0),
  _pwm(),
  _radio(paramRadio),
  _paramMgt(info, paramCount),
  _mavChannel(MAVLINK_COMM_0,_com0),
  _gcs(MAVLINK_COMM_0),
  _estimator(paramEst),
#ifdef MODULATORPINV
#else
  _modulator(paramMod),
#endif
  _attMgr(attMgrParam),
  _navMgr(navMgrParam),
  _timerArmMgt(0),
  _dyn(paramDyn)
{

}

System::~System()
{

}

void System::initialize()
{
	_paramMgt.loadAllValues();

    /* Disable magnetometer */
	pinMode(63, OUTPUT);
	digitalWrite(63, HIGH);

	/* Short pause to ensure SPI reset */
	infra::Task::delay(10);

	_spiBus.initialize();

	/* Short pause to ensure SPI reset */
	infra::Task::delay(10);

	_i2c.begin();
	_i2c.setSpeed(false);
	// on the APM2 board we have a mux thet switches UART0 between
	// USB and the board header. If the right ArduPPM firmware is
	// installed we can detect if USB is connected using the
	// USB_MUX_PIN
	pinMode(USB_MUX_PIN, INPUT);

	bool usb_connected = !digitalRead(USB_MUX_PIN);

	if (usb_connected)
		_com0.open(115200);
	else
		_com0.open(57600);

	/* Set clock diviser */
	_spiBus.setClockDivider(SPI_CLOCK_DIV32); // 2MHZ SPI rate

	/* Short pause to ensure SPI reset */
	infra::Task::delay(10);

	_imu.initialize();
	_baro.initialize();

	/* Short pause to ensure SPI reset */
	infra::Task::delay(10);
	/* Set clock diviser */
	_spiBus.setClockDivider(SPI_CLOCK_DIV16); // 2MHZ SPI rate

	/* Short pause to ensure SPI reset */
	infra::Task::delay(10);
	_compass.initialize();

	/* pwm */
	_pwm.initialize();

	/* Set fast output (400Hz) */
	for (uint8_t iChannel = 0 ; iChannel<8 ; iChannel++)
	{
		_pwm.setFastOutputChannels(_BV(CH_1), 400);
	}

	/* Estimator */
	_estimator.initialize();

	/* motors */
	_modulator.initialize();

	/* GCS */
	_gcs.initialize();

}

void System::execute()
{
	switch (_mode)
	{
	case E_SYS_MODE_INIT:
		/* Execute init */
		executeInitMode();
		break;
	case E_SYS_MODE_READY:
		/* Execute ready */
		executeReadyMode();
		break;
	case E_SYS_MODE_ARMED:
		/* Execute ready */
		executeArmedMode();
		break;
	case E_SYS_MODE_FAILSAFE:
		/* Execute failsafe */
		executeFailsafeMode();
		break;
	case E_SYS_MODE_NONE:
	default:
		/* Switch to init mode */
		setMode(E_SYS_MODE_INIT);
		break;
	}
}

/** @brief Execute init mode */
void System::executeInitMode()
{
	/* Process GCS new messages */
	_gcs.processNewMessages();

	/* Process sensors */
	processRawSensors();

	/* Post process sensors */
	postProcessSensors();

	/* Process estimation */
	processEstimation();

	/* Process GCS services */
	_gcs.processServices();

	/* Auto switch to ready */
	_timerArmMgt++;
	if (_timerArmMgt >= 1000)
		setMode(E_SYS_MODE_READY);
}

/** @brief Execute ready mode */
void System::executeReadyMode()
{
	/* Process GCS new messages */
	_gcs.processNewMessages();

	/* Process sensors */
	processRawSensors();

	/* Process radio */
	processRadio();

	/* Post process sensors */
	postProcessSensors();

	/* Process estimation */
	processEstimation();

	/* Process actuator */
	processActuators();

	/* Process GCS services */
	_gcs.processServices();


	/* Check if arm order is send */
	bool verifCond = true;
//	char message[50];
//	sprintf(message,"%d %d %d %d",
//			_radio.getSigned(RADIO_IDX_ROLL),
//			_radio.getSigned(RADIO_IDX_PITCH),
//			_radio.getUnsigned(RADIO_IDX_THRUST),
//			_radio.getSigned(RADIO_IDX_YAW));
//	mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_INFO, message);
	verifCond = verifCond && (math_abs(_radio.getSigned(hw::Radio::E_RADIO_CHANNEL_ROLL))<100);
	verifCond = verifCond && (math_abs(_radio.getSigned(hw::Radio::E_RADIO_CHANNEL_PITCH))<100);
	verifCond = verifCond && (_radio.getUnsigned(hw::Radio::E_RADIO_CHANNEL_THRUST)<100);
	verifCond = verifCond && ((_radio.getSignedMaxVal(hw::Radio::E_RADIO_CHANNEL_YAW)-100)<=_radio.getSigned(hw::Radio::E_RADIO_CHANNEL_YAW));
	// TODO add control mode verification (today only stabilized)

	/* Timer management */
	if (verifCond)
		_timerArmMgt++;
	else
		_timerArmMgt = 0;

	/* Check mode transition */
	if (_timerArmMgt >= CNF_TIMER_ARM)
		setMode(E_SYS_MODE_ARMED);

}

/** @brief Execute armed mode */
void System::executeArmedMode()
{
	/* Process GCS new messages */
	_gcs.processNewMessages();

	/* Process sensors */
	processRawSensors();

	/* Process radio */
	processRadio();

	/* Post process sensors */
	postProcessSensors();

	/* Process estimation */
	processEstimation();

	/* Process Navigation */
	processNavigation();

	/* Process Attitude */
	processAttitude();

	/* Process actuator */
	processActuators();

	/* Process GCS services */
	_gcs.processServices();


	/* Check transitions conditions */
	bool verifCond = true;
//	char message[50];
//	sprintf(message,"%d %d %d %d",
//			_radio.getSigned(RADIO_IDX_ROLL),
//			_radio.getSigned(RADIO_IDX_PITCH),
//			_radio.getUnsigned(RADIO_IDX_THRUST),
//			_radio.getSigned(RADIO_IDX_YAW));
//	mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_INFO, message);
	verifCond = verifCond && (math_abs(_radio.getSigned(hw::Radio::E_RADIO_CHANNEL_ROLL))<100);
	verifCond = verifCond && (math_abs(_radio.getSigned(hw::Radio::E_RADIO_CHANNEL_PITCH))<100);
	verifCond = verifCond && (_radio.getUnsigned(hw::Radio::E_RADIO_CHANNEL_THRUST)<100);
	verifCond = verifCond && (math_abs(_radio.getSigned(hw::Radio::E_RADIO_CHANNEL_YAW))<100);
	// TODO add control mode verification (today only stabilized)

	/* Timer management */
	if (verifCond)
		_timerArmMgt++;
	else
		_timerArmMgt = 0;

	/* Check mode transition */
	if (_timerArmMgt >= CNF_TIMER_ARM)
	{
		setMode(E_SYS_MODE_READY);
	}
}

/** @brief Execute fail safe mode */
void System::executeFailsafeMode()
{

}

/** @Brief Set new mode */
bool System::setMode(Mode mode)
{
	bool result = false;

	/* Check with current mode */
	if (_mode == mode)
		return true;

	switch (mode)
	{
	case E_SYS_MODE_INIT:
		/* Switch to initialization */
		result = switchToInitMode();
		break;
	case E_SYS_MODE_READY:
		/* Switch to ready */
		result = switchToReadyMode();
		break;
	case E_SYS_MODE_ARMED:
		/* Switch to ready */
		result = switchToArmedMode();
		break;
	case E_SYS_MODE_FAILSAFE:
		/* Switch to failsafe */
		result = switchToFailsafeMode();
		break;
	case E_SYS_MODE_NONE:
	default:
		/* Do nothing, invalid mode */
		break;
	}

	/* If transition accepted, switch to mode */
	if (result)
		_mode = mode;

	/* Return result */
	return result;
}


/** @brief Switch to init mode */
bool System::switchToInitMode()
{
	return true;
}

/** @brief Switch to ready mode */
bool System::switchToReadyMode()
{
	/* reset counter */
	_timerArmMgt = 0;

	/* Set attitude and nav manager to none mode */
	_attMgr.setMode(attitude::AttitudeManager::E_ATT_MODE_NONE);
	_navMgr.setMode(navigation::NavigationManager::E_NAV_MODE_NONE);

	/* Arm motors */
	disarmMotor();
	dataPool.ctrlTrqDemB(0.,0.,0.);
	dataPool.ctrlFrcDemB(0.,0.,0.);

	return true;
}

/** @brief Switch to armed mode */
bool System::switchToArmedMode()
{
	bool result = true;

	/* Switch the attitude and nav to demanded modes */
	result &= _attMgr.setMode(attitude::AttitudeManager::E_ATT_MODE_STABNOYAW);
	result &= _navMgr.setMode(navigation::NavigationManager::E_NAV_MODE_DIRECTTHRUST);

	if (result)
	{
		/* Arm motors */
		armMotor();

	}
	/* reset counter */
	_timerArmMgt = 0;

	return result;
}


/** @brief Switch to fail safe mode */
bool System::switchToFailsafeMode()
{
	/* Set commanded force and torque to zero */
	dataPool.ctrlTrqDemB(0,0,0);
	dataPool.ctrlFrcDemB(0,0,0);

	/* Disarm motors */
	disarmMotor();

	return true;
}



/** @brief Process sensors (raw) */
void System::processRawSensors()
{
	uint32_t now = micros();

	// Sample compass
	if (_compass.sample(dataPool.compassMagRaw_U))
	{
		dataPool.compassLastMeasDateUsec = now;
	}

	// Sample barometer
	if (_baro.sample(dataPool.baroPressRaw_U, dataPool.baroTempRaw_U))
	{
		dataPool.baroLastMeasDateUsec = now;
	}

	// Sample imu1
	if (_imu.sample(dataPool.imuRateRaw_U, dataPool.imuAccRaw_U,dataPool.imuTemp))
	{
		dataPool.imuLastMeasDateUsec = now;
	}
}

/** @brief Process RC (raw) */
void System::processRadio()
{
	_pwm.read(&dataPool.pwm_inputs[0]);
}

/** @brief Post process sensors */
void System::postProcessSensors()
{
//	char buffer[100];

	/* IMU gyro */
	dataPool.imuRate_B(
			((float) dataPool.imuRateRaw_U.x) * GYRO_SCALE_1000DPS,
			((float) dataPool.imuRateRaw_U.y) * GYRO_SCALE_1000DPS,
			((float) dataPool.imuRateRaw_U.z) * GYRO_SCALE_1000DPS);

	/* IMU acco */
	dataPool.imuAcc_B(
			((float) dataPool.imuAccRaw_U.x) * ACC_SCALE_4G,
			((float) dataPool.imuAccRaw_U.y) * ACC_SCALE_4G,
			((float) dataPool.imuAccRaw_U.z) * ACC_SCALE_4G);

	/* Compass */
	dataPool.compassMag_B(
			((float) (dataPool.compassMagRaw_U.x - 61)) * HALMAGHMC5883L_LSB_1_30GA,
			((float) (dataPool.compassMagRaw_U.y + 27)) * HALMAGHMC5883L_LSB_1_30GA,
			((float) (dataPool.compassMagRaw_U.z -  3)) * HALMAGHMC5883L_LSB_1_30GA);
}

/** @brief Process estimation */
void System::processEstimation()
{
	_estimator.update();
}

/** @brief Process actuators */
void System::processActuators()
{
	if (_radio.getUnsigned(hw::Radio::E_RADIO_CHANNEL_THRUST) < 50)
	{
		/* Inhibit motors when radio thrust is set to zero */
		for (uint8_t iMotor=0 ; iMotor<CNF_NB_MOTORS ; iMotor++)
		{
			dataPool.pwm_outputs[iMotor] = MIN_PULSEWIDTH;
		}
	}
	else
	{
		/* Compute motor inputs */
		_modulator.calcMotorCommand(
				dataPool.ctrlFrcDemB,
				dataPool.ctrlTrqDemB,
				&dataPool.pwm_outputs[0]);
	}

	/* write PWM outputs */
	_pwm.write(&dataPool.pwm_outputs[0]);

	/* Compute efforts produced by the modulator */
	_modulator.calcTorsor(
			&dataPool.pwm_outputs[0],
			dataPool.estForce_B,
			dataPool.estTorque_B);
}

/** @brief Process attitude controller */
void System::processAttitude()
{
	_attMgr.execute();
}

/** @brief Process navigation controller */
void System::processNavigation()
{
	_navMgr.execute();
}

/** @brief Arm motors */
void System::armMotor()
{
	/* Arm motors */
	for (uint8_t idx=0 ; idx<CNF_NB_MOTORS ; idx++)
	{
		_pwm.enable_out(idx);
	}
}

/** @brief Disarm motors */
void System::disarmMotor()
{
	/* Set out values to min PWM values and disarm */
	for (uint8_t iMotor=0 ; iMotor<CNF_NB_MOTORS ; iMotor++)
	{
		dataPool.pwm_outputs[iMotor] = MIN_PULSEWIDTH;
		_pwm.write(iMotor, 0);
		_pwm.disable_out(iMotor);
	}
}


PROGMEM const mavlink::ParameterMgt::ParamInfo info[] = {
		PARAM(UINT16,"RD_MIN_0", (void*)&paramRadio.pwmMin[0], MIN_PULSEWIDTH),
		PARAM(UINT16,"RD_MIN_1", (void*)&paramRadio.pwmMin[1], MIN_PULSEWIDTH),
		PARAM(UINT16,"RD_MIN_2", (void*)&paramRadio.pwmMin[2], MIN_PULSEWIDTH),
		PARAM(UINT16,"RD_MIN_3", (void*)&paramRadio.pwmMin[3], MIN_PULSEWIDTH),
		PARAM(UINT16,"RD_MIN_4", (void*)&paramRadio.pwmMin[4], MIN_PULSEWIDTH),
		PARAM(UINT16,"RD_MIN_5", (void*)&paramRadio.pwmMin[5], MIN_PULSEWIDTH),
		PARAM(UINT16,"RD_MIN_6", (void*)&paramRadio.pwmMin[6], MIN_PULSEWIDTH),
		PARAM(UINT16,"RD_MIN_7", (void*)&paramRadio.pwmMin[7], MIN_PULSEWIDTH),
		PARAM(UINT16,"RD_MAX_0", (void*)&paramRadio.pwmMax[0], MAX_PULSEWIDTH),
		PARAM(UINT16,"RD_MAX_1", (void*)&paramRadio.pwmMax[1], MAX_PULSEWIDTH),
		PARAM(UINT16,"RD_MAX_2", (void*)&paramRadio.pwmMax[2], MAX_PULSEWIDTH),
		PARAM(UINT16,"RD_MAX_3", (void*)&paramRadio.pwmMax[3], MAX_PULSEWIDTH),
		PARAM(UINT16,"RD_MAX_4", (void*)&paramRadio.pwmMax[4], MAX_PULSEWIDTH),
		PARAM(UINT16,"RD_MAX_5", (void*)&paramRadio.pwmMax[5], MAX_PULSEWIDTH),
		PARAM(UINT16,"RD_MAX_6", (void*)&paramRadio.pwmMax[6], MAX_PULSEWIDTH),
		PARAM(UINT16,"RD_MAX_7", (void*)&paramRadio.pwmMax[7], MAX_PULSEWIDTH),
		PARAM(UINT16,"RD_ZERO_0", (void*)&paramRadio.pwmZero[0], (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1),
		PARAM(UINT16,"RD_ZERO_1", (void*)&paramRadio.pwmZero[1], (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1),
		PARAM(UINT16,"RD_ZERO_2", (void*)&paramRadio.pwmZero[2], (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1),
		PARAM(UINT16,"RD_ZERO_3", (void*)&paramRadio.pwmZero[3], (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1),
		PARAM(UINT16,"RD_ZERO_4", (void*)&paramRadio.pwmZero[4], (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1),
		PARAM(UINT16,"RD_ZERO_5", (void*)&paramRadio.pwmZero[5], (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1),
		PARAM(UINT16,"RD_ZERO_6", (void*)&paramRadio.pwmZero[6], (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1),
		PARAM(UINT16,"RD_ZERO_7", (void*)&paramRadio.pwmZero[7], (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1),
		PARAM(UINT16,"RD_REVERS", (void*)&paramRadio.reversed, 0),
		PARAM(REAL32,"ATT_KP_X", (void*)&attMgrParam.ctrl.ctrl.axes[0].Kp, K_ATTCTRL_KP_0),
		PARAM(REAL32,"ATT_KP_Y", (void*)&attMgrParam.ctrl.ctrl.axes[1].Kp, K_ATTCTRL_KP_1),
		PARAM(REAL32,"ATT_KP_Z", (void*)&attMgrParam.ctrl.ctrl.axes[2].Kp, K_ATTCTRL_KP_2),
		PARAM(REAL32,"ATT_KD_X", (void*)&attMgrParam.ctrl.ctrl.axes[0].Kd, K_ATTCTRL_KD_0),
		PARAM(REAL32,"ATT_KD_Y", (void*)&attMgrParam.ctrl.ctrl.axes[1].Kd, K_ATTCTRL_KD_1),
		PARAM(REAL32,"ATT_KD_Z", (void*)&attMgrParam.ctrl.ctrl.axes[2].Kd, K_ATTCTRL_KD_2),
		PARAM(REAL32,"ATT_KI_X", (void*)&attMgrParam.ctrl.ctrl.axes[0].Ki, K_ATTCTRL_KI_0),
		PARAM(REAL32,"ATT_KI_Y", (void*)&attMgrParam.ctrl.ctrl.axes[1].Ki, K_ATTCTRL_KI_1),
		PARAM(REAL32,"ATT_KI_Z", (void*)&attMgrParam.ctrl.ctrl.axes[2].Ki, K_ATTCTRL_KI_2),
		PARAM(REAL32,"ATT_MAXI_X", (void*)&attMgrParam.ctrl.ctrl.axes[0].maxI, K_ATTCTRL_MAXI_0),
		PARAM(REAL32,"ATT_MAXI_Y", (void*)&attMgrParam.ctrl.ctrl.axes[1].maxI, K_ATTCTRL_MAXI_1),
		PARAM(REAL32,"ATT_MAXI_Z", (void*)&attMgrParam.ctrl.ctrl.axes[2].maxI, K_ATTCTRL_MAXI_2),
		PARAM(REAL32,"ATT_FX_N0", (void*)&attMgrParam.ctrl.filterX.a0, K_ATTCTRL_FILT_X_NUM_0),
		PARAM(REAL32,"ATT_FX_N1", (void*)&attMgrParam.ctrl.filterX.a1, K_ATTCTRL_FILT_X_NUM_1),
		PARAM(REAL32,"ATT_FX_N2", (void*)&attMgrParam.ctrl.filterX.a2, K_ATTCTRL_FILT_X_NUM_2),
		PARAM(REAL32,"ATT_FX_D1", (void*)&attMgrParam.ctrl.filterX.b1, -K_ATTCTRL_FILT_X_DEN_1),
		PARAM(REAL32,"ATT_FX_D2", (void*)&attMgrParam.ctrl.filterX.b2, -K_ATTCTRL_FILT_X_DEN_2),
		PARAM(REAL32,"ATT_FY_N0", (void*)&attMgrParam.ctrl.filterY.a0, K_ATTCTRL_FILT_Y_NUM_0),
		PARAM(REAL32,"ATT_FY_N1", (void*)&attMgrParam.ctrl.filterY.a1, K_ATTCTRL_FILT_Y_NUM_1),
		PARAM(REAL32,"ATT_FY_N2", (void*)&attMgrParam.ctrl.filterY.a2, K_ATTCTRL_FILT_Y_NUM_2),
		PARAM(REAL32,"ATT_FY_D1", (void*)&attMgrParam.ctrl.filterY.b1, -K_ATTCTRL_FILT_Y_DEN_1),
		PARAM(REAL32,"ATT_FY_D2", (void*)&attMgrParam.ctrl.filterY.b2, -K_ATTCTRL_FILT_Y_DEN_2),
		PARAM(REAL32,"ATT_FZ_N0", (void*)&attMgrParam.ctrl.filterZ.a0, K_ATTCTRL_FILT_Z_NUM_0),
		PARAM(REAL32,"ATT_FZ_N1", (void*)&attMgrParam.ctrl.filterZ.a1, K_ATTCTRL_FILT_Z_NUM_1),
		PARAM(REAL32,"ATT_FZ_N2", (void*)&attMgrParam.ctrl.filterZ.a2, K_ATTCTRL_FILT_Z_NUM_2),
		PARAM(REAL32,"ATT_FZ_D1", (void*)&attMgrParam.ctrl.filterZ.b1, -K_ATTCTRL_FILT_Z_DEN_1),
		PARAM(REAL32,"ATT_FZ_D2", (void*)&attMgrParam.ctrl.filterZ.b2, -K_ATTCTRL_FILT_Z_DEN_2),
		PARAM(INT32,"MOD_IM_FX0", (void*)&paramMod.mod.inflMat[0][0], K_MOD_INFLMAT_0_0),
		PARAM(INT32,"MOD_IM_FY0", (void*)&paramMod.mod.inflMat[0][1], K_MOD_INFLMAT_0_1),
		PARAM(INT32,"MOD_IM_FZ0", (void*)&paramMod.mod.inflMat[0][2], K_MOD_INFLMAT_0_2),
		PARAM(INT32,"MOD_IM_TX0", (void*)&paramMod.mod.inflMat[0][3], K_MOD_INFLMAT_0_3),
		PARAM(INT32,"MOD_IM_TY0", (void*)&paramMod.mod.inflMat[0][4], K_MOD_INFLMAT_0_4),
		PARAM(INT32,"MOD_IM_TZ0", (void*)&paramMod.mod.inflMat[0][5], K_MOD_INFLMAT_0_5),
		PARAM(INT32,"MOD_IM_FX1", (void*)&paramMod.mod.inflMat[1][0], K_MOD_INFLMAT_1_0),
		PARAM(INT32,"MOD_IM_FY1", (void*)&paramMod.mod.inflMat[1][1], K_MOD_INFLMAT_1_1),
		PARAM(INT32,"MOD_IM_FZ1", (void*)&paramMod.mod.inflMat[1][2], K_MOD_INFLMAT_1_2),
		PARAM(INT32,"MOD_IM_TX1", (void*)&paramMod.mod.inflMat[1][3], K_MOD_INFLMAT_1_3),
		PARAM(INT32,"MOD_IM_TY1", (void*)&paramMod.mod.inflMat[1][4], K_MOD_INFLMAT_1_4),
		PARAM(INT32,"MOD_IM_TZ1", (void*)&paramMod.mod.inflMat[1][5], K_MOD_INFLMAT_1_5),
		PARAM(INT32,"MOD_IM_FX2", (void*)&paramMod.mod.inflMat[2][0], K_MOD_INFLMAT_2_0),
		PARAM(INT32,"MOD_IM_FY2", (void*)&paramMod.mod.inflMat[2][1], K_MOD_INFLMAT_2_1),
		PARAM(INT32,"MOD_IM_FZ2", (void*)&paramMod.mod.inflMat[2][2], K_MOD_INFLMAT_2_2),
		PARAM(INT32,"MOD_IM_TX2", (void*)&paramMod.mod.inflMat[2][3], K_MOD_INFLMAT_2_3),
		PARAM(INT32,"MOD_IM_TY2", (void*)&paramMod.mod.inflMat[2][4], K_MOD_INFLMAT_2_4),
		PARAM(INT32,"MOD_IM_TZ2", (void*)&paramMod.mod.inflMat[2][5], K_MOD_INFLMAT_2_5),
		PARAM(INT32,"MOD_IM_FX3", (void*)&paramMod.mod.inflMat[3][0], K_MOD_INFLMAT_3_0),
		PARAM(INT32,"MOD_IM_FY3", (void*)&paramMod.mod.inflMat[3][1], K_MOD_INFLMAT_3_1),
		PARAM(INT32,"MOD_IM_FZ3", (void*)&paramMod.mod.inflMat[3][2], K_MOD_INFLMAT_3_2),
		PARAM(INT32,"MOD_IM_TX3", (void*)&paramMod.mod.inflMat[3][3], K_MOD_INFLMAT_3_3),
		PARAM(INT32,"MOD_IM_TY3", (void*)&paramMod.mod.inflMat[3][4], K_MOD_INFLMAT_3_4),
		PARAM(INT32,"MOD_IM_TZ3", (void*)&paramMod.mod.inflMat[3][5], K_MOD_INFLMAT_3_5),
		PARAM(INT32,"MOD_FMAX_X", (void*)&paramMod.mod.frcMaxPos_B[0], K_MOD_FMAXPOS_0),
		PARAM(INT32,"MOD_FMAX_Y", (void*)&paramMod.mod.frcMaxPos_B[1], K_MOD_FMAXPOS_1),
		PARAM(INT32,"MOD_FMAX_Z", (void*)&paramMod.mod.frcMaxPos_B[2], K_MOD_FMAXPOS_2),
		PARAM(INT32,"MOD_FMIN_X", (void*)&paramMod.mod.frcMaxNeg_B[0], K_MOD_FMAXNEG_0),
		PARAM(INT32,"MOD_FMIN_Y", (void*)&paramMod.mod.frcMaxNeg_B[1], K_MOD_FMAXNEG_1),
		PARAM(INT32,"MOD_FMIN_Z", (void*)&paramMod.mod.frcMaxNeg_B[2], K_MOD_FMAXNEG_2),
		PARAM(INT32,"MOD_TMAX_X", (void*)&paramMod.mod.trqMaxPos_B[0], K_MOD_TMAXPOS_0),
		PARAM(INT32,"MOD_TMAX_Y", (void*)&paramMod.mod.trqMaxPos_B[1], K_MOD_TMAXPOS_1),
		PARAM(INT32,"MOD_TMAX_Z", (void*)&paramMod.mod.trqMaxPos_B[2], K_MOD_TMAXPOS_2),
		PARAM(INT32,"MOD_TMIN_X", (void*)&paramMod.mod.trqMaxNeg_B[0], K_MOD_TMAXNEG_0),
		PARAM(INT32,"MOD_TMIN_Y", (void*)&paramMod.mod.trqMaxNeg_B[1], K_MOD_TMAXNEG_1),
		PARAM(INT32,"MOD_TMIN_Z", (void*)&paramMod.mod.trqMaxNeg_B[2], K_MOD_TMAXNEG_2),
		PARAM(UINT16,"MOD_MIN0", (void*)&paramMod.mod.minPwm[0], K_MOD_MIN0),
		PARAM(UINT16,"MOD_MIN1", (void*)&paramMod.mod.minPwm[1], K_MOD_MIN1),
		PARAM(UINT16,"MOD_MIN2", (void*)&paramMod.mod.minPwm[2], K_MOD_MIN2),
		PARAM(UINT16,"MOD_MIN3", (void*)&paramMod.mod.minPwm[3], K_MOD_MIN3),
		PARAM(INT32,"MOD_LUT_TXM0", (void*)&paramMod.lutTrq[0][0][0], K_MOD_LUTTRQ_0_0_0),
		PARAM(INT32,"MOD_LUT_TXM1", (void*)&paramMod.lutTrq[0][0][1], K_MOD_LUTTRQ_0_0_1),
		PARAM(INT32,"MOD_LUT_TXM2", (void*)&paramMod.lutTrq[0][0][2], K_MOD_LUTTRQ_0_0_2),
		PARAM(INT32,"MOD_LUT_TXM3", (void*)&paramMod.lutTrq[0][0][3], K_MOD_LUTTRQ_0_0_3),
		PARAM(INT32,"MOD_LUT_TYM0", (void*)&paramMod.lutTrq[0][1][0], K_MOD_LUTTRQ_0_1_0),
		PARAM(INT32,"MOD_LUT_TYM1", (void*)&paramMod.lutTrq[0][1][1], K_MOD_LUTTRQ_0_1_1),
		PARAM(INT32,"MOD_LUT_TYM2", (void*)&paramMod.lutTrq[0][1][2], K_MOD_LUTTRQ_0_1_2),
		PARAM(INT32,"MOD_LUT_TYM3", (void*)&paramMod.lutTrq[0][1][3], K_MOD_LUTTRQ_0_1_3),
		PARAM(INT32,"MOD_LUT_TZM0", (void*)&paramMod.lutTrq[0][2][0], K_MOD_LUTTRQ_0_2_0),
		PARAM(INT32,"MOD_LUT_TZM1", (void*)&paramMod.lutTrq[0][2][1], K_MOD_LUTTRQ_0_2_1),
		PARAM(INT32,"MOD_LUT_TZM2", (void*)&paramMod.lutTrq[0][2][2], K_MOD_LUTTRQ_0_2_2),
		PARAM(INT32,"MOD_LUT_TZM3", (void*)&paramMod.lutTrq[0][2][3], K_MOD_LUTTRQ_0_2_3),
		PARAM(INT32,"MOD_LUT_TXP0", (void*)&paramMod.lutTrq[1][0][0], K_MOD_LUTTRQ_1_0_0),
		PARAM(INT32,"MOD_LUT_TXP1", (void*)&paramMod.lutTrq[1][0][1], K_MOD_LUTTRQ_1_0_1),
		PARAM(INT32,"MOD_LUT_TXP2", (void*)&paramMod.lutTrq[1][0][2], K_MOD_LUTTRQ_1_0_2),
		PARAM(INT32,"MOD_LUT_TXP3", (void*)&paramMod.lutTrq[1][0][3], K_MOD_LUTTRQ_1_0_3),
		PARAM(INT32,"MOD_LUT_TYP0", (void*)&paramMod.lutTrq[1][1][0], K_MOD_LUTTRQ_1_1_0),
		PARAM(INT32,"MOD_LUT_TYP1", (void*)&paramMod.lutTrq[1][1][1], K_MOD_LUTTRQ_1_1_1),
		PARAM(INT32,"MOD_LUT_TYP2", (void*)&paramMod.lutTrq[1][1][2], K_MOD_LUTTRQ_1_1_2),
		PARAM(INT32,"MOD_LUT_TYP3", (void*)&paramMod.lutTrq[1][1][3], K_MOD_LUTTRQ_1_1_3),
		PARAM(INT32,"MOD_LUT_TZP0", (void*)&paramMod.lutTrq[1][2][0], K_MOD_LUTTRQ_1_2_0),
		PARAM(INT32,"MOD_LUT_TZP1", (void*)&paramMod.lutTrq[1][2][1], K_MOD_LUTTRQ_1_2_1),
		PARAM(INT32,"MOD_LUT_TZP2", (void*)&paramMod.lutTrq[1][2][2], K_MOD_LUTTRQ_1_2_2),
		PARAM(INT32,"MOD_LUT_TZP3", (void*)&paramMod.lutTrq[1][2][3], K_MOD_LUTTRQ_1_2_3),
		PARAM(INT32,"MOD_LUT_FXM0", (void*)&paramMod.lutFrc[0][0][0], K_MOD_LUTFRC_0_0_0),
		PARAM(INT32,"MOD_LUT_FXM1", (void*)&paramMod.lutFrc[0][0][1], K_MOD_LUTFRC_0_0_1),
		PARAM(INT32,"MOD_LUT_FXM2", (void*)&paramMod.lutFrc[0][0][2], K_MOD_LUTFRC_0_0_2),
		PARAM(INT32,"MOD_LUT_FXM3", (void*)&paramMod.lutFrc[0][0][3], K_MOD_LUTFRC_0_0_3),
		PARAM(INT32,"MOD_LUT_FYM0", (void*)&paramMod.lutFrc[0][1][0], K_MOD_LUTFRC_0_1_0),
		PARAM(INT32,"MOD_LUT_FYM1", (void*)&paramMod.lutFrc[0][1][1], K_MOD_LUTFRC_0_1_1),
		PARAM(INT32,"MOD_LUT_FYM2", (void*)&paramMod.lutFrc[0][1][2], K_MOD_LUTFRC_0_1_2),
		PARAM(INT32,"MOD_LUT_FYM3", (void*)&paramMod.lutFrc[0][1][3], K_MOD_LUTFRC_0_1_3),
		PARAM(INT32,"MOD_LUT_FZM0", (void*)&paramMod.lutFrc[0][2][0], K_MOD_LUTFRC_0_2_0),
		PARAM(INT32,"MOD_LUT_FZM1", (void*)&paramMod.lutFrc[0][2][1], K_MOD_LUTFRC_0_2_1),
		PARAM(INT32,"MOD_LUT_FZM2", (void*)&paramMod.lutFrc[0][2][2], K_MOD_LUTFRC_0_2_2),
		PARAM(INT32,"MOD_LUT_FZM3", (void*)&paramMod.lutFrc[0][2][3], K_MOD_LUTFRC_0_2_3),
		PARAM(INT32,"MOD_LUT_FXP0", (void*)&paramMod.lutFrc[1][0][0], K_MOD_LUTFRC_1_0_0),
		PARAM(INT32,"MOD_LUT_FXP1", (void*)&paramMod.lutFrc[1][0][1], K_MOD_LUTFRC_1_0_1),
		PARAM(INT32,"MOD_LUT_FXP2", (void*)&paramMod.lutFrc[1][0][2], K_MOD_LUTFRC_1_0_2),
		PARAM(INT32,"MOD_LUT_FXP3", (void*)&paramMod.lutFrc[1][0][3], K_MOD_LUTFRC_1_0_3),
		PARAM(INT32,"MOD_LUT_FYP0", (void*)&paramMod.lutFrc[1][1][0], K_MOD_LUTFRC_1_1_0),
		PARAM(INT32,"MOD_LUT_FYP1", (void*)&paramMod.lutFrc[1][1][1], K_MOD_LUTFRC_1_1_1),
		PARAM(INT32,"MOD_LUT_FYP2", (void*)&paramMod.lutFrc[1][1][2], K_MOD_LUTFRC_1_1_2),
		PARAM(INT32,"MOD_LUT_FYP3", (void*)&paramMod.lutFrc[1][1][3], K_MOD_LUTFRC_1_1_3),
		PARAM(INT32,"MOD_LUT_FZP0", (void*)&paramMod.lutFrc[1][2][0], K_MOD_LUTFRC_1_2_0),
		PARAM(INT32,"MOD_LUT_FZP1", (void*)&paramMod.lutFrc[1][2][1], K_MOD_LUTFRC_1_2_1),
		PARAM(INT32,"MOD_LUT_FZP2", (void*)&paramMod.lutFrc[1][2][2], K_MOD_LUTFRC_1_2_2),
		PARAM(INT32,"MOD_LUT_FZP3", (void*)&paramMod.lutFrc[1][2][3], K_MOD_LUTFRC_1_2_3),
		PARAM(REAL32,"EA_ANG_ACC", (void*)&paramEst.gainAttAngAcco, EST_GAIN_ACCO_ANGLE),
		PARAM(REAL32,"EA_DRI_ACC", (void*)&paramEst.gainAttDriftAcco, EST_GAIN_ACCO_DRIFT),
		PARAM(REAL32,"EA_ANG_CMP", (void*)&paramEst.gainAttAngCompass, EST_GAIN_COMPASS_ANGLE),
		PARAM(REAL32,"EA_DRI_CMP", (void*)&paramEst.gainAttDriftCompass, EST_GAIN_COMPASS_DRIFT),
		PARAM(INT32,"NAV_DIRTHR_X", (void*)&navMgrParam.paramDirectThrust.unitThrust[0], K_NAV_DIRTHR_X),
		PARAM(INT32,"NAV_DIRTHR_Y", (void*)&navMgrParam.paramDirectThrust.unitThrust[1], K_NAV_DIRTHR_Y),
		PARAM(INT32,"NAV_DIRTHR_Z", (void*)&navMgrParam.paramDirectThrust.unitThrust[2], K_NAV_DIRTHR_Z)
};

uint16_t paramCount = 147;

System system = System();


} /* namespace system */
