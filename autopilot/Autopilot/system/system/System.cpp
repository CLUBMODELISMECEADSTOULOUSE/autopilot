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

#include <system/params/MavlinkData.hpp>
#include "SystemData.hpp"

#define USB_MUX_PIN 23


namespace system {


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
		  hw::HalImuMpu6000::E_GYR_CNF_250DPS,
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
  _timerArmMgt(0),
  _imuMgt(_imu, imuMgtParam)
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

	_imuMgt.initialize();
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

	/* GCS */
	_gcs.initialize();

	/* Initialize control mode */
	_modeCtrlMgr.initialize();

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
	if (_estimator.getState() != autom::SimpleAttitudeKalmanFilter::E_STATE_INIT)
	{
		setMode(E_SYS_MODE_READY);
	}
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

	/* Execute control mode manager */
	_modeCtrlMgr.execute();

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
//	verifCond = verifCond && ((_radio.getSignedMinVal(hw::Radio::E_RADIO_CHANNEL_YAW)+100)>=_radio.getSigned(hw::Radio::E_RADIO_CHANNEL_YAW));
	verifCond = verifCond && (math_abs(_radio.getSigned(hw::Radio::E_RADIO_CHANNEL_YAW))<= 100);
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

	/* Set control mode to idle */
	_modeCtrlMgr.setMode(ModeControlMgr::E_MODE_IDLE);

	/* Disarm motors */
	disarmMotor();

	return true;
}

/** @brief Switch to armed mode */
bool System::switchToArmedMode()
{
	bool result = true;

	/* Switch the attitude and nav to demanded modes */
	// TODO: implement selection according to PWM channel / mavlink services
	_modeCtrlMgr.setMode(ModeControlMgr::E_MODE_AUTOSTAB);

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
	/* Set control mode to idle */
	_modeCtrlMgr.setMode(ModeControlMgr::E_MODE_IDLE);

	/* Disarm motors */
	disarmMotor();

	return true;
}



/** @brief Process sensors (raw) */
void System::processRawSensors()
{
	uint32_t now = micros();

	// Sample compass
	dataPool.compassIsMeasAvail = _compass.sample(dataPool.compassMagRaw_U);
	if (dataPool.compassIsMeasAvail)
	{
		dataPool.compassLastMeasDateUsec = now;
		math::Vector3i biasMag(95, -24, 4);
		dataPool.compassMagRaw_U -= biasMag;
	}

	// Sample barometer
	dataPool.baroIsMeasAvail = _baro.sample(dataPool.baroPressRaw, dataPool.baroTempRaw);
	if (dataPool.baroIsMeasAvail)
	{
		dataPool.baroLastMeasDateUsec = now;
	}

	// Sample imu1
	_imuMgt.execute();
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

	/* Compass */
	dataPool.compassMag_B(
			((float) (dataPool.compassMagRaw_U.x)) * HALMAGHMC5883L_LSB_1_30GA,
			((float) (dataPool.compassMagRaw_U.y)) * HALMAGHMC5883L_LSB_1_30GA,
			((float) (dataPool.compassMagRaw_U.z)) * HALMAGHMC5883L_LSB_1_30GA);
}

/** @brief Process estimation */
void System::processEstimation()
{
	_estimator.execute();
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

	/* write PWM outputs */
	_pwm.write(&dataPool.pwm_outputs[0]);

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



System system = System();


} /* namespace system */
