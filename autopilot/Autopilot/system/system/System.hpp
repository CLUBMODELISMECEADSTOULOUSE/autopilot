/*
 * System.hpp
 *
 *  Created on: 23 mai 2015
 *      Author: Modélisme
 */

#ifndef SYSTEM_HPP_
#define SYSTEM_HPP_

#include <avr/pgmspace.h>

#include <gcs/channel/SerialChannel.hpp>

#include <hw/spi/SpiBus.hpp>
#include <hw/bus/I2C.hpp>

#include <hw/imu/HalImuMpu6000.hpp>
#include <hw/mag/HalMagHMC5883L.hpp>
#include <hw/baro/HalBarometerMs5611OverSpi.hpp>
#include <hw/serial/Serial.hpp>
#include <hw/pwm/PwmApm25.hpp>

#include <hw/radio/Radio.hpp>

#include <system/params/Nrd.hpp>

#include <gcs/gcs/Gcs.hpp>
#include <gcs/channel/SerialChannel.hpp>
#include <gcs/param/ParameterMgt.hpp>

#include <autom/est/SimpleAttitudeKalmanFilter.hpp>
#include <mode/control/ModeControl.hpp>
#include <system/system/DataPool.hpp>
#include <system/system/Dynamics.hpp>

#include <fdir/mgt/FdirManager.hpp>

#ifndef SERIAL0_RX_LEN
#define SERIAL0_RX_LEN (256)
#endif

#ifndef SERIAL0_TX_LEN
#define SERIAL0_TX_LEN (128)
#endif


namespace system {

class System
{
public:
	typedef enum
	{
		E_SYS_MODE_NONE = 0,
		E_SYS_MODE_INIT,
		E_SYS_MODE_READY,
		E_SYS_MODE_ARMED,
		E_SYS_MODE_FAILSAFE
	} Mode;

public:
	/** @brief Default constructor */
	System();

	/** @brief Destructor */
	virtual ~System();

	/** @Brief Initialize the system */
	void initialize();

	/** @Brief Execute the system */
	void execute();

	/* ----- States ----------------------------------- */
public:

	/** @Brief Getter for current mode */
	inline Mode getMode();

	/** @Brief Set new mode */
	bool setMode(Mode mode);

protected:
	/** @brief Execute init mode */
	void executeInitMode();

	/** @brief Execute ready mode */
	void executeReadyMode();

	/** @brief Execute armed mode */
	void executeArmedMode();

	/** @brief Execute fail safe mode */
	void executeFailsafeMode();

	/** @brief Switch to init mode */
	bool switchToInitMode();

	/** @brief Switch to armed mode */
	bool switchToArmedMode();

	/** @brief Switch to ready mode */
	bool switchToReadyMode();

	/** @brief Switch to fail safe mode */
	bool switchToFailsafeMode();

	/* ----- Hardware ----------------------------------- */
public:

	/** @brief Getter method for SPI bus object */
	inline hw::SpiBus& getSpiBus();

	/** @brief Getter method for I2C bus object */
	inline hw::I2C& getI2C();

	/** @brief Getter method for Imu object */
	inline hw::HalImu& getImu();

	/** @brief Getter method for Compass object */
	inline hw::HalMagnetometer& getCompass();

	/** @brief Getter method for Barometer object */
	inline hw::HalBarometer& getBarometer();

	/** @brief Getter method for COM0 serial object */
	inline hw::Serial& getCom0();

	/* ----- Services ----------------------------------- */

	/** @brief Getter method for Parameter Management Service */
	inline mavlink::ParameterMgt& getParameterMgt();

	/** @brief Getter method for radio service */
	inline hw::Radio& getRadio();

	/** @brief FDIR manager */
	fdir::FdirManager& getFdir();

	/* ----- Motors ----------------------------------- */
protected:

	/** @brief Arm motors */
	void armMotor();

	/** @brief Disarm motors */
	void disarmMotor();


	/* ----- Others ----------------------------------- */
protected:

	/** @brief Process sensors (raw) */
	void processRawSensors();

	/** @brief Process Radio (raw) */
	void processRadio();

	/** @brief Post process sensors */
	void postProcessSensors();

	/** @brief Process estimation */
	void processEstimation();

	/** @brief Process actuators */
	void processActuators();

public:

	/** @brief Datapool */
	DataPool dataPool;

protected:
	/** @brief Current mode */
	Mode _mode;

	/** @brief Raw buffer for RX0 circular buffer */
	uint8_t _buffRx0_buffer[SERIAL0_RX_LEN];

	/** @brief Raw buffer for TX0 circular buffer */
	uint8_t _buffTx0_buffer[SERIAL0_TX_LEN];

	/** @brief RX circular buffer for Serial #0 */
	infra::Buffer _buffRx0;

	/** @brief TX circular buffer for Serial #0 */
	infra::Buffer _buffTx0;

	/** @brief SPI bus object */
	hw::SpiBus _spiBus;

	/** @brief I2C bus object */
	hw::I2C _i2c;

	/** @brief IMU object */
	hw::HalImuMpu6000 _imu;

	/** @brief Barometer object */
	hw::HalBarometerMs5611OverSpi _baro;

	/** @brief Compass object */
	hw::HalMagHMC5883L _compass;

	/** @brief COM0 object */
	hw::Serial _com0;

	/** @brief PWM */
	hw::PwmApm25 _pwm;

	/** @Radio */
	hw::Radio _radio;

	/** @brief Parameter Management object */
	mavlink::ParameterMgt _paramMgt;

	/** @brief MAV channel object */
	mavlink::SerialChannel _mavChannel;

	/** @brief MAV GCS object */
	mavlink::Gcs _gcs;

	/** @brief Estimator */
	autom::SimpleAttitudeKalmanFilter _estimator;

	/** @brief arm/disarm timer */
	uint16_t _timerArmMgt;

	/** @brief FDIR manager */
	fdir::FdirManager _fdir;

};

hw::Serial& System::getCom0()
{
	return _com0;
}

hw::SpiBus& System::getSpiBus()
{
	return _spiBus;
}

hw::I2C& System::getI2C()
{
	return _i2c;
}

hw::HalImu& System::getImu()
{
	return _imu;
}

hw::HalMagnetometer& System::getCompass()
{
	return _compass;
}

hw::HalBarometer& System::getBarometer()
{
	return _baro;
}

mavlink::ParameterMgt& System::getParameterMgt()
{
	return _paramMgt;
}

/** @brief Getter method for radio service */
hw::Radio& System::getRadio()
{
	return _radio;
}

/** @Brief Getter for current mode */
System::Mode System::getMode()
{
	return _mode;
}

/** @brief Getter method for dynamics */
inline fdir::FdirManager& System::getFdir()
{
	return _fdir;
}

extern System system;


} /* namespace system */

#endif /* SYSTEM_HPP_ */
