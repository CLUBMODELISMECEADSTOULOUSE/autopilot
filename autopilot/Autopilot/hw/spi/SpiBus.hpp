/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <stddef.h>
#include <stdio.h>
#include <avr/pgmspace.h>

#include <FreeRTOS.h>
#include <Arduino.h>
#include <semphr.h>
#include <hw/common/Driver.hpp>

namespace hw {


#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV32 0x06
//#define SPI_CLOCK_DIV64 0x07

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

#define SPI_MODE_MASK 0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01  // SPI2X = bit 0 on SPSR

class SpiBus : public Driver {
public:
	SpiBus();
	virtual ~SpiBus();

	/** @brief Interrupt. */
	virtual inline void attachInterrupt();

	/** @brief Clear interrupt. */
	virtual inline void detachInterrupt(); // Default

	/** @brief Initialize the SpiBus */
	virtual bool initialize() ;

	/** @brief Stop the bus. */
	virtual void reset();

	/** @brief Select endianess. */
	virtual void setBitOrder(uint8_t);

	/** @brief Select data mode. */
	virtual void setDataMode(uint8_t);

	/** @brief Set clock divider. */
	virtual void setClockDivider(uint8_t);

	/** @brief Select the bus to access the slave. */
	virtual inline void select(uint8_t id);

	/** @brief Release the bus. */
	virtual inline void release(uint8_t id);

	/** @brief Read any size. */
	void read(uint8_t id, uint8_t addr, uint8_t cnt, uint8_t* dataOut);

	/** @brief Read n times the same address. */
	void readn(uint8_t id, uint8_t addr, uint8_t cnt, uint8_t* dataOut);

	/** @brief read any size. */
	void read(uint8_t id, uint8_t addr, uint8_t cnt, uint16_t* dataOut);

	/** @brief Write / transfer any size. */
	void transfer(uint8_t id, uint8_t* dataIn, uint8_t cnt, uint8_t* dataOut = NULL);

	/** @brief Read 8bits. */
	void transfer0(uint8_t id, uint8_t dataIn);

	/** @brief Read 8bits. */
	void transfer8(uint8_t id, uint8_t addr, uint8_t& dataOut, bool discardFirst=true);

	/** @brief Read 16bits. */
	void transfer16(uint8_t id, uint8_t addr, uint16_t& dataOut, bool discardFirst=true);

	/** @brief Read 24bits. */
	void transfer24(uint8_t id, uint8_t addr, uint32_t& dataOut, bool discardFirst=true);

	/** @brief Read 32bits. */
	void transfer32(uint8_t id, uint8_t addr, uint32_t& dataOut, bool discardFirst=true);

protected:

	/** @brief Data transfer. */
	virtual inline uint8_t transfer(uint8_t data);

};

void SpiBus::attachInterrupt() {
  SPCR |= _BV(SPIE);
}

void SpiBus::detachInterrupt() {
  SPCR &= ~_BV(SPIE);
}

uint8_t SpiBus::transfer(uint8_t _data) {
  SPDR = _data;
  while (!(SPSR & _BV(SPIF)))
    ;
  return SPDR;
}

/** @brief Select the bus to access the slave. */
void SpiBus::select(uint8_t id)
{
	/* Activate the device */
	digitalWrite(id, LOW);
}

/** @brief Release the bus. */
void SpiBus::release(uint8_t id)
{
	/* Set SS to deselect the slave */
	digitalWrite(id, HIGH);
}


} /* namespace hw */

#endif
