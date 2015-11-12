/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */


#include "SpiBus.hpp"
#include <Arduino.h>

namespace hw {

SpiBus::SpiBus(){
	/* Set SS to high so a connected chip will be "deselected" by default */
	digitalWrite(SS, HIGH);

	/* When the SS pin is set as OUTPUT, it can be used as
	 * a general purpose output port (it doesn't influence
	 * SPI operations). */
	pinMode(SS, OUTPUT);
}

SpiBus::~SpiBus() {

}

bool SpiBus::initialize() {

  // Set SS to high so a connected chip will be "deselected" by default
  digitalWrite(SS, HIGH);

  // When the SS pin is set as OUTPUT, it can be used as
  // a general purpose output port (it doesn't influence
  // SPI operations).
  pinMode(SS, OUTPUT);

  // Warning: if the SS pin ever becomes a LOW INPUT then SPI
  // automatically switches to Slave, so the data direction of
  // the SS pin MUST be kept as OUTPUT.
  SPCR |= _BV(MSTR);
  SPCR |= _BV(SPE);

  // Set direction register for SCK and MOSI pin.
  // MISO pin automatically overrides to INPUT.
  // By doing this AFTER enabling SPI, we avoid accidentally
  // clocking in a single bit since the lines go directly
  // from "input" to SPI control.  
  // http://code.google.com/p/arduino/issues/detail?id=888
  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);

  return Driver::initialize();
}


void SpiBus::reset()
{
  SPCR &= ~_BV(SPE);
  Driver::reset();
}

void SpiBus::setBitOrder(uint8_t bitOrder)
{
  if(bitOrder == LSBFIRST) {
    SPCR |= _BV(DORD);
  } else {
    SPCR &= ~(_BV(DORD));
  }
}

void SpiBus::setDataMode(uint8_t mode)
{
  SPCR = (SPCR & ~SPI_MODE_MASK) | mode;
}

void SpiBus::setClockDivider(uint8_t rate)
{
  SPCR = (SPCR & ~SPI_CLOCK_MASK) | (rate & SPI_CLOCK_MASK);
  SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((rate >> 2) & SPI_2XCLOCK_MASK);
}

/** @brief Write / transfer any size. */
void SpiBus::read(uint8_t id, uint8_t addr, uint8_t cnt, uint8_t* dataOut)
{
	uint8_t idx;
	/* Check if current slave is selected */
	/* Set SS to deselect the slave */
	digitalWrite(id, LOW);

	/* Discard */
	transfer(addr);
	for (idx=0 ; idx<cnt ; idx++)
	{
		/* copy */
		dataOut[idx] = transfer(0);
	}
	/* Set SS to deselect the slave */
	digitalWrite(id, HIGH);
}

/** @brief Write / transfer any size. */
void SpiBus::readn(uint8_t id, uint8_t addr, uint8_t cnt, uint8_t* dataOut)
{
	uint8_t idx;
	/* Check if current slave is selected */
	/* Set SS to deselect the slave */
	digitalWrite(id, LOW);

	/* Discard */
	transfer(addr);
	for (idx=cnt-1 ; idx!=0 ; idx--)
	{
		/* copy */
		dataOut[idx] = transfer(addr);
	}
	dataOut[idx] = transfer(0);
	/* Set SS to deselect the slave */
	digitalWrite(id, HIGH);
}

/** @brief Write / transfer any size. */
void SpiBus::read(uint8_t id, uint8_t addr, uint8_t cnt, uint16_t* dataOut)
{
	uint8_t idx;
	uint16_t dataL = 0, dataH = 0;

	/* Set SS to deselect the slave */
	digitalWrite(id, LOW);

	/* Discard */
	transfer(addr);
	for (idx=0 ; idx<cnt ; idx++)
	{
		dataH = transfer(0);
		dataL = transfer(0);
		dataOut[idx] = (dataH<<8) | dataL;
	}
	/* Set SS to deselect the slave */
	digitalWrite(id, HIGH);
}


/** @brief Transfer arbitrary count. */
void SpiBus::transfer(uint8_t id, uint8_t* dataIn, uint8_t cnt, uint8_t* dataOut)
{
	uint8_t idx;

	/* Set SS to select the slave */
	digitalWrite(id, LOW);

	if (dataOut == NULL)
	{
		for (idx=0 ; idx<cnt ; idx++)
			/* Discard */
			transfer(dataIn[idx]);
	}
	else
	{
		for (idx=0 ; idx<cnt ; idx++)
			/* Discard */
			dataOut[idx] = transfer(dataIn[idx]);
	}
	/* Set SS to deselect the slave */
	digitalWrite(id, HIGH);
}

/** @brief Read 8bits. */
void SpiBus::transfer0(uint8_t id, uint8_t cmd) {

	/* Set SS to select the slave */
	digitalWrite(id, LOW);
	/* Discard */
	transfer(cmd);
	/* Set SS to deselect the slave */
	digitalWrite(id, HIGH);
}

/** @brief Read 8bits. */
void SpiBus::transfer8(uint8_t id, uint8_t cmd, uint8_t& dataOut, bool discardFirst) {
	/* Set SS to select the slave */
	digitalWrite(id, LOW);
	dataOut = transfer(cmd); /* Discard */
	if (discardFirst)
		dataOut = transfer(0);
	/* Set SS to deselect the slave */
	digitalWrite(id, HIGH);
}

/** @brief Read 16bits. */
void SpiBus::transfer16(uint8_t id, uint8_t cmd, uint16_t& dataOut, bool discardFirst) {
	uint16_t dataL, dataH;

	/* Set SS to select the slave */
	digitalWrite(id, LOW);
	dataH = transfer(cmd);
	if (discardFirst)
		dataH = transfer(0);
	dataL = transfer(0);
	/* Set SS to deselect the slave */
	digitalWrite(id, HIGH);
	dataOut = (dataH<<8) | dataL;
}

/** @brief Transfer 24bits. */
void SpiBus::transfer24(uint8_t id, uint8_t cmd, uint32_t& dataOut, bool discardFirst) {
	uint32_t dataL, dataM, dataH;

	/* Set SS to select the slave */
	digitalWrite(id, LOW);
	dataH = transfer(cmd);
	if (discardFirst)
		dataH = transfer(0);
	dataM = transfer(0);
	dataL = transfer(0);
	/* Set SS to deselect the slave */
	digitalWrite(id, HIGH);
	dataOut = (dataH<<16) | (dataM<<8) | dataL;
}

/** @brief Transfer 32bits. */
void SpiBus::transfer32(uint8_t id, uint8_t cmd, uint32_t& dataOut, bool discardFirst) {
	uint8_t dataL;
	uint16_t dataML;
	uint32_t dataMH, dataH;

	/* Set SS to select the slave */
	digitalWrite(id, LOW);
	dataH = transfer(cmd);
	if (discardFirst)
		dataH = transfer(0);
	dataMH = transfer(0);
	dataML = transfer(0);
	dataL = transfer(0);
	/* Set SS to deselect the slave */
	digitalWrite(id, HIGH);
	dataOut = (dataH<<24) | (dataMH<<16) | (dataML<<8) | dataL;
}

}; /* namespace hw*/

