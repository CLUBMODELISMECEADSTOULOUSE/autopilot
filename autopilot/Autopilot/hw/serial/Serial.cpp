/*
 * Serial.cpp
 *
 *  Created on: 8 mai 2015
 *      Author: Aberzen
 */

#include "Serial.hpp"

namespace hw {

Serial* Serial::serials[SERIAL_COM_NB];

Serial::Serial(
		const uint8_t portNumber,
		volatile uint8_t *udr,
		volatile uint8_t *ubrrh,
		volatile uint8_t *ubrrl,
		volatile uint8_t *ucsra,
		volatile uint8_t *ucsrb,
		const uint8_t u2x,
		const uint8_t portEnableBits,
		const uint8_t portTxBits,
		infra::BufferReadProtected& buffRx,
		infra::BufferWriteProtected& buffTx)
: InputStream(),
  OutputStream(),
  _baudrate(0),
  _udr(udr),
  _ubrrh(ubrrh),
  _ubrrl(ubrrl),
  _ucsra(ucsra),
  _ucsrb(ucsrb),
  _u2x(u2x),
  _portEnableBits(portEnableBits),
  _portTxBits(portTxBits),
  _buffRx(buffRx),
  _buffTx(buffTx),
  _open(false)
{
	serials[portNumber] = this;
}

Serial::~Serial()
{
	close();
}


bool Serial::open(uint32_t baudrate)
{
	bool use_u2x = true;
	uint16_t ubrr = 0;

	// If the user has supplied a new baud rate, compute the new UBRR value.
	if (baudrate > 0) {
#if F_CPU == 16000000UL
		// hardcoded exception for compatibility with the bootloader shipped
		// with the Duemilanove and previous boards and the firmware on the 8U2
		// on the Uno and Mega 2560.
		if (baudrate == 57600)
			use_u2x = false;
#endif

		if (use_u2x) {
			*_ucsra = 1 << _u2x;
			ubrr = (F_CPU / 4 / baudrate - 1) / 2;
		} else {
			*_ucsra = 0;
			ubrr = (F_CPU / 8 / baudrate - 1) / 2;
		}

		*_ubrrh = (ubrr >> 8) & 0xFF;
		*_ubrrl = (ubrr & 0xFF);
	}

	*_ucsrb |= _portEnableBits;

	_open = true;
	return _open;
}

/** Returns an estimate of the number of bytes that can be read (or skipped over)
 * from this input stream without blocking by the next invocation of a method for this input stream. */
size_t Serial::available()
{
	return _buffRx.available();
}

/** Closes this input stream and releases any system resources associated with the
 * stream. */
void Serial::close()
{
	*_ucsrb &= ~(_portEnableBits | _portTxBits);

	_buffRx.reset();
	_buffTx.reset();
	_open = false;

}

/** Marks the current position in this input stream. */
void Serial::mark(size_t readlimit)
{

}

/** Tests if this input stream supports the mark and reset methods. */
bool Serial::markSupported()
{
	return false;
}

/** Reads the next byte of data from the input stream (0xFFFF for none). */
uint16_t Serial::read()
{
	uint8_t c;
	uint16_t result = (uint16_t) -1;
	if (_buffRx.read(&c))
		result = (uint16_t)(c);
	return result;
}

/** Reads some number of bytes from the input stream and stores them into the buffer array b. */
size_t Serial::read(uint8_t *b, size_t len)
{
	return _buffRx.read(b,len);
}

/** Reads up to len bytes of data from the input stream into an array of bytes. */
size_t Serial::read(uint8_t *b, size_t off, size_t len)
{
	return _buffRx.read(&b[off],len);
}

/** Repositions this stream to the position at the time the mark method was last called on this input stream. */
void Serial::reset()
{
	_buffRx.reset();
}

/** Skips over and discards n bytes of data from this input stream. */
size_t Serial::skip(size_t n)
{
	return _buffRx.discard(n);
}

/** Flushes this output stream and forces any buffered output bytes to be written out. */
void Serial::flush()
{
	while(_buffTx.available()){};
}

/** Get Tx freespace. */
size_t Serial::freespace()
{
	return _buffTx.freeSpace();
}


/** Writes b.length bytes from the specified byte array to this output stream. */
size_t Serial::write(const uint8_t *b, size_t len)
{
	size_t result = _buffTx.write(b,len);

	// enable the data-ready interrupt, as it may be off if the buffer is empty
	*_ucsrb |= _portTxBits;

	return result;
}

/** Writes len bytes from the specified byte array starting at offset off to this output stream. */
size_t Serial::write(const uint8_t *b, size_t off, size_t len)
{
	size_t result = _buffTx.write(&b[off],len);

	// enable the data-ready interrupt, as it may be off if the buffer is empty
	*_ucsrb |= _portTxBits;

	return result;
}

/** Writes the specified byte to this output stream. */
size_t Serial::write(uint8_t b)
{
	size_t result = _buffTx.write(b);

	// enable the data-ready interrupt, as it may be off if the buffer is empty
	*_ucsrb |= _portTxBits;

	return result;
}

} /* namespace hw */
