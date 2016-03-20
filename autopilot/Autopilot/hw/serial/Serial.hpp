/*
 * Serial.hpp
 *
 *  Created on: 8 mai 2015
 *      Author: Aberzen
 */

#ifndef SERIAL_HPP_
#define SERIAL_HPP_

#include <infra/buffer/BufferReadProtected.hpp>
#include <infra/buffer/BufferWriteProtected.hpp>
#include <hw/io/InputStream.hpp>
#include <hw/io/OutputStream.hpp>

#ifndef SERIAL_COM_NB
#define SERIAL_COM_NB (4)
#endif

namespace hw {

class Serial : public InputStream, OutputStream {
public:
	typedef enum {
		E_BAUDRATE_NONE   =      0,
		E_BAUDRATE_1200   =   1200,
		E_BAUDRATE_2400   =   2400,
		E_BAUDRATE_4800   =   4800,
		E_BAUDRATE_9600   =   9600,
		E_BAUDRATE_19200  =  19200,
		E_BAUDRATE_38400  =  38400,
		E_BAUDRATE_57600  =  57600,
		E_BAUDRATE_111100 = 111100,
		E_BAUDRATE_115200 = 115200,
	} E_BAUDRATE;
public:
	/* Serial */
	static Serial* serials[SERIAL_COM_NB];

public:
	Serial(
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
			infra::BufferWriteProtected& buffTx);
	virtual ~Serial();

	virtual bool open(uint32_t baudrate);

	/** Returns an estimate of the number of bytes that can be read (or skipped over)
	 * from this input stream without blocking by the next invocation of a method for this input stream. */
	virtual size_t available();

	/** Closes this input stream and releases any system resources associated with the
	 * stream. */
	virtual void close();

	/** Marks the current position in this input stream. */
	virtual void mark(size_t readlimit);

	/** Tests if this input stream supports the mark and reset methods. */
	virtual bool markSupported();

	/** Reads the next byte of data from the input stream (0xFFFF for none). */
	virtual uint16_t read();

	/** Reads some number of bytes from the input stream and stores them into the buffer array b. */
	virtual size_t read(uint8_t *b, size_t len);

	/** Reads up to len bytes of data from the input stream into an array of bytes. */
	virtual size_t read(uint8_t *b, size_t off, size_t len);

	/** Repositions this stream to the position at the time the mark method was last called on this input stream. */
	virtual void reset();

	/** Skips over and discards n bytes of data from this input stream. */
	virtual size_t skip(size_t n);

	/** Get Tx freespace. */
	virtual size_t freespace();

	/** Flushes this output stream and forces any buffered output bytes to be written out. */
	virtual void flush();

	/** Writes b.length bytes from the specified byte array to this output stream. */
	virtual size_t write(const uint8_t *b, size_t len);

	/** Writes len bytes from the specified byte array starting at offset off to this output stream. */
	virtual size_t write(const uint8_t *b, size_t off, size_t len);

	/** Writes the specified byte to this output stream. */
	virtual size_t write(uint8_t b);

	/** Interrupt handler Rx */
	virtual inline void handleRx();

	/** Interrupt handler Tx */
	virtual inline void handleTx();

//protected:

	// Baudrate
	uint32_t _baudrate;

	// register accessors
	volatile uint8_t * _udr;
	volatile uint8_t * const _ubrrh;
	volatile uint8_t * const _ubrrl;
	volatile uint8_t * const _ucsra;
	volatile uint8_t * const _ucsrb;

	// register magic numbers
	const uint8_t	_u2x;
	const uint8_t	_portEnableBits;		///< rx, tx and rx interrupt enables
	const uint8_t	_portTxBits;			///< tx data and completion interrupt enables

	// Rx/Tx buffers
	infra::BufferReadProtected& _buffRx;
	infra::BufferWriteProtected& _buffTx;

	// Status
	bool _open;
};


/** Interrupt handler Rx */
void Serial::handleRx()
{
    /* read the byte as quickly as possible and add it to buffer */
	_buffRx.write(*_udr);
}

/** Interrupt handler Tx */
void Serial::handleTx()
{
    /* read the byte as quickly as possible and add it to buffer */
	size_t len = _buffTx.read((uint8_t *) _udr);

	/* If nothing to send, disable the interrupt */
	if (!len)
	{
		*_ucsrb &= ~_portTxBits;
	}
}

} /* namespace hw */

/// Generic Rx/Tx vectors for a serial port - needs to know magic numbers
///
#define SerialHandler(_num)                     						\
ISR(USART##_num##_RX_vect, ISR_BLOCK)                                   \
{                                                                       \
	hw::Serial::serials[_num]->handleRx();                              \
}                                                                       \
ISR(USART##_num##_UDRE_vect, ISR_BLOCK)                                 \
{                                                                       \
	hw::Serial::serials[_num]->handleTx();                              \
}

#define SerialPortConstParam(_name, _num, _buffRx, _buffTx)             		\
		_name (_num,                                              			\
		 &UDR##_num,                                    				\
		 &UBRR##_num##H,                                				\
		 &UBRR##_num##L,                                				\
		 &UCSR##_num##A,                                				\
		 &UCSR##_num##B,                                				\
		 U2X##_num,                                     				\
		 (_BV(RXEN##_num) |  _BV(TXEN##_num) | _BV(RXCIE##_num)), 		\
		 (_BV(UDRIE##_num)),                            				\
		 _buffRx,                                       				\
		 _buffTx)
///
/// Macro defining a FastSerial port instance.
///
#define SerialPort(_name, _num, _buffRx, _buffTx)                       \
	hw::Serial _name(_num,                                              \
                         &UDR##_num,                                    \
                         &UBRR##_num##H,                                \
                         &UBRR##_num##L,                                \
                         &UCSR##_num##A,                                \
                         &UCSR##_num##B,                                \
                         U2X##_num,                                     \
                         (_BV(RXEN##_num) |  _BV(TXEN##_num) | _BV(RXCIE##_num)), \
                         (_BV(UDRIE##_num)),                            \
                         _buffRx,                                       \
                         _buffTx);                                      \
	SerialHandler(_num)


#endif /* SERIAL_HPP_ */
