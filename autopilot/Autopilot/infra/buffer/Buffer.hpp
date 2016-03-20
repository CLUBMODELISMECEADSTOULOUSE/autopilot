/*
 * Buffer.hpp
 *
 *  Created on: 7 oct. 2014
 *      Author: Aberzen
 */

#ifndef BUFFER_HPP_
#define BUFFER_HPP_

#include <stdint.h>
#include <inttypes.h>
#include <stddef.h>

namespace infra {

class Buffer {
public:
	Buffer(uint8_t *buffer, size_t len);
	virtual ~Buffer();

	/** @brief Read a buffer */
	virtual size_t read(uint8_t *value, size_t len);

	/** @brief Read one byte */
	virtual size_t read(uint8_t *value);

	/** @brief Write a buffer */
	virtual size_t write(const uint8_t *value, size_t len);

	/** @brief Write one byte */
	virtual size_t write(uint8_t value);

	/** @brief Get number of elements in the buffer */
	virtual size_t available();

	/** @brief Get number of free space in the buffer */
	virtual size_t freeSpace();

	/** @brief Discard bytes */
	virtual size_t discard(size_t len);

	/** @brief Reset the buffer (free all space) */
	virtual void reset();

protected:
	virtual void getIdxHeadProtected(uint16_t& idxHead);
	virtual void getIdxTailProtected(uint16_t& idxTail);
	virtual void setIdxHeadProtected(const uint16_t& idxHead);
	virtual void setIdxTailProtected(const uint16_t& idxTail);
	virtual void incrIdxHeadProtected();
	virtual void incrIdxTailProtected();
	virtual void incrIdxHeadProtected(const uint16_t& len);
	virtual void incrIdxTailProtected(const uint16_t& len);
//protected:
public:
	/** @brief buffer */
	uint8_t *_buffer;

	/** @brief Index start */
	uint16_t _mask;

	/** @brief Index start */
	uint16_t _idxHead;

	/** @brief Index start */
	uint16_t _idxTail;

};

} /* namespace infra */

#endif /* BUFFER_HPP_ */
