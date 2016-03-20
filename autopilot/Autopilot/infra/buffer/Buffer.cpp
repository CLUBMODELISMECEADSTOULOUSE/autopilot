/*
 * Buffer.cpp
 *
 *  Created on: 7 oct. 2014
 *      Author: Aberzen
 */

#include <string.h>

#include "Buffer.hpp"
#include <math/MathMacro.hpp>


namespace infra {

Buffer::Buffer(uint8_t *buffer, size_t len)
: _buffer(buffer),
  _mask(0x0000),
  _idxHead(0),
  _idxTail(0)
{
	uint16_t power = 0;
	uint16_t tmp = len;

	/* Find the power of two 2^n greater than length */
	while (tmp != 0)
	{
		tmp >>= 1;
		power ++;
	}
	/* Compute mask using this power of two as
	 * the power of two directly smaller minus one
	 * mask = (2^n) - 1
	 */
	if (power != 0)
		_mask = ((1 << (power-1)) - 1);
}

Buffer::~Buffer() {
}


/** @brief Read a buffer */
size_t Buffer::read(uint8_t *value, size_t len) {
	/* _idxTail is not protected when reading
	 * since assuming a unique reader and only readers
	 * modify its value */

	/* Bound elements to available elements */
	size_t nRead = math_min(available(), len);

	/* Copy data from current pointer to end of buffer */
	size_t remaining = (_mask + 1) - _idxTail;
	size_t lenFirst = math_min(remaining, nRead);
	memcpy(&value[0], &_buffer[_idxTail], lenFirst);

	/* Continue if necessary starting from beginning of the buffer */
	if (nRead > lenFirst)
	{
		memcpy(&value[lenFirst], &_buffer[0], (nRead-lenFirst));
	}

	/* Increment the index */
	incrIdxTailProtected(nRead);

	/* Return number of read bytes */
	return nRead;
}

/** @brief Read one char */
size_t Buffer::read(uint8_t *value) {
	/* _idxTail is not protected when reading
	 * since assuming a unique reader and only readers
	 * modify its value */

	/* Check available bytes */
	size_t nRead = math_min(available(),1);

	if (nRead != 0)
	{
		/* Copy data */
		*value = _buffer[_idxTail];
		incrIdxTailProtected();
	}
	return nRead;
}

/** @brief Write a buffer */
size_t Buffer::write(const uint8_t *value, size_t len) {
	/* _idxHead is not protected when writing
	 * since assuming a unique writer and only writers
	 * modify its value */

	/* Bound elements to available elements */
	size_t nWrite = math_min(freeSpace(), len);

	/* Copy data from current pointer to end of buffer */
	size_t remaining = (_mask + 1) - _idxHead;
	size_t lenFirst = math_min(remaining, nWrite);
	memcpy(&_buffer[_idxHead], &value[0], lenFirst);

	/* Continue if necessary starting from beginning of the buffer */
	if (nWrite > lenFirst)
	{
		memcpy(&_buffer[0], &value[lenFirst], (nWrite-lenFirst));
	}

	/* Increment the index */
	incrIdxHeadProtected(nWrite);

	return nWrite;
}

/** @brief Write one char */
size_t Buffer::write(uint8_t value) {
	/* _idxHead is not protected when writing
	 * since assuming a unique writer and only writers
	 * modify its value */

	/* Get free space */
	size_t nWrite = math_min(freeSpace(), 1);

	/* If one byte can be written, write the byte */
	if (nWrite != 0)
	{
		/* Copy data */
		_buffer[_idxHead] = value;
		incrIdxHeadProtected();
	}
	return nWrite;
}
/** @brief Get number of elements in the buffer */
size_t Buffer::available() {
	size_t avail = 0;
	avail = ((_idxHead - _idxTail) & _mask);
	return avail;
}


/** @brief Get number of free space in the buffer */
size_t Buffer::freeSpace() {
	size_t avail = 0;
	avail = ((_idxTail - 1 - _idxHead) & _mask);
	return avail;
}


/** @brief Discard bytes */
size_t Buffer::discard(size_t len) {
	/* Bound elements to available elements */
	size_t nDiscarded = math_min(available(), len);

	/* Remove elements */
	incrIdxTailProtected(nDiscarded);

	return nDiscarded;
}


/** @brief Reset the buffer (free all space) */
void Buffer::reset() {
	_idxHead = 0;
	_idxTail = 0;
}

void Buffer::getIdxHeadProtected(uint16_t& idxHead)
{
	idxHead = _idxHead;
}
void Buffer::getIdxTailProtected(uint16_t& idxTail)
{
	idxTail = _idxTail;
}
void Buffer::setIdxHeadProtected(const uint16_t& idxHead)
{
	_idxHead = idxHead;
}
void Buffer::setIdxTailProtected(const uint16_t& idxTail)
{
	_idxTail = idxTail;
}
void Buffer::incrIdxHeadProtected()
{
	_idxHead = ((_idxHead + 1) & _mask);
}
void Buffer::incrIdxTailProtected()
{
	_idxTail = ((_idxTail + 1) & _mask);
}
void Buffer::incrIdxHeadProtected(const uint16_t& len)
{
	_idxHead = ((_idxHead + len) & _mask);
}
void Buffer::incrIdxTailProtected(const uint16_t& len)
{
	_idxTail = ((_idxTail + len) & _mask);
}

} /* namespace infra */
