/*
 * BufferReadProtected.cpp
 *
 *  Created on: 26 févr. 2016
 *      Author: AdministrateurLocal
 */

#include <infra/buffer/BufferReadProtected.hpp>
#include "Buffer.hpp"
#include <infra/rtos/Task.hpp>

namespace infra {

BufferReadProtected::BufferReadProtected(uint8_t *buffer, size_t len)
: Buffer(buffer, len)
{
}

BufferReadProtected::~BufferReadProtected() {
	// TODO Auto-generated destructor stub
}

/** @brief Get number of elements in the buffer */
size_t BufferReadProtected::available() {
	size_t avail = 0;
	infra::Task::disableInterrupt();
	avail = Buffer::available();
	infra::Task::enableInterrupt();
	return avail;
}


void BufferReadProtected::getIdxTailProtected(uint16_t& idxTail)
{
	infra::Task::disableInterrupt();
	Buffer::getIdxTailProtected(idxTail);
	infra::Task::enableInterrupt();
}
void BufferReadProtected::setIdxTailProtected(const uint16_t& idxTail)
{
	infra::Task::disableInterrupt();
	Buffer::setIdxTailProtected(idxTail);
	infra::Task::enableInterrupt();
}
void BufferReadProtected::incrIdxTailProtected()
{
	infra::Task::disableInterrupt();
	Buffer::incrIdxTailProtected();
	infra::Task::enableInterrupt();
}
void BufferReadProtected::incrIdxTailProtected(const uint16_t& len)
{
	infra::Task::disableInterrupt();
	Buffer::incrIdxTailProtected(len);
	infra::Task::enableInterrupt();
}

} /* namespace infra */
