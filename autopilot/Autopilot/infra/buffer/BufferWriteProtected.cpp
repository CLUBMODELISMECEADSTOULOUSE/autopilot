/*
 * BufferWriteProtected.cpp
 *
 *  Created on: 26 févr. 2016
 *      Author: AdministrateurLocal
 */

#include <infra/buffer/BufferWriteProtected.hpp>
#include "Buffer.hpp"
#include <infra/rtos/Task.hpp>

namespace infra {

BufferWriteProtected::BufferWriteProtected(uint8_t *buffer, size_t len)
: Buffer(buffer, len)
{
}

BufferWriteProtected::~BufferWriteProtected() {
	// TODO Auto-generated destructor stub
}

/** @brief Get number of free space in the buffer */
size_t BufferWriteProtected::freeSpace() {
	size_t avail = 0;
	infra::Task::disableInterrupt();
	avail = Buffer::freeSpace();
	infra::Task::enableInterrupt();
	return avail;
}
void BufferWriteProtected::getIdxHeadProtected(uint16_t& idxHead)
{
	infra::Task::disableInterrupt();
	Buffer::getIdxHeadProtected(idxHead);
	infra::Task::enableInterrupt();
}
void BufferWriteProtected::setIdxHeadProtected(const uint16_t& idxHead)
{
	infra::Task::disableInterrupt();
	Buffer::setIdxHeadProtected(idxHead);
	infra::Task::enableInterrupt();
}
void BufferWriteProtected::incrIdxHeadProtected()
{
	infra::Task::disableInterrupt();
	Buffer::incrIdxHeadProtected();
	infra::Task::enableInterrupt();
}
void BufferWriteProtected::incrIdxHeadProtected(const uint16_t& len)
{
	infra::Task::disableInterrupt();
	Buffer::incrIdxHeadProtected(len);
	infra::Task::enableInterrupt();
}

} /* namespace infra */
