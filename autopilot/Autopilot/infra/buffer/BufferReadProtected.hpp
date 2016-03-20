/*
 * BufferReadProtected.hpp
 *
 *  Created on: 26 févr. 2016
 *      Author: AdministrateurLocal
 */

#ifndef INFRA_BUFFER_BUFFERREADPROTECTED_HPP_
#define INFRA_BUFFER_BUFFERREADPROTECTED_HPP_

#include "Buffer.hpp"

namespace infra {

class BufferReadProtected : public Buffer{
public:
	BufferReadProtected(uint8_t *buffer, size_t len);
	virtual ~BufferReadProtected();

	/** @brief Get number of elements in the buffer */
	virtual size_t available();

protected:
	virtual void getIdxTailProtected(uint16_t& idxTail);
	virtual void setIdxTailProtected(const uint16_t& idxTail);
	virtual void incrIdxTailProtected();
	virtual void incrIdxTailProtected(const uint16_t& len);
};

} /* namespace infra */

#endif /* INFRA_BUFFER_BUFFERREADPROTECTED_HPP_ */
