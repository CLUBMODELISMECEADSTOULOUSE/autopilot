/*
 * BufferWriteProtected.hpp
 *
 *  Created on: 26 févr. 2016
 *      Author: AdministrateurLocal
 */

#ifndef INFRA_BUFFER_BUFFERWRITEPROTECTED_HPP_
#define INFRA_BUFFER_BUFFERWRITEPROTECTED_HPP_

#include "Buffer.hpp"

namespace infra {

class BufferWriteProtected  : public Buffer{
public:
	BufferWriteProtected(uint8_t *buffer, size_t len);
	virtual ~BufferWriteProtected();

	/** @brief Get number of free space in the buffer */
	virtual size_t freeSpace();


protected:
	virtual void getIdxHeadProtected(uint16_t& idxHead);
	virtual void setIdxHeadProtected(const uint16_t& idxHead);
	virtual void incrIdxHeadProtected();
	virtual void incrIdxHeadProtected(const uint16_t& len);

};

} /* namespace infra */

#endif /* INFRA_BUFFER_BUFFERWRITEPROTECTED_HPP_ */
