/*
 * ringbuffer.h
 *
 *  Created on: May 15, 2019
 *      Author: Jeffrey
 */

#ifndef FIFOBUFFER_H_
#define FIFOBUFFER_H_

#define FIFO_AVL(rdIdx,wrtIdx) (rdIdx != wrtIdx)
#define FIFO_BUFFER_ADVANCE(idx,size) ((idx + 1) % size)
#define FIFO_BUFFER_FULL(rdIdx,wrtIdx,size) (rdIdx == FIFO_BUFFER_ADVANCE(wrtIdx,size))
#define FIFO_BUFFER_EMPTY(rdIdx,wrtIdx) (rdIdx == wrtIdx)

#endif /* FIFOBUFFER_H_ */
