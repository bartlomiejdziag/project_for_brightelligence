#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#define RING_BUFFER_SIZE 64

typedef enum { RB_OK, RB_ERROR } RB_Status;

typedef struct {
  uint16_t Head;
  uint16_t Tail;
  uint8_t Buffer[RING_BUFFER_SIZE];
} RingBuffer_t;

RB_Status RB_Write(RingBuffer_t *Buf, uint8_t Value);
RB_Status RB_Read(RingBuffer_t *Buf, uint8_t *Value);
void RB_Flush(RingBuffer_t *Buf);

#endif // !RING_BUFFER_H
