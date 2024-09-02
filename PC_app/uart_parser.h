#ifndef UART_PARSER_H
#define UART_PARSER_H

#include "ring_buffer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define ENDLINE '\n'

struct eth_data {
  uint32_t counter;
  uint32_t random_data;
};

void parse_line(RingBuffer_t *Buf, uint8_t *Dest);
void parse_eth_data(uint8_t *DataToParse, struct eth_data *OutputData);

#endif // !UART_PARSER_H
