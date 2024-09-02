#include "uart_parser.h"
#include "ring_buffer.h"
#include <string.h>

void parse_line(RingBuffer_t *Buf, uint8_t *Dest) {
  uint8_t Tmp;
  uint8_t i = 0;

  do {
    RB_Read(Buf, &Tmp);

    if (Tmp == ENDLINE) {
      Dest[i] = 0; // Change '\n' to c-string end sign
    } else {
      Dest[i] = Tmp;
    }
    i++;
  } while (Tmp != ENDLINE);
}

void parse_eth_data(uint8_t *DataToParse, struct eth_data *OutputData) {

  if (strcmp((char *)&DataToParse, "Compare") != 0) {
    char *ParsePointer = strtok((char *)DataToParse, ",");

    OutputData->counter = atoi(ParsePointer);

    ParsePointer = strtok(NULL, ",");
    OutputData->random_data = atoi(ParsePointer);

    // printf("Counter: %d ", OutputData->counter);
    // printf("Random_data: %d", OutputData->random_data);
  }
}
