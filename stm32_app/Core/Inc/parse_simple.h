#ifndef INC_PARSE_SIMPLE_H_
#define INC_PARSE_SIMPLE_H_

#define ENDLINE '\n'

void Parser_TakeLine(RingBuffer_t *Buf, uint8_t *Destination);
void Parser_Parse(uint8_t *DataToParse);

#endif /* INC_PARSE_SIMPLE_H_ */
