#include "main.h"
#include "ring_buffer.h"
#include "parse_simple.h"
#include "string.h"
#include "../../Drivers/BSP/stm32746g_discovery_lcd.h"

//
// Get a one complete line from Ring Buffer
//
void Parser_TakeLine(RingBuffer_t *Buf, uint8_t *Destination)
{
	uint8_t Tmp;
	uint8_t i = 0;

	// Loop for every char in Ring Buffer
	do
	{
		// Read a one byte from Ring Buffer
		RB_Read(Buf, &Tmp);

		// Check if we take the endline char
		if(Tmp == ENDLINE)
		{
			// If endline - replace it with c-string end sign - 0x00
			Destination[i] = 0;
		}
		else
		{
			// If not endline - just write to work-buffer
			Destination[i] = Tmp;
		}

		i++; // increment array iterator
	}while(Tmp != ENDLINE); // If we hit the endline char - end the loop
}

// Parsing function
// Commands to detect:
// 	LED_ON
// 	LED_OFF
//
// uint8_t *DataToParse - an array with complete line taken from Ring Buffer earlier
//
void Parser_Parse(uint8_t *DataToParse)
{
	// Compare provided array with line to parse with command template
	if(strcmp("MATCH", (char*)DataToParse) == 0)
	{
		BSP_LCD_Clear(LCD_COLOR_WHITE);
		BSP_LCD_DisplayStringAt(0, 136, (uint8_t*) "MATCH", CENTER_MODE);
	} else if(strcmp("MISMATCH", (char*)DataToParse) == 0) {
		BSP_LCD_Clear(LCD_COLOR_WHITE);
		BSP_LCD_DisplayStringAt(0, 136, (uint8_t*) "MISMATCH", CENTER_MODE);
	}
}

