#ifndef FT6236_H
#define FT6236_H

#include "bme280spi.h"
#include "tft3in5.h"
#include "lcdgui.h"

#define APP_TFT_CTP_IRQ_EVT               0x0001

//#define FT62XX_ADDR 0x38           //!< I2C address
#define FT62XX_ADDR_READ  (0x38 << 1) | 0x01
#define FT62XX_ADDR_WRITE (0x38 << 1) | 0x00

// calibrated for Adafruit 2.8" ctp screen
#define FT62XX_DEFAULT_THRESHOLD 128 //!< Default threshold for touch detection

extern bool tp_mode;

//Touch screen structure
typedef struct {
	//Select the coordinates of the FT6236 touch
	//  screen relative to what scan direction
	LCD_SCAN_DIR TP_Scan_Dir;
}CTP_DEV;

//Brush structure
typedef struct{
	POINT Xpoint;
	POINT Ypoint;
	COLOR Color;
	DOT_PIXEL DotPixel; 
}CTP_DRAW;

extern void ft6236_Init(uint8 task_id);
extern uint16 ft6236_event_loop(uint8 task_id, uint16 events);
extern void ft6236_HandleKeys(uint8 portAndAction, uint8 keyCode);
extern uint8 ft6236_readRegister8(uint8_t reg);

#endif /* FT6236_H */

/* END OF FILE */