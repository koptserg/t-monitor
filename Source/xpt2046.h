#ifndef XPT2046_H
#define XPT2046_H

#include "bme280spi.h"
#include "tft3in5.h"
#include "lcdgui.h"

#define APP_TFT_RTP_IRQ_EVT               0x0001

#define TP_PRESS_DOWN           0x80
#define TP_PRESSED              0x40

extern bool tp_mode;
	
//Touch screen structure
typedef struct {
	POINT Xpoint0;
	POINT Ypoint0;
	POINT Xpoint;
	POINT Ypoint;
	unsigned char chStatus;
	unsigned char chType;
	int iXoff;
	int iYoff;
	float fXfac;
	float fYfac;
	//Select the coordinates of the XPT2046 touch
	//  screen relative to what scan direction
	LCD_SCAN_DIR TP_Scan_Dir;
}TP_DEV;

//Brush structure
typedef struct{
	POINT Xpoint;
	POINT Ypoint;
	COLOR Color;
	DOT_PIXEL DotPixel; 
}TP_DRAW;

//extern uint8 TP_Scan(uint8 chCoordType);

extern void TP_GetAdFac(void);
extern void TP_GetAdFac2(void);

extern void TP_DrawBoard(void);
extern void TP_Init( LCD_SCAN_DIR Lcd_ScanDir );

extern void xpt2046_Init(uint8 task_id);
extern uint16 xpt2046_event_loop(uint8 task_id, uint16 events);
extern void xpt2046_HandleKeys(uint8 portAndAction, uint8 keyCode);

extern void TP_Adjust(void);

#endif /* XPT2046_H */

/* END OF FILE */