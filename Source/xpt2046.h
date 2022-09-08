#ifndef XPT2046_H
#define XPT2046_H

#include "bme280spi.h"
#include "tft3in5.h"
#include "lcdgui.h"

#define TP_PRESS_DOWN           0x80
#define TP_PRESSED              0x40
	
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

void TP_GetAdFac(void);
void TP_Adjust(void);
void TP_Dialog(void);
void TP_DrawBoard(void);
void TP_Init( LCD_SCAN_DIR Lcd_ScanDir );

void DelayUs(uint16 microSecs);

#endif /* XPT2046_H */

/* END OF FILE */