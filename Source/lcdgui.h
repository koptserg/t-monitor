#ifndef LCDGUI_H
#define LCDGUI_H

#include "fonts.h"
#include "tft3in5.h"

#define LOW_Speed_Show 0
#define HIGH_Speed_Show 1

#define BUTTON_COUNT_MAX  6

/********************************************************************************
function:
			dot pixel
********************************************************************************/
typedef enum {
    DOT_PIXEL_1X1  = 1,		// dot pixel 1 x 1
    DOT_PIXEL_2X2  , 		// dot pixel 2 X 2
    DOT_PIXEL_3X3  ,		// dot pixel 3 X 3
    DOT_PIXEL_4X4  ,		// dot pixel 4 X 4
    DOT_PIXEL_5X5  , 		// dot pixel 5 X 5
    DOT_PIXEL_6X6  , 		// dot pixel 6 X 6
    DOT_PIXEL_7X7  , 		// dot pixel 7 X 7
    DOT_PIXEL_8X8  , 		// dot pixel 8 X 8
} DOT_PIXEL;
#define DOT_PIXEL_DFT  DOT_PIXEL_1X1  //Default dot pilex

/********************************************************************************
function:
			dot Fill style
********************************************************************************/
typedef enum {
    DOT_FILL_AROUND  = 1,		// dot pixel 1 x 1
    DOT_FILL_RIGHTUP  , 		// dot pixel 2 X 2
} DOT_STYLE;
#define DOT_STYLE_DFT  DOT_FILL_AROUND  //Default dot pilex
/********************************************************************************
function:
			solid line and dotted line
********************************************************************************/
typedef enum {
    LINE_SOLID = 0,
    LINE_DOTTED,
} LINE_STYLE;

/********************************************************************************
function:
			DRAW Internal fill
********************************************************************************/
typedef enum {
    DRAW_EMPTY = 0,
    DRAW_FULL,
} DRAW_FILL;

/********************************************************************************
function:
	time
********************************************************************************/
typedef struct {
    uint16 Year;  //0000
    uint8  Month; //1 - 12
    uint8  Day;   //1 - 30
    uint8  Hour;  //0 - 23
    uint8  Min;   //0 - 59
    uint8  Sec;   //0 - 59
} DEV_TIME;
extern DEV_TIME sDev_time;

/********************************************************************************
function:
	button
********************************************************************************/
typedef struct {
  int16 x1;
  int16 y1;
  uint16 w;
  uint16 h;
  uint16 outlinecolor;
  uint16 fillcolor;
  uint16 textcolor;
  uint8 textsize;
  char label[9];
} BUTTON;
static BUTTON sButton[6];

/********************************************************************************
function:
			Defines commonly used colors for the display
********************************************************************************/
#define LCD_BACKGROUND		WHITE   //Default background color
#define FONT_BACKGROUND		WHITE   //Default font background color
#define FONT_FOREGROUND	    GRAY    //Default font foreground color

// Color definitions
#define  BLACK       0x0000  ///<   0,   0,   0
#define  NAVY        0x000F  ///<   0,   0, 123
#define  DARKGREEN   0x03E0  ///<   0, 125,   0
#define  DARKCYAN    0x03EF  ///<   0, 125, 123
#define  MAROON      0x7800  ///< 123,   0,   0
#define  PURPLE      0x780F  ///< 123,   0, 123
#define  OLIVE       0x7BE0  ///< 123, 125,   0
#define  LIGHTGREY   0xC618  ///< 198, 195, 198
#define  DARKGREY    0x7BEF  ///< 123, 125, 123
#define  GRAY       0x7BEF  ///< 123, 125, 123
#define  BLUE        0x001F  ///<   0,   0, 255
#define  GREEN       0x07E0  ///<   0, 255,   0
#define  LIGHTGREEN  0xAFE5
#define  CYAN        0x07FF  ///<   0, 255, 255
#define  RED         0xF800  ///< 255,   0,   0
#define  MAGENTA     0xF81F  ///< 255,   0, 255
#define  YELLOW      0xFFE0  ///< 255, 255,   0
#define  WHITE       0xFFFF  ///< 255, 255, 255
#define  ORANGE      0xFD20  ///< 255, 165,   0
#define  GREENYELLOW 0xAFE5  ///< 173, 255,  41
#define  PINK        0xFC18  ///< 255, 130, 198

#define  BLUE0        0x3A6E  ///< 62, 78, 114  
#define  BLUE1        0x116C  ///< 19, 44, 98  
#define  BLUE2        0x3AD3  ///< 59, 89, 153
#define  BLUE3        0x6B99  ///< 110, 140, 204      
#define  BLUE4        0x8CF9  ///< 138, 156, 202  
#define  BLUE5        0x5BD6  ///< 95, 120, 176 
#define  ORANG1       0xE569  ///< 228, 175, 73                 
#define  ORANG2       0xA363  ///< 161, 111, 26 

#define  RED0         0xAA8A  ///< 169, 83, 184  
#define  RED1         0x90A3  ///< 147, 22, 26  
#define  RED2         0xDA49  ///< 219, 75, 75
#define  RED3         0xF3AE  ///< 240, 117, 120      
#define  RED4         0xF4B3  ///< 240, 150, 152  
#define  RED5         0xCB2C  ///< 207, 101, 101 
#define  GREEN1       0x45A7  ///< 65, 183, 61                 
#define  GREEN2       0x1421  ///< 19, 132, 14 

#define  YELLOW0      0xAD4A  ///< 171, 169, 86  
#define  YELLOW1      0x9482  ///< 148, 146, 23  
#define  YELLOW2      0xE729  ///< 228, 228, 72
#define  YELLOW3      0xF78E  ///< 240, 240, 118      
#define  YELLOW4      0xF793  ///< 241, 241, 153  
#define  YELLOW5      0xD66D  ///< 209, 207, 104 
#define  PURPLE1      0x71B3  ///< 116, 54, 153                 
#define  PURPLE2      0x508E  ///< 80, 19, 114 

#define  GREEN_0      0x4448  ///< 71, 138, 67  
#define  GREEN_1      0x1BC2  ///< 24, 120, 20  
#define  GREEN_2      0x45A7  ///< 67, 183, 60
#define  GREEN_3      0x76CD  ///< 113, 219, 108      
#define  GREEN_4      0x8EF1  ///< 141, 220, 137  
#define  GREEN_5      0x5DEB  ///< 95, 191, 91 
#define  RED_1     0xE249  ///< 226, 72, 72                 
#define  RED_2     0xA0A2  ///< 165, 20, 23 
/********************************************************************************
function:
			Macro definition variable name
********************************************************************************/
void GUI_Swop(POINT Point1, POINT Point2);
sFONT *GUI_GetFontSize(POINT Dx, POINT Dy);

//Clear
void GUI_Clear(COLOR Color);

//Drawing
void GUI_DrawPoint(POINT Xpoint, POINT Ypoint, COLOR Color, DOT_PIXEL Dot_Pixel, DOT_STYLE Dot_FillWay);
void GUI_DrawLine(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend, COLOR Color, LINE_STYLE Line_Style, DOT_PIXEL Dot_Pixel);
void GUI_DrawRectangle(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend, COLOR Color, DRAW_FILL Filled , DOT_PIXEL Dot_Pixel );
void GUI_DrawRoundRectangle(POINT r, POINT Xstart, POINT Ystart, POINT Xend, POINT Yend, COLOR Color, DRAW_FILL Filled, DOT_PIXEL Dot_Pixel);
void GUI_DrawCircle(POINT X_Center, POINT Y_Center, LENGTH Radius, COLOR Color, DRAW_FILL Draw_Fill , DOT_PIXEL Dot_Pixel );
void GUI_DrawCircleHelper(uint8 cornername, POINT X_Center, POINT Y_Center, LENGTH Radius, COLOR Color, DRAW_FILL Draw_Fill , DOT_PIXEL Dot_Pixel );

//pic
void GUI_Disbitmap(POINT Xpoint, POINT Ypoint, const unsigned char *pMap, POINT Width, POINT Height, COLOR Color, bool invert);
void GUI_DisGrayMap(POINT Xpoint, POINT Ypoint, const unsigned char *pBmp);

//Display string
void GUI_DisChar(POINT Xstart, POINT Ystart, const char Acsii_Char, sFONT* Font, COLOR Color_Background, COLOR Color_Foreground);
void GUI_DisString_EN(POINT Xstart, POINT Ystart, const char * pString, sFONT* Font, COLOR Color_Background, COLOR Color_Foreground );
void GUI_DisNumDP(POINT Xpoint, POINT Ypoint, uint32 Nummber, uint8 dec_places, sFONT* Font, COLOR Color_Background, COLOR Color_Foreground );
void GUI_DisNum(POINT Xpoint, POINT Ypoint, int32_t Nummber, sFONT* Font, COLOR Color_Background, COLOR Color_Foreground );
void GUI_Showtime(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend, DEV_TIME *pTime, COLOR Color);
//show
void GUI_Show(void);

void initButton(uint8 i,
 int16 x, int16 y, uint16 w, uint16 h,
 uint16 outline, uint16 fill, uint16 textcolor,
 char *label, uint8 textsize);
void initButtonUL(uint8 i,
 int16 x1, int16 y1, uint16 w, uint16 h,
 uint16 outline, uint16 fill, uint16 textcolor,
 char *label, uint8 textsize);
void drawButton(uint8 i, bool inverted);
int8 pressButton(void);
#endif