#ifndef TFT3IN5_H
#define TFT3IN5_H

#define	COLOR				unsigned int		//The variable type of the color (unsigned short) 
#define	POINT				unsigned int		//The type of coordinate (unsigned short) 
#define	LENGTH				unsigned int		//The type of coordinate (unsigned short) 

/********************************************************************************
function:
		Define the full screen height length of the display
********************************************************************************/
#define LCD_X_MAXPIXEL  480  //LCD width maximum memory 
#define LCD_Y_MAXPIXEL  320 //LCD height maximum memory
#define LCD_X	 0
#define LCD_Y	 0

#define LCD_WIDTH  (LCD_X_MAXPIXEL - 2 * LCD_X)  //LCD width
#define LCD_HEIGHT  LCD_Y_MAXPIXEL //LCD height

/********************************************************************************
function:
			scanning method
********************************************************************************/
typedef enum {
    L2R_U2D  = 0,	//The display interface is displayed , left to right, up to down
    L2R_D2U  ,
    R2L_U2D  ,
    R2L_D2U  ,

    U2D_L2R  ,
    U2D_R2L  ,
    D2U_L2R  ,
    D2U_R2L  ,
} LCD_SCAN_DIR;
//#define SCAN_DIR_DFT  D2U_L2R  //Default scan direction = L2R_U2D

#ifdef HAL_LCD_RGB_18BIT
#define SCAN_DIR_DFT  L2R_U2D // portrait
#else
#define SCAN_DIR_DFT  R2L_D2U //portrait
#endif
    

/********************************************************************************
function:
	Defines the total number of rows in the display area
********************************************************************************/
typedef struct {
    LENGTH LCD_Dis_Column;	//COLUMN
    LENGTH LCD_Dis_Page;	//PAGE
    LCD_SCAN_DIR LCD_Scan_Dir;
    POINT LCD_X_Adjust;		//LCD x actual display position calibration
    POINT LCD_Y_Adjust;		//LCD y actual display position calibration
} LCD_DIS;

/********************************************************************************
function:
			Macro definition variable name
********************************************************************************/
extern void LCD_Init(LCD_SCAN_DIR LCD_ScanDir, uint16 LCD_BLval);
extern void LCD_SetPointlColor(POINT Xpoint, POINT Ypoint, COLOR Color);
extern void LCD_SetArealColor(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend,COLOR  Color);
extern void LCD_SetArealColorWH(POINT Xstart, POINT Ystart, LENGTH w, LENGTH h, COLOR Color);
extern void LCD_Clear(COLOR  Color);

#endif /* TFT3IN5_H */

/* END OF FILE */