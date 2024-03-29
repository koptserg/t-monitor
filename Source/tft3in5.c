/*
  ILI9486/ILI9488 SPI driver for CC2530 
  Product: https://aliexpress.ru/item/1005001989363608.html
           https://www.waveshare.com/3.5inch-tft-touch-shield.htm
           https://aliexpress.ru/item/1005005174685600.html
  Written by Koptyakov Sergey 
  From source https://www.waveshare.com/wiki/File:3.5inch_TFT_Touch_Shield_Code.7z                                                            
*/

#include <stdlib.h>

#ifdef TFT3IN5

#include "tft3in5.h"
#include "bme280spi.h" // HalLcd_HW_Control(), HalLcd_HW_Write()
#include "utils.h"

#ifndef HAL_LCD_PWM_PORT
#define HAL_LCD_PWM_PORT 1
#endif
#ifndef HAL_LCD_PWM_PIN
#define HAL_LCD_PWM_PIN  4  // TFT PWM
#endif

LCD_DIS sLCD_DIS;

static void DelayMs(unsigned int delaytime);
static void LCD_SetGramScanWay(LCD_SCAN_DIR Scan_dir);

static void LCD_WriteReg(uint8 Reg);
static void LCD_WriteData(uint8 Data);
//static void LCD_Write_AllData(uint16 Data, uint32 DataLen);
static void LCD_Reset(void);
#ifdef TFT_ILI9486
static void LCD_InitReg_ILI9486(void);
#endif
#ifdef TFT_ILI9488
static void LCD_InitReg_ILI9488(void);
#endif

static void LCD_SetWindow(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend);
//static void LCD_SetCursor(POINT Xpoint, POINT Ypoint);
static void LCD_SetColor(COLOR Color ,POINT Xpoint, POINT Ypoint);

/*******************************************************************************
function:
	Hardware reset
*******************************************************************************/
static void LCD_Reset(void)
{
  HalLcd_HW_Init(); 
}

static void LCD_SetBackLight(uint16 value)
{
  BNAME(HAL_LCD_PWM_PORT, HAL_LCD_PWM_PIN) = value;
}

/*******************************************************************************
function:
		Write register address and data
*******************************************************************************/
static void LCD_WriteReg(uint8 Reg)
{
  HalLcd_HW_Control(Reg);
}

static void LCD_WriteData(uint8 Data)
{
  HalLcd_HW_Write(Data);
}

/*******************************************************************************
function:
		Write register data
*******************************************************************************/
/*
static void LCD_Write_AllData(uint16 Data, uint32 DataLen)
{
  HalLcd_HW_Write_AllData(Data, DataLen);
}
*/
/*******************************************************************************
function:
		Common register initialization
*******************************************************************************/
#ifdef TFT_ILI9486 
static void LCD_InitReg_ILI9486(void) // ILI9486
{

    LCD_WriteReg(0XF9);
    LCD_WriteData(0x00);
    LCD_WriteData(0x08);

    LCD_WriteReg(0xC0);
    LCD_WriteData(0x19);//VREG1OUT POSITIVE
    LCD_WriteData(0x1a);//VREG2OUT NEGATIVE

    LCD_WriteReg(0xC1);
    LCD_WriteData(0x45);//VGH,VGL    VGH>=14V.
    LCD_WriteData(0x00);

    LCD_WriteReg(0xC2);	//Normal mode, increase can change the display quality, while increasing power consumption
    LCD_WriteData(0x33);

    LCD_WriteReg(0XC5);
    LCD_WriteData(0x00);
    LCD_WriteData(0x28);//VCM_REG[7:0]. <=0X80.

    LCD_WriteReg(0xB1);//Sets the frame frequency of full color normal mode
    LCD_WriteData(0xA0);//0XB0 =70HZ, <=0XB0.0xA0=62HZ
    LCD_WriteData(0x11);

    LCD_WriteReg(0xB4);
    LCD_WriteData(0x02); //2 DOT FRAME MODE,F<=70HZ.

    LCD_WriteReg(0xB6);//
    LCD_WriteData(0x00);
    LCD_WriteData(0x42);//0 GS SS SM ISC[3:0];
    LCD_WriteData(0x3B);

    LCD_WriteReg(0xB7);
    LCD_WriteData(0x07);

    LCD_WriteReg(0xE0);
    LCD_WriteData(0x1F);
    LCD_WriteData(0x25);
    LCD_WriteData(0x22);
    LCD_WriteData(0x0B);
    LCD_WriteData(0x06);
    LCD_WriteData(0x0A);
    LCD_WriteData(0x4E);
    LCD_WriteData(0xC6);
    LCD_WriteData(0x39);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);

    LCD_WriteReg(0XE1);
    LCD_WriteData(0x1F);
    LCD_WriteData(0x3F);
    LCD_WriteData(0x3F);
    LCD_WriteData(0x0F);
    LCD_WriteData(0x1F);
    LCD_WriteData(0x0F);
    LCD_WriteData(0x46);
    LCD_WriteData(0x49);
    LCD_WriteData(0x31);
    LCD_WriteData(0x05);
    LCD_WriteData(0x09);
    LCD_WriteData(0x03);
    LCD_WriteData(0x1C);
    LCD_WriteData(0x1A);
    LCD_WriteData(0x00);

    LCD_WriteReg(0XF1);
    LCD_WriteData(0x36);
    LCD_WriteData(0x04);
    LCD_WriteData(0x00);
    LCD_WriteData(0x3C);
    LCD_WriteData(0x0F);
    LCD_WriteData(0x0F);
    LCD_WriteData(0xA4);
    LCD_WriteData(0x02);

    LCD_WriteReg(0XF2);
    LCD_WriteData(0x18);
    LCD_WriteData(0xA3);
    LCD_WriteData(0x12);
    LCD_WriteData(0x02);
    LCD_WriteData(0x32);
    LCD_WriteData(0x12);
    LCD_WriteData(0xFF);
    LCD_WriteData(0x32);
    LCD_WriteData(0x00);

    LCD_WriteReg(0XF4);
    LCD_WriteData(0x40);
    LCD_WriteData(0x00);
    LCD_WriteData(0x08);
    LCD_WriteData(0x91);
    LCD_WriteData(0x04);

    LCD_WriteReg(0XF8);
    LCD_WriteData(0x21);
    LCD_WriteData(0x04);

    LCD_WriteReg(0X3A);	//Set Interface Pixel Format
#ifdef HAL_LCD_RGB_18BIT
    LCD_WriteData(0x66);
#else
    LCD_WriteData(0x55);
#endif    

}
#endif
#ifdef TFT_ILI9488
static void LCD_InitReg_ILI9488(void) // ILI9488_CMI35IPS
{	
    LCD_WriteReg(0xE0); //P-Gamma
    LCD_WriteData(0x00);
    LCD_WriteData(0x13);
    LCD_WriteData(0x18);
    LCD_WriteData(0x04);
    LCD_WriteData(0x0F);
    LCD_WriteData(0x06);
    LCD_WriteData(0x3A);
    LCD_WriteData(0x56);
    LCD_WriteData(0x4D);
    LCD_WriteData(0x03);
    LCD_WriteData(0x0A);
    LCD_WriteData(0x06);
    LCD_WriteData(0x30);
    LCD_WriteData(0x3E);
    LCD_WriteData(0x0F);

    LCD_WriteReg(0XE1); //N-Gamma
    LCD_WriteData(0x00);
    LCD_WriteData(0x13);
    LCD_WriteData(0x18);
    LCD_WriteData(0x01);
    LCD_WriteData(0x11);
    LCD_WriteData(0x06);
    LCD_WriteData(0x38);
    LCD_WriteData(0x34);
    LCD_WriteData(0x4D);
    LCD_WriteData(0x06);
    LCD_WriteData(0x0D);
    LCD_WriteData(0x0B);
    LCD_WriteData(0x31);
    LCD_WriteData(0x37);
    LCD_WriteData(0x0F); 

    LCD_WriteReg(0XC0);   //Power Control 1
    LCD_WriteData(0x18); //Vreg1out
    LCD_WriteData(0x17); //Verg2out

    LCD_WriteReg(0xC1);   //Power Control 2
    LCD_WriteData(0x41); //VGH,VGL

    LCD_WriteReg(0xC5);   //Power Control 3
    LCD_WriteData(0x00);
    LCD_WriteData(0x1A); //Vcom
    LCD_WriteData(0x80);

    LCD_WriteReg(0x36);
    LCD_WriteData(0x48);  // MY MX MV ML BGR MH HF VF  4A     48  08
    //Write_D(0xE8);  // MY MX MV ML BGR MH X X
                    // MY Row Address Order
                    // MX Column Address Order
                    // MV Row / Column Exchange
                    // ML Vertical Refresh Order
                    // (0=RGB color filter panel, 1=BGR color filter panel)
                    // MH Horizontal Refresh ORDER
                    // Horizontal Flip x
                    // Vertical Flip  y

    LCD_WriteReg(0XB0);   // Interface Mode Control
    LCD_WriteData(0x00);

    LCD_WriteReg(0xB1);   //Frame rate
    LCD_WriteData(0xA0); //60Hz

    LCD_WriteReg(0xB4);   //Display Inversion Control
    LCD_WriteData(0x02); //2-dot

    LCD_WriteReg(0XB6);   //RGB/MCU Interface Control
    LCD_WriteData(0x02); //MCU RGB
    LCD_WriteData(0x02); //Source,Gate scan dieection

    LCD_WriteReg(0XE9);    // Set Image Function
    LCD_WriteData(0x00);  //disable 24 bit data input

    LCD_WriteReg(0xF7);    // Adjust Control
    LCD_WriteData(0xA9);
    LCD_WriteData(0x51);
    
    LCD_WriteReg(0X3A);	//Set Interface Pixel Format
#ifdef HAL_LCD_RGB_18BIT
    LCD_WriteData(0x66);
#else
    LCD_WriteData(0x55);
#endif

//    LCD_WriteReg(0x20); // Display Inversion OFF    
//    LCD_WriteReg(0x21); // Display Inversion ON	  

}
#endif
/********************************************************************************
function:	Set the display scan and color transfer modes
parameter:
		Scan_dir   :   Scan direction
		Colorchose :   RGB or GBR color format
********************************************************************************/
static void LCD_SetGramScanWay(LCD_SCAN_DIR Scan_dir)
{
    uint16 MemoryAccessReg_Data = 0; //addr:0x36
    uint16 DisFunReg_Data = 0; //addr:0xB6

    // Gets the scan direction of GRAM
    switch (Scan_dir) {
    case L2R_U2D:
        MemoryAccessReg_Data = 0x08;//0x08 | 0X8
        DisFunReg_Data = 0x22;
        break;
    case L2R_D2U:
        MemoryAccessReg_Data = 0x08;
        DisFunReg_Data = 0x62;
        break;
    case R2L_U2D: //0X4
        MemoryAccessReg_Data = 0x08;
        DisFunReg_Data = 0x02;
        break;
    case R2L_D2U: //0XC
        MemoryAccessReg_Data = 0x08;
        DisFunReg_Data = 0x42;
        break;
    case U2D_L2R: //0X2
        MemoryAccessReg_Data = 0x28;
        DisFunReg_Data = 0x22;
        break;
    case U2D_R2L: //0X6
        MemoryAccessReg_Data = 0x28;
        DisFunReg_Data = 0x02;
        break;
    case D2U_L2R: //0XA
        MemoryAccessReg_Data = 0x28;
        DisFunReg_Data = 0x62;
        break;
    case D2U_R2L: //0XE
        MemoryAccessReg_Data = 0x28;
        DisFunReg_Data = 0x42;
        break;
    }

    //Get the screen scan direction
    sLCD_DIS.LCD_Scan_Dir = Scan_dir;

    //Get GRAM and LCD width and height
    if(Scan_dir == L2R_U2D || Scan_dir == L2R_D2U || Scan_dir == R2L_U2D || Scan_dir == R2L_D2U) {
        sLCD_DIS.LCD_Dis_Column	= LCD_HEIGHT ;
        sLCD_DIS.LCD_Dis_Page = LCD_WIDTH ;
    } else {
        sLCD_DIS.LCD_Dis_Column	= LCD_WIDTH ;
        sLCD_DIS.LCD_Dis_Page = LCD_HEIGHT ;
    }

    // Set the read / write scan direction of the frame memory
    LCD_WriteReg(0xB6);
    LCD_WriteData(0X00);
    LCD_WriteData(DisFunReg_Data);

    LCD_WriteReg(0x36);
    LCD_WriteData(MemoryAccessReg_Data);
}

/********************************************************************************
function:
	initialization
********************************************************************************/
void LCD_Init(LCD_SCAN_DIR LCD_ScanDir, uint16 LCD_BLval)
{
    //Hardware reset
    LCD_Reset();
    
    LCD_SetBackLight(LCD_BLval);
    
    //Set the initialization register
#ifdef TFT_ILI9486    
    LCD_InitReg_ILI9486();
#endif
#ifdef TFT_ILI9488     
    LCD_InitReg_ILI9488();
#endif
    
    //Set the display scan and color transfer modes
    LCD_SetGramScanWay( LCD_ScanDir);
    DelayMs(200);

    //sleep out
    LCD_WriteReg(0x11);
    DelayMs(120);

    //Turn on the LCD display
    LCD_WriteReg(0x29);
}

static void DelayMs(unsigned int delaytime) {
  while(delaytime--)
  {
    uint16 microSecs = 1000;
    while(microSecs--)
    {
      asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    }
  }
}

/********************************************************************************
function:	Sets the start position and size of the display area
parameter:
	Xstart 	:   X direction Start coordinates
	Ystart  :   Y direction Start coordinates
	Xend    :   X direction end coordinates
	Yend    :   Y direction end coordinates
********************************************************************************/
static void LCD_SetWindow(POINT Xstart, POINT Ystart,	POINT Xend, POINT Yend)
{
    //set the X coordinates
    LCD_WriteReg(0x2A);
    LCD_WriteData(Xstart >> 8);	 				//Set the horizontal starting point to the high octet
    LCD_WriteData(Xstart & 0xff);	 				//Set the horizontal starting point to the low octet
    LCD_WriteData((Xend - 1) >> 8);	//Set the horizontal end to the high octet
    LCD_WriteData((Xend - 1) & 0xff);	//Set the horizontal end to the low octet
    //set the Y coordinates
    LCD_WriteReg(0x2B);
    LCD_WriteData(Ystart >> 8);
    LCD_WriteData(Ystart & 0xff );
    LCD_WriteData((Yend - 1) >> 8);
    LCD_WriteData((Yend - 1) & 0xff);
    LCD_WriteReg(0x2C);
}

/********************************************************************************
function:	Set the display point (Xpoint, Ypoint)
parameter:
	xStart :   X direction Start coordinates
	xEnd   :   X direction end coordinates
********************************************************************************/
/*
static void LCD_SetCursor(POINT Xpoint, POINT Ypoint)
{
    LCD_SetWindow(Xpoint, Ypoint, Xpoint, Ypoint);
}
*/
/********************************************************************************
function:	Set show color
parameter:
		Color  :   Set show color,16-bit depth
********************************************************************************/
//static void LCD_SetColor(LENGTH Dis_Width, LENGTH Dis_Height, COLOR Color ){
static void LCD_SetColor(COLOR Color , POINT Xpoint, POINT Ypoint)
{
//    LCD_Write_AllData(Color , (uint32_t)Xpoint * (uint32_t)Ypoint);
  HalLcd_HW_Write_AllData(Color, (uint32_t)Xpoint * (uint32_t)Ypoint);
}

/********************************************************************************
function:	Point (Xpoint, Ypoint) Fill the color
parameter:
	Xpoint :   The x coordinate of the point
	Ypoint :   The y coordinate of the point
	Color  :   Set the color
********************************************************************************/
void LCD_SetPointlColor( POINT Xpoint, POINT Ypoint, COLOR Color)
{
    if ((Xpoint <= sLCD_DIS.LCD_Dis_Column) && (Ypoint <= sLCD_DIS.LCD_Dis_Page)) {
//        LCD_SetCursor (Xpoint, Ypoint);
        LCD_SetWindow(Xpoint, Ypoint, Xpoint, Ypoint);
        LCD_SetColor(Color, 1, 1);
    }
}

/********************************************************************************
function:	Fill the area with the color
parameter:
	Xstart :   Start point x coordinate
	Ystart :   Start point y coordinate
	Xend   :   End point coordinates
	Yend   :   End point coordinates
	Color  :   Set the color
********************************************************************************/
void LCD_SetArealColor(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend, COLOR Color)
{
    if((Xend > Xstart) && (Yend > Ystart)) {
        LCD_SetWindow(Xstart , Ystart , Xend , Yend  );
        LCD_SetColor ( Color , Xend - Xstart, Yend - Ystart);
    }
}

void LCD_SetArealColorWH(POINT Xstart, POINT Ystart, LENGTH w, LENGTH h, COLOR Color)
{
    POINT Xend = Xstart + w;
    POINT Yend = Ystart + h;
    if((Xend > Xstart) && (Yend > Ystart)) {
        LCD_SetWindow(Xstart , Ystart , Xend , Yend  );
        LCD_SetColor ( Color , Xend - Xstart, Yend - Ystart);
    }
}

/********************************************************************************
function:
			Clear screen
********************************************************************************/
void LCD_Clear(COLOR  Color)
{
    LCD_SetArealColor(0, 0, sLCD_DIS.LCD_Dis_Column , sLCD_DIS.LCD_Dis_Page , Color);
}

#endif //end TFT3IN5