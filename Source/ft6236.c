/*
  FT6236 CTP I2C driver for CC2530 
  Product: https://aliexpress.ru/item/1005005174685600.html
  Written by Koptyakov Sergey 
  From source https://github.com/adafruit/Adafruit_FT6206_Library/tree/master
              https://github.com/renaudcalmont/FT6236_t3                                                               
*/

#include <stdlib.h>
#if defined(CTP_FT6236)

#include "Debug.h"
#include "ft6236.h"
#include "hal_i2c.h"
#include "math.h"

#include "zcl_app.h"
#include "hal_key.h"

extern LCD_DIS sLCD_DIS;
static CTP_DEV sTP_DEV;

CTP_DRAW sTP_Draw;
static bool tp_pres = 1;
bool tp_mode = 1;

uint8 touches;
uint16 touchX[2];
uint16 touchY[2];
uint16 touchID[2];

static void CTP_Init( LCD_SCAN_DIR Lcd_ScanDir );
static void ft6236_writeRegister8(uint8_t reg, uint8_t val);
static void ft6236_readData(void);

uint8 ft6236_TaskId = 0;

void ft6236_Init(uint8 task_id) {
    ft6236_TaskId = task_id;
#ifdef HAL_LCD_RGB_18BIT    
    LCD_SCAN_DIR Lcd_ScanDir = SCAN_DIR_DFT;
#else    
    LCD_SCAN_DIR Lcd_ScanDir = SCAN_DIR_DFT;    //SCAN_DIR_DFT = R2L_D2U
#endif    
    CTP_Init( Lcd_ScanDir );
}

uint16 ft6236_event_loop(uint8 task_id, uint16 events) {
     if (events & APP_TFT_CTP_IRQ_EVT) {
        LREPMaster("APP_TFT_CTP_IRQ_EVT\r\n");
        
        ft6236_readData();
          
//          LREP("Xpoint=%d Ypoint=%d\r\n", sTP_Draw.Xpoint, sTP_Draw.Ypoint);
       
//         LCD_SetArealColor(40, 160, 320, 176, FONT_BACKGROUND);
//         GUI_DisNum(40, 160, sTP_Draw.Xpoint, &Font16, FONT_BACKGROUND, BLACK);
//         GUI_DisNum(140, 160,sTP_Draw.Ypoint, &Font16, FONT_BACKGROUND, BLACK);
//         GUI_DisNum(240, 160,touches, &Font16, FONT_BACKGROUND, BLACK);         
//         GUI_DrawPoint(touchX[0], touchY[0], WHITE, DOT_PIXEL_DFT, DOT_STYLE_DFT);
         
         zclApp_TPkeyprocessing();
        
        return (events ^ APP_TFT_CTP_IRQ_EVT);
    }
    
    return 0;
}    

void ft6236_HandleKeys(uint8 portAndAction, uint8 keyCode) { 
  
  bool contact = portAndAction & HAL_KEY_PRESS ? TRUE : FALSE;
  if (portAndAction & HAL_KEY_PORT0) {
//        LREPMaster("Key press PORT0\r\n");       
        if (contact){
          if (tp_pres){
            if (tp_mode) {
              osal_start_timerEx(ft6236_TaskId, APP_TFT_CTP_IRQ_EVT, 30); //1
            } else {
              osal_start_reload_timer(ft6236_TaskId, APP_TFT_CTP_IRQ_EVT, 30);
            }
          }
          if (tp_mode) {
            tp_pres = 0;
          }
        } else {
          tp_pres = 1;
          osal_stop_timerEx(ft6236_TaskId, APP_TFT_CTP_IRQ_EVT);
          osal_clear_event(ft6236_TaskId, APP_TFT_CTP_IRQ_EVT);
        }
    }

}

static void CTP_Init( LCD_SCAN_DIR Lcd_ScanDir )
{
  sTP_DEV.TP_Scan_Dir = Lcd_ScanDir;
  ft6236_writeRegister8(0, 0); // device mode = Normal
  ft6236_writeRegister8(0xA4, 0x00); // Interrupt polling mode
}

static void ft6236_writeRegister8(uint8_t reg, uint8_t val) {
  // use i2c    
  uint8 buf[] = {reg, val};
  HalI2CSend(FT62XX_ADDR_WRITE, buf, 2);
}

uint8 ft6236_readRegister8(uint8 reg) {
  uint8 buf[1] = {0x00};
  uint8 regbuf[1] = {0x00};
  regbuf[0] = reg;
  // use i2c
  HalI2CSend(FT62XX_ADDR_WRITE, regbuf, 1);

  HalI2CReceive(FT62XX_ADDR_READ, buf, 1);
  uint8 x = buf[0];
  return x;
}

static void ft6236_readData(void) {

  uint8 i2cdat[16];
  ft6236_writeRegister8(0, 0x00); // device mode = Normal
//  ft6236_readRegister8(0x02);

  HalI2CReceive(FT62XX_ADDR_READ, i2cdat, 16);  

  touches = i2cdat[0x01]; // 0x02 TD_STATUS [3:0] Number of touch points
  if ((touches > 2) || (touches == 0)) {
    touches = 0;
  }
 
  for (uint8_t i = 0; i < 2; i++) {
    touchX[i] = i2cdat[0x02 + i * 6] & 0x0F; // 0x03 P1_XH [3:0] i st Touch X Position[11:8]
    touchX[i] <<= 8;           
    touchX[i] |= i2cdat[0x03 + i * 6];       // 0x04 P1_XL [7:0] i st Touch X Position 
    touchY[i] = i2cdat[0x04 + i * 6] & 0x0F; // 0x05 P1_YH [3:0] i st Touch Y Position[11:8] 
    touchY[i] <<= 8;
    touchY[i] |= i2cdat[0x05 + i * 6];       // 0x06 P1_YL [7:0] i st Touch Y Position
    touchID[i] = i2cdat[0x04 + i * 6] >> 4;  // 0x05 P1_YH [7:4] i st Touch ID 
  }
  
  // rotate L2R_U2D - 0, U2D_R2L - 90, R2L_D2U - 180, D2U_L2R - 270
  if (sTP_DEV.TP_Scan_Dir == R2L_D2U) {       //Converts the result to screen coordinates
        sTP_Draw.Xpoint = LCD_HEIGHT - touchX[0] ;
        sTP_Draw.Ypoint = LCD_WIDTH - touchY[0];
      } else if (sTP_DEV.TP_Scan_Dir == L2R_U2D) { // default L2R_U2D       
        sTP_Draw.Xpoint = touchX[0];
        sTP_Draw.Ypoint = touchY[0];
      } else if (sTP_DEV.TP_Scan_Dir == D2U_L2R) { 
        sTP_Draw.Xpoint = LCD_WIDTH - touchY[0];
        sTP_Draw.Ypoint = touchX[0];
      } else if (sTP_DEV.TP_Scan_Dir == U2D_R2L) {
        sTP_Draw.Xpoint = touchY[0];
        sTP_Draw.Ypoint = LCD_HEIGHT - touchX[0];
      }else {
        sTP_Draw.Xpoint = touchX[0];
        sTP_Draw.Ypoint = touchY[0];
      }
}

#endif //CTP_FT6236