
#include "AF.h"
#include "OSAL.h"
#include "OSAL_Clock.h"
#include "OSAL_PwrMgr.h"
#include "OSAL_Memory.h"
#include "ZComDef.h"
#include "ZDApp.h"
#include "ZDNwkMgr.h"
#include "ZDObject.h"
#include "math.h"

#include "nwk_util.h"
#include "zcl.h"

#include "zcl_app.h"
#include "zcl_rep.h"

#include "zcl_diagnostic.h"
#include "zcl_general.h"
#include "zcl_ms.h"
#include "zcl_hvac.h"

#include "bdb.h"
#include "bdb_interface.h"
#include "gp_interface.h"

#include "Debug.h"

#include "OnBoard.h"

/* HAL */
#include "hal_adc.h"
#include "hal_drivers.h"
#include "hal_i2c.h"
#include "hal_key.h"
#include "hal_led.h"

#include "bme280spi.h"
#include "bh1750.h"
//#include "scd40.h"
#include "scd4x.h"
#include "battery.h"
#include "commissioning.h"
#include "factory_reset.h"
#include "utils.h"
#include "version.h"
#include "beeping.h"

#ifdef EPD3IN7
#include "epd3in7.h"
#include "epdpaint.h"
#endif
#include "imagedata.h"
#ifdef TFT3IN5
#include "tft3in5.h"
#include "xpt2046.h"
#include "lcdgui.h"
#if defined(BREAKOUT) 
#include "breakout.h"
#endif
#endif

#if defined(EPD3IN7)
#define HAL_LCD_BUSY BNAME(HAL_LCD_BUSY_PORT, HAL_LCD_BUSY_PIN)
#endif
/*********************************************************************
 * MACROS
 */
#ifndef MOTION_PORT
#define MOTION_PORT 1 
#endif   
#ifndef MOTION_PIN
#define MOTION_PIN  3
#endif   
#define MOTION BNAME(MOTION_PORT, MOTION_PIN)

#ifndef MOTION_POWER_PORT
#define MOTION_POWER_PORT 1 
#endif   
#ifndef MOTION_POWER_PIN
#define MOTION_POWER_PIN  0
#endif 
 
//#define HAL_KEY_P0_EDGE_BITS HAL_KEY_BIT0

#define HAL_KEY_P1_EDGE_BITS (HAL_KEY_BIT1 | HAL_KEY_BIT2)

//#define HAL_KEY_CODE_RELEASE_KEY HAL_KEY_CODE_NOKEY

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

extern bool requestNewTrustCenterLinkKey;
byte zclApp_TaskID;

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint8 currentSensorsReadingPhase = 0;

uint8 report = 0;
uint8 power = 0;
bool  bmeDetect = 0;
uint8 motionDetect = 0;
uint8 bh1750Detect = 0;
uint8 BH1750_mode = ONE_TIME_HIGH_RES_MODE;
bool  scd4xDetect = 0;

uint16 temp_bh1750IlluminanceSensor_MeasuredValue[3];
uint16 old_bh1750IlluminanceSensor_MeasuredValue;
uint16 temp_scd4xCO2_Sensor_MeasuredValue[3];
uint16 old_scd4xCO2_Sensor_MeasuredValue;
uint16 temp_Temperature_Sensor_MeasuredValue[3];
uint16 old_Temperature_Sensor_MeasuredValue;
uint16 temp_PressureSensor_MeasuredValue[3];
uint16 old_PressureSensor_MeasuredValue;
uint16 temp_PressureSensor_ScaledValue[3];
uint16 old_PressureSensor_ScaledValue;
uint16 temp_HumiditySensor_MeasuredValue[3];
uint16 old_HumiditySensor_MeasuredValue;
uint16 temp_Sender_shortAddr[3] = {0xFFFE , 0xFFFE, 0xFFFE};
uint16 temp_Sender_extAddr[3] = {0xFFFE , 0xFFFE, 0xFFFE};
uint8 temp_Battery_PercentageRemainig[3] = {0xFF , 0xFF, 0xFF};
uint8 temp_Occupied[3];
uint8 temp_Binary[3];

bool EpdDetect = 1; 
uint8 fullupdate_hour = 0;
uint8 zclApp_color = 1;
uint32 zclApp_GenTime_old = 0;

#ifdef LQI_REQ
uint8 temp_lqi[3] = {0xFF , 0xFF, 0xFF};
uint8 temp_LqiStartIndex[3] = {0, 0, 0};
uint8 temp_countReqLqi = 0;
#endif
#ifdef MT_ZDO_MGMT
//uint8 temp_countReqLqi = 0;
#endif
#ifdef BIND_REQ 
uint8 temp_bindingCount[3] = {0, 0, 0};
uint8 temp_bindingStartIndex[3] = {0, 0, 0};
uint8 temp_countReqBind = 0;

uint16 temp_bindClusterDev[3] = {0x0000, 0x0000, 0x0000};
uint16 old_bindClusterDev[3]  = {0x0000, 0x0000, 0x0000};
uint16 zclApp_UpDown[3]       = {0x0000, 0x0000, 0x0000}; 
uint16 zlcApp_ExtAddr = 0xFFFE;
#endif

afAddrType_t inderect_DstAddr = {.addrMode = (afAddrMode_t)AddrNotPresent, .endPoint = 0, .addr.shortAddr = 0};

#if defined(TFT3IN5)
extern TP_DRAW sTP_Draw;
uint16 zclApp_lcd_background;
bool zcl_game = 0;

const uint16 color_scheme[3][8] = {BLUE0,    BLUE1,    BLUE2,    BLUE3,    BLUE4,    BLUE5,   ORANG1,   ORANG2,
                                   RED0,     RED1,     RED2,     RED3,     RED4,     RED5,     GREEN1,  GREEN2,
                                   GREEN_0,  GREEN_1,  GREEN_2,  GREEN_3,  GREEN_4,  GREEN_5,  RED_1,   RED_2};
uint8 scheme = 1;
uint8 zclApp_menu = 0;
#endif //TFT3IN5

//#define BUTTON_COUNT_MAX  4
//extern BUTTON sButton[BUTTON_COUNT_MAX];
//bool butt_pause = 0;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclApp_HandleKeys(byte shift, byte keys);
static void zclApp_Report(void);

static void zclApp_BasicResetCB(void);
static void zclApp_RestoreAttributesFromNV(void);
static void zclApp_SaveAttributesToNV(void);
static void zclApp_StopReloadTimer(void);
static void zclApp_StartReloadTimer(void);

static ZStatus_t zclApp_ReadWriteAuthCB(afAddrType_t *srcAddr, zclAttrRec_t *pAttr, uint8 oper);

static void zclApp_LocalTime(void);
static void zclApp_ReadSensors(void);
static void zclApp_ReadBME280Temperature(void);
static void zclApp_ReadBME280Pressure(void);
static void zclApp_ReadBME280Humidity(void);
static void zclApp_bh1750StartLumosity(void);
static void zclApp_bh1750ReadLumosity(void);
static void zclApp_bh1750setMTreg(void);
static void zclApp_MotionPullUpDown(void);
static void zclApp_ConfigDisplay(void);
static void zclApp_scd4xReadCO2(void);

#if defined(EPD3IN7)
static void zclApp_EpdUpdateClock(void);
static void EpdRefresh(void);
static void EpdtestRefresh(void);
static void zclApp_EpdSensors(uint8 i);
static void EpdTimeDateWeek(void);
static void EpdStatus(uint8 temp_s);
static void EpdBindStatus(uint8 temp_s);
static void EpdLqi(uint8 temp_l);
static void EpdBattery(uint8 temp_b);
static void EpdNwk(uint8 temp_n);
static void EpdOccupancy(uint8 temp_oc);
static void EpdIlluminance(uint8 temp_i);
static void EpdTemperature(uint8 temp_t);
static void EpdHumidity(uint8 temp_h);
static void EpdPressure(uint8 temp_p);
static void _delay_us(uint16 microSecs);
static void _delay_ms(uint16 milliSecs);
#endif

#if defined(TFT3IN5)
static void TfttestRefresh(uint8 i);
static void TftUpdateRefresh(void);
static void TftTimeDateWeek(void);
static void TftStatus(uint8 temp_s);
static void TftBindStatus(uint8 temp_s);
static void TftLqi(uint8 temp_l);
static void TftNwk(uint8 temp_n);
static void TftBattery(uint8 temp_b);
static void TftOccupancy(uint8 temp_oc);
static void TftBinary(uint8 temp_b);
static void TftIlluminance(uint8 temp_i);
static void TftCO2(uint8 temp_co2);
static void TftTemperature(uint8 temp_t);
static void TftHumidity(uint8 temp_h);
static void TftPressure(uint8 temp_p);
static void TftWidgetMeasuredValue(uint8 dev_num, uint16 cluster_bit);
static void zclApp_keyprocessing(void);
static void zclApp_create_butt_main(uint8 block);
static void zclApp_automenu(void);
#if defined(HAL_LCD_PWM_PORT1)
static void InitLedPWM(uint8 level);
#endif
#endif

#ifdef MT_ZDO_MGMT
void zclApp_ProcessZDOMsgs(zdoIncomingMsg_t *InMsg);
#endif
#ifdef LQI_REQ
void zclApp_RequestLqi(void);
#endif
#ifdef BIND_REQ
void zclApp_RequestBind(void);
#endif
#ifdef IEEE_ADDR_REQ
void zclApp_RequestAddr(uint16 nwkaddr);
#endif
static void zclApp_ProcessIncomingMsg( zclIncomingMsg_t *msg );
static void zclApp_AttrIncomingReport( zclIncomingMsg_t *pInMsg );
static void zclApp_DefaultRspCmd( zclIncomingMsg_t *pInMsg );

void zclApp_SetTimeDate(void);

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclApp_CmdCallbacks = {
    zclApp_BasicResetCB, // Basic Cluster Reset command
    NULL, // Identify Trigger Effect command
    zclApp_OnOffCB, // On/Off cluster commands
    NULL, // On/Off cluster enhanced command Off with Effect
    NULL, // On/Off cluster enhanced command On with Recall Global Scene
    NULL, // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
    NULL,   // Level Control Move to Level command
    NULL,          // Level Control Move command
    NULL,          // Level Control Step command
    NULL,          // Level Control Stop command    
#endif
#ifdef ZCL_GROUPS
    NULL,                  // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,             // Scene Store Request command
  NULL,            // Scene Recall Request command
  NULL,                  // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                     // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,              // Get Event Log command
  NULL,           // Publish Event Log command
#endif    
    NULL, // RSSI Location command
    NULL  // RSSI Location Response command
};

void zclApp_MotionPullUpDown(void) {
    if (zclApp_Occupied == 1){
      P2INP &= ~HAL_KEY_BIT6; // pull up port1
//      IO_PUD_PORT(MOTION_PORT, IO_PUP); // pull up port1
      MicroWait(50);
      PICTL |= HAL_KEY_P1_EDGE_BITS; // set falling edge on port
    } else {
      P2INP |= HAL_KEY_BIT6; // pull down port1
//      IO_PUD_PORT(MOTION_PORT, IO_PDN); // pull down port1
      MicroWait(50);
      PICTL &= ~(HAL_KEY_P1_EDGE_BITS);     
    }
}

void zclApp_Init(byte task_id) {
    zclApp_RestoreAttributesFromNV();
        
#ifdef HAL_LCD_PWM_PORT1    
    InitLedPWM(10);
#endif
#ifdef HAL_LCD_PWM_PORT0    
//    InitBuzzer(100, 3000);
#endif
    P1SEL &= ~BV(0); // Set P1_0 to GPIO
//    IO_FUNC_PORT_PIN(MOTION_POWER_PORT, MOTION_POWER_PIN, IO_GIO); // Set P1_0 to GPIO
    P1DIR |= BV(0); // P1_0 output
//    IO_DIR_PORT_PIN(MOTION_POWER_PORT, MOTION_POWER_PIN, IO_OUT); // P1_0 output
    P1 &= ~BV(0);   // power off DD //--
//    BNAME(MOTION_POWER_PORT, MOTION_POWER_PIN)= 0; // P1_0 = 0 power off DD
    
    motionDetect = P1_3;
//    motionDetect = MOTION;
    zclApp_Occupied_OnOff = motionDetect;
    zclApp_Occupied = motionDetect;
    zclApp_MotionPullUpDown();
    
    SPIInit();
    
    bmeDetect = BME280Init();
    
    HalI2CInit();
    bh1750Detect = bh1750_init(BH1750_mode);
    zclApp_bh1750setMTreg();

    SCD4x_setTemperatureOffset(4.3, 1); // 5 grC 1 mc
//    SCD4x_setSensorAltitude(270, 1); 
    scd4xDetect = SCD4x_begin(true, true, false);
    
    // this is important to allow connects throught routers
    // to make this work, coordinator should be compiled with this flag #define TP2_LEGACY_ZC
    requestNewTrustCenterLinkKey = FALSE;

    zclApp_TaskID = task_id;

    zclGeneral_RegisterCmdCallbacks(1, &zclApp_CmdCallbacks);
    zcl_registerAttrList(zclApp_FirstEP.EndPoint, zclApp_AttrsFirstEPCount, zclApp_AttrsFirstEP);
    bdb_RegisterSimpleDescriptor(&zclApp_FirstEP);
    zcl_registerReadWriteCB(zclApp_FirstEP.EndPoint, NULL, zclApp_ReadWriteAuthCB);

    zcl_registerAttrList(zclApp_SecondEP.EndPoint, zclApp_AttrsSecondEPCount, zclApp_AttrsSecondEP);
    bdb_RegisterSimpleDescriptor(&zclApp_SecondEP);
    zcl_registerReadWriteCB(zclApp_SecondEP.EndPoint, NULL, zclApp_ReadWriteAuthCB);
    
    zcl_registerAttrList(zclApp_ThirdEP.EndPoint, zclApp_AttrsThirdEPCount, zclApp_AttrsThirdEP);
    bdb_RegisterSimpleDescriptor(&zclApp_ThirdEP);
    zcl_registerReadWriteCB(zclApp_ThirdEP.EndPoint, NULL, zclApp_ReadWriteAuthCB);
    
    zcl_registerAttrList(zclApp_FourthEP.EndPoint, zclApp_AttrsFourthEPCount, zclApp_AttrsFourthEP);
    bdb_RegisterSimpleDescriptor(&zclApp_FourthEP);   
    zcl_registerReadWriteCB(zclApp_FourthEP.EndPoint, NULL, zclApp_ReadWriteAuthCB);
    
    zcl_registerAttrList(zclApp_FifthEP.EndPoint, zclApp_AttrsFifthEPCount, zclApp_AttrsFifthEP);
    bdb_RegisterSimpleDescriptor(&zclApp_FifthEP);   
    zcl_registerReadWriteCB(zclApp_FifthEP.EndPoint, NULL, zclApp_ReadWriteAuthCB);

    zcl_registerForMsg(zclApp_TaskID);

    // Register for all key events - This app will handle all key events
    RegisterForKeys(zclApp_TaskID);
#ifdef LQI_REQ
    // Register the callback Mgmt_Lqi_rsp
    ZDO_RegisterForZDOMsg(zclApp_TaskID, Mgmt_Lqi_rsp);
#endif
#ifdef BIND_REQ    
    // Register the callback Mgmt_Bind_rsp
    ZDO_RegisterForZDOMsg(zclApp_TaskID, Mgmt_Bind_rsp);
    //IEEE address device definition
    uint8* eAd8 = NULL;
    eAd8 = NLME_GetExtAddr();
    zlcApp_ExtAddr = BUILD_UINT16( eAd8[0],eAd8[1] );
#endif
#ifdef IEEE_ADDR_REQ    
    // Register the callback IEEE_addr_rsp
    ZDO_RegisterForZDOMsg(zclApp_TaskID, IEEE_addr_rsp);
#endif
#if defined(TFT3IN5)
  if (zclApp_Config.HvacUiDisplayMode & DM_BLUE){
    scheme = 0;
  }else if (zclApp_Config.HvacUiDisplayMode & DM_RED){
    scheme = 1;
  }else if (zclApp_Config.HvacUiDisplayMode & DM_GREEN){
    scheme = 2;
  }
    
  LCD_SCAN_DIR Lcd_ScanDir = SCAN_DIR_DFT;    //SCAN_DIR_DFT = D2U_L2R
  LCD_Init( Lcd_ScanDir, 1);
  zclApp_SetTimeDate();

//  TP_Init( Lcd_ScanDir );
//  TP_GetAdFac(); // default calibration factor

  TftUpdateRefresh();

#endif    
#if defined(EPD3IN7)
  // check epd
  EpdReset();  
  uint8 error_time = 25; // over 2.5 sec return
  while(HAL_LCD_BUSY == 1) {      //LOW: idle, HIGH: busy
    _delay_ms(100);
    error_time = error_time - 1;
    if (error_time == 0){    
      EpdDetect = 0;
      break;
    }
  }   
    
  if (EpdDetect == 1) { 
    if (zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT){
      zclApp_color = 0xFF;
    } else {
      zclApp_color = 0x00;
    }
    // epd full screen
    EpdInitFull();

    EpdClearFrameMemory(zclApp_color);
    EpdDisplayFrame();
    EpdClearFrameMemory(zclApp_color);
    EpdDisplayFrame();
  
    // epd partial screen
    EpdInitPartial();
    EpdClearFrameMemory(zclApp_color);
    EpdDisplayFramePartial();
    EpdClearFrameMemory(zclApp_color);
    EpdDisplayFramePartial();

    zclApp_SetTimeDate();

    EpdRefresh();

  }
#endif  
    zclApp_StartReloadTimer();
    osal_start_reload_timer(zclApp_TaskID, APP_REPORT_CLOCK_EVT, 60000);
}

#ifdef MT_ZDO_MGMT
void zclApp_ProcessZDOMsgs(zdoIncomingMsg_t *InMsg){
  
  switch (InMsg->clusterID){
#ifdef IEEE_ADDR_REQ     
    case IEEE_addr_rsp:
      LREP("InMsg->clusterID=0x%X InMsg->asdu=%X\r\n", InMsg->clusterID, InMsg->asdu);
      {
        ZDO_NwkIEEEAddrResp_t *AddrRsp;
        
        AddrRsp = ZDO_ParseAddrRsp(InMsg);
        uint16 extAd = BUILD_UINT16(  AddrRsp->extAddr[0],AddrRsp->extAddr[1] );
        for (uint8 y = 0; y <= 2; y++ ){
          if (AddrRsp->nwkAddr == temp_Sender_shortAddr[y]) {
            temp_Sender_extAddr[y] = extAd;
            LREP("nwkAddr=%X extAddr=0x%X\r\n", AddrRsp->nwkAddr, extAd);
          }
        }
        
        osal_mem_free(AddrRsp);
      }
      break;
#endif
#ifdef BIND_REQ     
    case Mgmt_Bind_rsp:
      LREP("InMsg->clusterID=0x%X InMsg->asdu=%X\r\n", InMsg->clusterID, InMsg->asdu);
      {
//        HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK);
        uint8 bindingListCount = 0;
        
        ZDO_MgmtBindRsp_t *BindRsp;
        
        BindRsp = ZDO_ParseMgmtBindRsp(InMsg);
        bindingListCount = BindRsp->bindingListCount;
        
        uint8 bdev = temp_countReqBind;
        temp_bindingCount[bdev] = BindRsp->bindingCount;
        LREP("bc=%d bs=%d bl=%d ex=%X\r\n", BindRsp->bindingCount, BindRsp->startIndex, BindRsp->bindingListCount, temp_Sender_extAddr[temp_countReqBind]);
        
        for ( uint8 x = 0; x < bindingListCount; x++ ){
          uint16 sAd = BUILD_UINT16( BindRsp->list[x].srcAddr[0],BindRsp->list[x].srcAddr[1] );         
          for (uint8 d = 0; d <=2; d++ ) {
            if (sAd == temp_Sender_extAddr[d]) {
              bdev = d;
            }
          }
          LREP("%d %X %X %X\r\n", bdev, BindRsp->list[x].clusterID, sAd, BindRsp->list[x].dstAddr.addr.shortAddr);
          
          if (BindRsp->list[x].dstAddr.addr.shortAddr == zlcApp_ExtAddr) {
            // enable/disable display of values
            // bit 0 - 0x0001 POWER_CFG, 1 - 0x0400 ILLUMINANCE, 2 - 0x0402 TEMP,         3 - 0x0403 PRESSURE, 
            //     4 - 0x0405 HUMIDITY,  5 - 0x0406 OCCUPANCY,   6 - 0x000F BINARY_INPUT, 7 - table received
            switch (BindRsp->list[x].clusterID){
              case POWER_CFG:
                temp_bindClusterDev[bdev] |= CB_POWER_CFG;
                break;
              case ILLUMINANCE:
                temp_bindClusterDev[bdev] |= CB_ILLUMINANCE;
                break;
              case TEMP:
                temp_bindClusterDev[bdev] |= CB_TEMP;
                break;
              case PRESSURE:
                temp_bindClusterDev[bdev] |= CB_PRESSURE;
                break;
              case HUMIDITY:
                temp_bindClusterDev[bdev] |= CB_HUMIDITY;
                break;
              case OCCUPANCY:
                temp_bindClusterDev[bdev] |= CB_OCCUPANCY;
                break;
              case BINARY_INPUT:
                temp_bindClusterDev[bdev] |= CB_BINARY_INPUT;
                break;
              case ZCL_CO2:
                temp_bindClusterDev[bdev] |= CB_CO2;
                break;
            }           
          }
        }
        
        if (temp_bindingCount[bdev] != 0 && temp_bindingCount[bdev] - temp_bindingStartIndex[bdev] > 3){
//          LREPMaster("BIND_BIND_BIND\r\n");
          temp_bindingStartIndex[bdev] = temp_bindingStartIndex[bdev] +3;
          temp_countReqBind = bdev;
//          osal_start_timerEx(zclApp_TaskID, APP_REQ_BIND_EVT, 100);
          zclApp_RequestBind();
        } else {
          LREPMaster("BIND_BIND_0\r\n");
          temp_bindingCount[bdev] = 0;
          temp_bindingStartIndex[bdev] = 0;         
          if (temp_bindClusterDev[bdev] == 0x0000) {
            temp_Sender_shortAddr[bdev] = 0xFFFE;
            temp_LqiStartIndex[bdev] = 0;
            temp_lqi[bdev] = 255;
          } else {
            temp_bindClusterDev[bdev] |= CB_TABLE;
          }
          if (temp_bindClusterDev[bdev] != old_bindClusterDev[bdev]) {
            old_bindClusterDev[bdev] = temp_bindClusterDev[bdev];
          }
#if defined(EPD3IN7)
          EpdRefresh();
//          EpdtestRefresh(bdev);
#endif
#if defined(TFT3IN5)
//          TftRefresh();
          if (!zcl_game){
            TfttestRefresh(bdev);
          }
#endif          
        }        
        osal_mem_free(BindRsp);
        
      }
      break;
#endif      
#ifdef LQI_REQ
    case Mgmt_Lqi_rsp:
      LREP("InMsg->clusterID=0x%X InMsg->asdu=%X\r\n", InMsg->clusterID, InMsg->asdu);      
    {
      ZDO_MgmtLqiRsp_t *LqRsp;
      uint8 num;
      uint8 find = 0;
      uint8 i;
      uint8 EndDevice_Lqi;
      uint16 nwkDevAddr;

      LqRsp = ZDO_ParseMgmtLqiRsp(InMsg);

      num = LqRsp->neighborLqiCount;
      LREP("startIndex=%d neighborLqiCount=%d\r\n", LqRsp->startIndex, LqRsp->neighborLqiCount);
      if (num != 0) {
        for(i= 0; i < num && find == 0; i++)
        {
            nwkDevAddr = temp_Sender_shortAddr[temp_countReqLqi];
            
          if (LqRsp->list[i].nwkAddr == nwkDevAddr) {
            EndDevice_Lqi = LqRsp->list[i].lqi;
            temp_lqi[temp_countReqLqi] = EndDevice_Lqi;
            temp_LqiStartIndex[temp_countReqLqi] = LqRsp->startIndex;
            find = 1;
          } else {
            temp_LqiStartIndex[temp_countReqLqi] = temp_LqiStartIndex[temp_countReqLqi] + 1;
          }
          LREP("nwkAddr=0x%X lqi=%d\r\n", LqRsp->list[i].nwkAddr, LqRsp->list[i].lqi);
        }

      } else {
        temp_LqiStartIndex[temp_countReqLqi] = 0;
        temp_lqi[temp_countReqLqi] = 255;
      }
      osal_mem_free(LqRsp);      
#if defined(EPD3IN7)      
      EpdRefresh();
//      EpdLqi(temp_countReqLqi);
//      EpdLqi(temp_countReqLqi);
#endif
#if defined(TFT3IN5)      
//      TftRefresh();
      if (!zcl_game){
        TftLqi(temp_countReqLqi);
      }
#endif      
      if (temp_countReqLqi >=2){
        temp_countReqLqi = 0;
      } else {
        temp_countReqLqi = temp_countReqLqi + 1;
      }

    }

    break;
#endif
  }
  if (InMsg->asdu) {
//    LREP("osal_mem_free InMsg->asdu=%X\r\n", InMsg->asdu);
    osal_mem_free(InMsg->asdu);
  }
}
#endif

#ifdef LQI_REQ
void zclApp_RequestLqi(void){
  if ( bdbAttributes.bdbNodeIsOnANetwork ){
        zAddrType_t destAddr;
        uint8 startIndex;
        /* Dev address */
        destAddr.addrMode = Addr16Bit;
        destAddr.addr.shortAddr = _NIB.nwkCoordAddress; // Parent, Coordinator always address 0
        startIndex = temp_LqiStartIndex[temp_countReqLqi];
        ZDP_MgmtLqiReq(&destAddr,startIndex, 0);
        temp_lqi[temp_countReqLqi] = 255;
        LREP("REQ_sI=%d\r\n", temp_LqiStartIndex[temp_countReqLqi]);
  }
}
#endif

#ifdef IEEE_ADDR_REQ
void zclApp_RequestAddr(uint16 nwkaddr){
  if ( bdbAttributes.bdbNodeIsOnANetwork ){
//        zAddrType_t destAddr;
        uint8 startIndex;
        uint8 ReqType;
        /* Dev address */
//        destAddr.addrMode = Addr16Bit;
//        destAddr.addr.shortAddr = _NIB.nwkDevAddress; 
        startIndex = 0;
        ReqType = 0;
        if (nwkaddr != 0xFFFE) {
          ZDP_IEEEAddrReq(nwkaddr, ReqType, startIndex, 0);
          LREP("REQ_Addr=%X\r\n", nwkaddr);
        }
  }
}
#endif

#ifdef BIND_REQ
void zclApp_RequestBind(void){
  if ( bdbAttributes.bdbNodeIsOnANetwork ){
        zAddrType_t destAddr;
        uint8 startIndex;
        /* Dev address */
        destAddr.addrMode = Addr16Bit;
//        destAddr.addr.shortAddr = _NIB.nwkCoordAddress; // Parent, Coordinator always address 0
//        destAddr.addr.shortAddr = 0x4436; 
      if (temp_Sender_shortAddr[temp_countReqBind] != 0xFFFE) {
        temp_bindClusterDev[temp_countReqBind] &= ~CB_TABLE;
#if defined(TFT3IN5)
        if (!zcl_game){
          TftBindStatus(temp_countReqBind);
        }
#endif  
        destAddr.addr.shortAddr = temp_Sender_shortAddr[temp_countReqBind];      
        startIndex = temp_bindingStartIndex[temp_countReqBind];
        ZDP_MgmtBindReq(&destAddr,startIndex, 0);
        LREP("REQ_sAd=0x%X\r\n", temp_Sender_shortAddr[temp_countReqBind]);
      }
        
  }
}
#endif

#if defined(EPD3IN7)
static void zclApp_EpdUpdateClock(void) {
//        if (EpdDetect == 1) {
          if (zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT){
              zclApp_color = 0xFF;
          } else {
              zclApp_color = 0x00;
          }
            // epd full screen
          EpdInitFull();
          EpdClearFrameMemory(zclApp_color);
          EpdDisplayFrame();
          EpdClearFrameMemory(zclApp_color);
          EpdDisplayFrame();
            // epd partial screen
          EpdInitPartial();
          EpdClearFrameMemory(zclApp_color);
          EpdDisplayFramePartial();
          EpdClearFrameMemory(zclApp_color);
          EpdDisplayFramePartial();
          
//        }
        
}
#endif

uint16 zclApp_event_loop(uint8 task_id, uint16 events) {
  
//  TP_DrawBoard();
  
    afIncomingMSGPacket_t *MSGpkt;
    devStates_t zclApp_NwkState; //---
    
    (void)task_id; // Intentionally unreferenced parameter
    if (events & SYS_EVENT_MSG) {
        while ((MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive(zclApp_TaskID))) {          
            switch (MSGpkt->hdr.event) {
#ifdef MT_ZDO_MGMT
            case ZDO_CB_MSG: // ZDO incoming message callback
                zclApp_ProcessZDOMsgs((zdoIncomingMsg_t *)MSGpkt);
                LREP("MSGpkt->hdr.event=0x%X\r\n", MSGpkt->hdr.event);
                
              break;
#endif // MT_ZDO_MGMT
            case KEY_CHANGE:
                zclApp_HandleKeys(((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys);
                
                break;
            case ZDO_STATE_CHANGE: //devStates_t ZDApp.h             
                zclApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
                LREP("NwkState=%d\r\n", zclApp_NwkState);
                if (zclApp_NwkState == DEV_END_DEVICE) {
#ifdef LQI_REQ                  
//                  temp_LqiStartIndex[0] = 0;
//                  temp_lqi[0] = 255;
#if defined(EPD3IN7)                  
                  EpdRefresh();
//                  EpdStatus(0); // status network device
//                  EpdStatus(0); // status network device
#endif //  EPD3IN7 
#if defined(TFT3IN5)                  
//                  TftRefresh();
                  if (!zcl_game){
                    TftStatus(0); // update status network device
                  }
#endif //  TFT3IN5                  
#endif // LQI_REQ
                  IEN2 |= HAL_KEY_BIT4; // enable port1 int
                  P1DIR |=  BV(0); // P1_0 output
                  P1 |=  BV(0);   // power on DD
                } else {
                  IEN2 &= ~HAL_KEY_BIT4; // disable port1 int
                  P1 &= ~BV(0);   // power off DD //-- 
                  P1DIR &= ~BV(0); // P1_0 input
                }
                break;
            case ZCL_INCOMING_MSG:               
                zclApp_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
                
                if (((zclIncomingMsg_t *)MSGpkt)->attrCmd) {
                    osal_mem_free(((zclIncomingMsg_t *)MSGpkt)->attrCmd);
                }
                break;

            default:
                break;
            }
            // Release the memory
            osal_msg_deallocate((uint8 *)MSGpkt);
        }
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }
#ifdef BIND_REQ 
/*    
    if (events & APP_REQ_BIND_EVT) {
        LREPMaster("APP_REQ_BIND_EVT\r\n");
        zclApp_RequestBind();
        return (events ^ APP_REQ_BIND_EVT);
    }
*/
#endif 
    if (events & APP_REPORT_CLOCK_EVT) {
      LREPMaster("APP_REPORT_CLOCK_EVT\r\n");
      
      //Fix osalclock bug 88 min in 15 days
      osalTimeUpdate();
      zclApp_GenTime_TimeUTC = osal_getClock();
      if ((zclApp_GenTime_TimeUTC - zclApp_GenTime_old) > 70){ //if the interval is more than 70 seconds, then adjust the time
        zclApp_LocalTime(); //report
        // Update OSAL time
        osal_setClock(zclApp_GenTime_old + 60);
        osalTimeUpdate();
        zclApp_GenTime_TimeUTC = osal_getClock();
        zclApp_LocalTime(); //report
      }
      zclApp_GenTime_old = zclApp_GenTime_TimeUTC;
      
//      if(zclApp_Occupied == 1 || bdbAttributes.bdbNodeIsOnANetwork == 0) {  
//        osalTimeUpdate();
//        zclApp_GenTime_TimeUTC = osal_getClock();                
#ifdef LQI_REQ        
        zclApp_RequestLqi();
#endif 
#ifdef BIND_REQ        
        if (temp_countReqBind >=2) {
          temp_countReqBind = 0;
          temp_bindingCount[temp_countReqBind] = 0;
          temp_bindingStartIndex[temp_countReqBind] = 0;
        } else { 
          temp_countReqBind = temp_countReqBind +1;
          temp_bindingCount[temp_countReqBind] = 0;
          temp_bindingStartIndex[temp_countReqBind] = 0;
        }
        temp_bindClusterDev[temp_countReqBind] = 0x0000;
//        osal_start_timerEx(zclApp_TaskID, APP_REQ_BIND_EVT, 100);
        zclApp_RequestBind();
#endif 
#if defined(EPD3IN7)        
        fullupdate_hour = fullupdate_hour +1;
        if (fullupdate_hour == 5){ // over 5 min clear          
          zclApp_EpdUpdateClock();          
          fullupdate_hour = 0;
/*          
          for(uint8 i = 0; i <= 2; i++ ){
            EpdtestRefresh(i);
          }
*/          
        }       
        EpdRefresh();
#endif // EPD3IN7
#if defined(TFT3IN5)        
//        TftRefresh();
        if (!zcl_game && zclApp_menu == 0){
          TftTimeDateWeek(); // update time, date, weekday
        }
#endif        
//      }
        return (events ^ APP_REPORT_CLOCK_EVT);
    }

    if (events & APP_REPORT_TEMPERATURE_EVT) {
        LREPMaster("APP_REPORT_TEMPERATURE_EVT\r\n");
        report = 0;
        zclApp_ReadBME280Temperature();
        
        return (events ^ APP_REPORT_TEMPERATURE_EVT);
    }
    
    if (events & APP_REPORT_PRESSURE_EVT) {
        LREPMaster("APP_REPORT_PRESSURE_EVT\r\n");
        report = 0;
        zclApp_ReadBME280Pressure();
   
        return (events ^ APP_REPORT_PRESSURE_EVT);
    }
    
    if (events & APP_REPORT_HUMIDITY_EVT) {
        LREPMaster("APP_REPORT_HUMIDITY_EVT\r\n");
        report = 0;
        zclApp_ReadBME280Humidity();
   
        return (events ^ APP_REPORT_HUMIDITY_EVT);
    }
    
    if (events & APP_REPORT_ILLUMINANCE_EVT) {
        LREPMaster("APP_REPORT_ILLUMINANCE_EVT\r\n");       
        report = 0;
        zclApp_bh1750StartLumosity();
           
        return (events ^ APP_REPORT_ILLUMINANCE_EVT);
    }
    
    if (events & APP_REPORT_CO2_EVT) {
        LREPMaster("APP_REPORT_CO2_EVT\r\n");       
        report = 0;
        zclApp_scd4xReadCO2();
           
        return (events ^ APP_REPORT_CO2_EVT);
    }
   
    if (events & APP_REPORT_BATTERY_EVT) {
        LREPMaster("APP_REPORT_BATTERY_EVT\r\n");
        report = 0;
        zclBattery_Report();

        return (events ^ APP_REPORT_BATTERY_EVT);
    }
    
    if (events & APP_REPORT_EVT) {
        LREPMaster("APP_REPORT_EVT\r\n");
        report = 1;
        zclApp_Report();
        return (events ^ APP_REPORT_EVT);
    }

    if (events & APP_READ_SENSORS_EVT) {
//        LREPMaster("APP_READ_SENSORS_EVT\r\n");
        zclApp_ReadSensors();
        return (events ^ APP_READ_SENSORS_EVT);
    }
        
    if (events & APP_MOTION_ON_EVT) {
        LREPMaster("APP_MOTION_ON_EVT\r\n");
        P1 &= ~BV(0);   // power off motion
//        BNAME(MOTION_POWER_PORT, MOTION_POWER_PIN)= 0; //power off motion
        osal_start_timerEx(zclApp_TaskID, APP_MOTION_DELAY_EVT, (uint32)zclApp_Config.PirOccupiedToUnoccupiedDelay * 1000);
        LREPMaster("MOTION_START_DELAY\r\n");
        //report
        if (zclApp_Occupied == 0) {
          zclApp_Occupied = 1;        
        }
        zclApp_Occupied = 1;
        zclApp_Occupied_OnOff = 1;
        zclGeneral_SendOnOff_CmdOn(zclApp_ThirdEP.EndPoint, &inderect_DstAddr, TRUE, bdb_getZCLFrameCounter());
//        bdb_RepChangedAttrValue(zclApp_ThirdEP.EndPoint, OCCUPANCY, ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY);
        zclRep_Occupancy();
        
        return (events ^ APP_MOTION_ON_EVT);
    }
    
    if (events & APP_MOTION_OFF_EVT) {
        LREPMaster("APP_MOTION_OFF_EVT\r\n");
        //report
        zclApp_Occupied = 0;
        zclApp_Occupied_OnOff = 0;
        zclGeneral_SendOnOff_CmdOff(zclApp_ThirdEP.EndPoint, &inderect_DstAddr, TRUE, bdb_getZCLFrameCounter());
//        bdb_RepChangedAttrValue(zclApp_ThirdEP.EndPoint, OCCUPANCY, ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY);
        zclRep_Occupancy();       
        return (events ^ APP_MOTION_OFF_EVT);
    }
    
    if (events & APP_MOTION_DELAY_EVT) {
        LREPMaster("APP_MOTION_DELAY_EVT\r\n");
        power = 2;       
        P1DIR |=  BV(0); // P1_0 output
//        IO_DIR_PORT_PIN(MOTION_POWER_PORT, MOTION_POWER_PIN, IO_OUT); // P1_0 output
        P1 |=  BV(0);   // power on motion
//        BNAME(MOTION_POWER_PORT, MOTION_POWER_PIN)= 1; // power on motion
        
        return (events ^ APP_MOTION_DELAY_EVT);
    }
#if defined(EPD3IN7)
    if (events & APP_EPD_DELAY_EVT) {
        LREPMaster("APP_EPD_DELAY_EVT\r\n");
        EpdtestRefresh();
        
        return (events ^ APP_EPD_DELAY_EVT);
    }
#endif // EPD3IN7
    if (events & APP_BH1750_DELAY_EVT) {
        LREPMaster("APP_BH1750_DELAY_EVT\r\n");
        zclApp_bh1750ReadLumosity();
        
        return (events ^ APP_BH1750_DELAY_EVT);
    }
    
    if (events & APP_SAVE_ATTRS_EVT) {
        LREPMaster("APP_SAVE_ATTRS_EVT\r\n");
        zclApp_SaveAttributesToNV();
        
        return (events ^ APP_SAVE_ATTRS_EVT);
    }

    // Discard unknown events
    return 0;
}

static void zclApp_HandleKeys(byte portAndAction, byte keyCode) {
//    LREP("zclApp_HandleKeys portAndAction=0x%X keyCode=0x%X\r\n", portAndAction, keyCode);
#if APP_COMMISSIONING_BY_LONG_PRESS == TRUE
    if (bdbAttributes.bdbNodeIsOnANetwork == 1) {
      zclFactoryResetter_HandleKeys(portAndAction, keyCode);
    }
#else
    zclFactoryResetter_HandleKeys(portAndAction, keyCode);
#endif
    zclCommissioning_HandleKeys(portAndAction, keyCode); 
#if defined(TFT3IN5)    
    xpt2046_HandleKeys(portAndAction, keyCode);
#endif    
    if (portAndAction & HAL_KEY_PRESS) {
//        LREPMaster("Key press\r\n");
    }

    bool contact = portAndAction & HAL_KEY_PRESS ? TRUE : FALSE;
    if (portAndAction & HAL_KEY_PORT0) {
        LREPMaster("Key press PORT0\r\n");

    } else if (portAndAction & HAL_KEY_PORT1) {     
        LREPMaster("Key press PORT1\r\n");
        if (!contact) {
          P2INP |= HAL_KEY_BIT6;  // pull down
        } else {
          P2INP &= ~HAL_KEY_BIT6; // pull up
        }
        if (power == 0){
          if (contact) {
            osal_start_timerEx(zclApp_TaskID, APP_MOTION_ON_EVT, 100);
            osal_stop_timerEx(zclApp_TaskID, APP_MOTION_OFF_EVT);
            osal_clear_event(zclApp_TaskID, APP_MOTION_OFF_EVT);
            
          }
        } else {
          if (power == 1){
            //end adaptive motion
            osal_start_timerEx(zclApp_TaskID, APP_MOTION_OFF_EVT, (uint32)zclApp_Config.PirUnoccupiedToOccupiedDelay * 1000); 
          }
          power = power - 1;
        }
        LREP("power=%d\r\n", power);
     } else if (portAndAction & HAL_KEY_PORT2) {
       LREPMaster("Key press PORT2\r\n");
       if (contact) {
//          HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK);
//          IEN2 &= ~HAL_KEY_BIT4; // disable port1 int
          osal_start_timerEx(zclApp_TaskID, APP_REPORT_EVT, 200);
       } else {
//          IEN2 |= HAL_KEY_BIT4; // enable port1 int
       }
     }
     LREP("contact=%d\r\n", contact);
     uint16 alarmStatus = 0;
     if (!contact) {
        alarmStatus |= BV(0);
     } 
}

static void zclApp_ReadSensors(void) {
  zclApp_StopReloadTimer();
  LREP("currentSensorsReadingPhase %d\r\n", currentSensorsReadingPhase);
    /**
     * FYI: split reading sensors into phases, so single call wouldn't block processor
     * for extensive ammount of time
     * */
  if (report == 1) {
    switch (currentSensorsReadingPhase++) {
    case 0:
//        HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK);    
      zclBattery_Report();      
        break;
    case 4:
      if (scd4xDetect == 1) {
        zclApp_scd4xReadCO2();
      }
          break;
    case 1:
      if (bmeDetect == 1){
          zclApp_ReadBME280Temperature();
      }
        break;
    case 2:
      if (bmeDetect == 1){
          zclApp_ReadBME280Pressure();
      }
        break;
    case 3:
      if (bmeDetect == 1){
          zclApp_ReadBME280Humidity();
      }
        break;
    case 5:
      if (bh1750Detect == 1){
        osal_stop_timerEx(zclApp_TaskID, APP_BH1750_DELAY_EVT);
        osal_clear_event(zclApp_TaskID, APP_BH1750_DELAY_EVT);
        zclApp_bh1750StartLumosity();
      }      
        break;
     case 6:
       zclApp_ConfigDisplay();

        break;
     case 7:
       zclApp_LocalTime();
        break;
    case 8:
#ifdef LQI_REQ
       temp_countReqLqi = 0;
       zclApp_RequestLqi();
#endif        
        break;
    case 9: 
#ifdef BIND_REQ
//      temp_countReqBind = 0;
//      temp_bindingCount[temp_countReqBind] = 0;
//      temp_bindingStartIndex[temp_countReqBind] = 0;
//      temp_bindClusterDev[temp_countReqBind] = 0x0000;
//      osal_start_timerEx(zclApp_TaskID, APP_REQ_BIND_EVT, 100);
//      zclApp_RequestBind();
#endif
        break;
    default:
        osal_stop_timerEx(zclApp_TaskID, APP_READ_SENSORS_EVT);
        osal_clear_event(zclApp_TaskID, APP_READ_SENSORS_EVT);
        currentSensorsReadingPhase = 0;
        break;
    }
  }
  zclApp_StartReloadTimer();
}

static void zclApp_scd4xReadCO2(void) {
//        LCD_SetArealColorWH(100, 0, 200, 16, WHITE);        
        uint16 scd4x_co2_16 = (uint32)SCD4x_getCO2();
//        GUI_DisNumDP(100, 0, scd4x_co2_16, 0, &Font16, WHITE, BLACK );
//        GUI_DisNumDP(150, 0, old_scd4xCO2_Sensor_MeasuredValue, 0, &Font16, WHITE, BLACK );
        
        float scd4x_co2 = (float)SCD4x_getCO2()/ 1000000.0;
        zclApp_scd4xCO2Sensor_MeasuredValue = scd4x_co2;
        
        uint16 co2 = 0;
        if (old_scd4xCO2_Sensor_MeasuredValue > scd4x_co2_16){
          co2 = (old_scd4xCO2_Sensor_MeasuredValue - scd4x_co2_16);
        } else {
          co2 = (scd4x_co2_16 - old_scd4xCO2_Sensor_MeasuredValue);
        }
        if (co2 > zclApp_Config.CO2MinAbsoluteChange || report == 1){ // 100 
          old_scd4xCO2_Sensor_MeasuredValue = scd4x_co2_16;         
          zclRep_CO2Report();
        } 
}

static void zclApp_ConfigDisplay(void) {
//  bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, HVAC_UI_CONFIG, ATTRID_HVAC_THERMOSTAT_UI_CONFIG_DISPLAY_MODE);
  zclRep_ConfigDisplay();
}

static void zclApp_LocalTime(void) {
//  bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, GEN_TIME, ATTRID_TIME_LOCAL_TIME);
  zclRep_LocalTime();
}

static void zclApp_bh1750StartLumosity(void) {
        bh1850_Write(BH1750_POWER_ON);        
        bh1850_Write(BH1750_mode);
   
        if (BH1750_mode == CONTINUOUS_LOW_RES_MODE || BH1750_mode == ONE_TIME_LOW_RES_MODE) {
          osal_start_timerEx(zclApp_TaskID, APP_BH1750_DELAY_EVT, 30);
        } else {
          osal_start_timerEx(zclApp_TaskID, APP_BH1750_DELAY_EVT, 180);
        }
}

static void zclApp_bh1750ReadLumosity(void) {
    zclApp_bh1750IlluminanceSensor_MeasuredValue = (uint16)(bh1850_Read());
    bh1850_PowerDown();
        
    uint16 illum = 0;
    if (old_bh1750IlluminanceSensor_MeasuredValue > zclApp_bh1750IlluminanceSensor_MeasuredValue){
      illum = (old_bh1750IlluminanceSensor_MeasuredValue - zclApp_bh1750IlluminanceSensor_MeasuredValue);
    } else {
      illum = (zclApp_bh1750IlluminanceSensor_MeasuredValue - old_bh1750IlluminanceSensor_MeasuredValue);
    }
    if (illum > zclApp_Config.MsIlluminanceMinAbsoluteChange || report == 1){ // 10 lux
      old_bh1750IlluminanceSensor_MeasuredValue = zclApp_bh1750IlluminanceSensor_MeasuredValue;
//      bdb_RepChangedAttrValue(zclApp_FourthEP.EndPoint, ILLUMINANCE, ATTRID_MS_ILLUMINANCE_MEASURED_VALUE);
      zclRep_IlluminanceReport();
    }
#ifdef HAL_LCD_PWM_PORT1
    uint8 levelPWM = 0;
    if (zclApp_bh1750IlluminanceSensor_MeasuredValue > MAX_LEVEL_PWM){
      levelPWM = MAX_LEVEL_PWM;
    } else {
      levelPWM = (uint8)zclApp_bh1750IlluminanceSensor_MeasuredValue;
    }
    if (zclApp_Config.HvacUiDisplayMode & DM_BACKLIGHT){
      levelPWM = MAX_LEVEL_PWM;
    }
    InitLedPWM(254-levelPWM);    
#endif
    LREP("bh1750IlluminanceSensor_MeasuredValue value=%X\r\n", zclApp_bh1750IlluminanceSensor_MeasuredValue);
}

static void zclApp_ReadBME280Temperature(void) {
        bme280_takeForcedMeasurement();
        zclApp_Temperature_Sensor_MeasuredValue = (int16)(bme280_readTemperature() *100);
//        LREP("Temperature=%d\r\n", zclApp_Temperature_Sensor_MeasuredValue);
        
        uint16 temp = 0;
        if (old_Temperature_Sensor_MeasuredValue > zclApp_Temperature_Sensor_MeasuredValue){
          temp = (old_Temperature_Sensor_MeasuredValue - zclApp_Temperature_Sensor_MeasuredValue);
        } else {
          temp = (zclApp_Temperature_Sensor_MeasuredValue - old_Temperature_Sensor_MeasuredValue);
        }
        if (temp > zclApp_Config.MsTemperatureMinAbsoluteChange || report == 1){ //50 - 0.5 
          old_Temperature_Sensor_MeasuredValue = zclApp_Temperature_Sensor_MeasuredValue;         
//          bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, TEMP, ATTRID_MS_TEMPERATURE_MEASURED_VALUE);
          zclRep_BME280TemperatureReport();
        }        
}

static void zclApp_ReadBME280Pressure(void) {
        bme280_takeForcedMeasurement();
        zclApp_PressureSensor_ScaledValue = (int16) (pow(10.0, (double) zclApp_PressureSensor_Scale) * (double) bme280_readPressure()* 100);

        zclApp_PressureSensor_MeasuredValue = (uint16)bme280_readPressure();
//        LREP("Pressure=%d\r\n", zclApp_PressureSensor_MeasuredValue);
                
        uint16 press = 0;
        if (old_PressureSensor_MeasuredValue > zclApp_PressureSensor_MeasuredValue){
          press = (old_PressureSensor_MeasuredValue - zclApp_PressureSensor_MeasuredValue);
        } else {
          press = (zclApp_PressureSensor_MeasuredValue - old_PressureSensor_MeasuredValue);
        }
        if (press > zclApp_Config.MsPressureMinAbsoluteChange || report == 1){ //1gPa
          old_PressureSensor_MeasuredValue = zclApp_PressureSensor_MeasuredValue;
          old_PressureSensor_ScaledValue = zclApp_PressureSensor_ScaledValue;
//          bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, PRESSURE, ATTRID_MS_PRESSURE_MEASUREMENT_MEASURED_VALUE);
          zclRep_BME280PressureReport();
        }
        if (scd4xDetect == 1){
          SCD4x_setAmbientPressure((uint32)bme280_readPressure()*100, 1);
        }
}

static void zclApp_ReadBME280Humidity(void) {
        bme280_takeForcedMeasurement();
        zclApp_HumiditySensor_MeasuredValue = (uint16)(bme280_readHumidity() * 100);
//        LREP("Humidity=%d\r\n", zclApp_HumiditySensor_MeasuredValue);
                
        uint16 humid = 0;
        if (old_HumiditySensor_MeasuredValue > zclApp_HumiditySensor_MeasuredValue){
          humid = (old_HumiditySensor_MeasuredValue - zclApp_HumiditySensor_MeasuredValue);
        } else {
          humid = (zclApp_HumiditySensor_MeasuredValue - old_HumiditySensor_MeasuredValue);
        }
        if (humid > zclApp_Config.MsHumidityMinAbsoluteChange || report == 1 ){ //10%
          old_HumiditySensor_MeasuredValue = zclApp_HumiditySensor_MeasuredValue;
//          bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, HUMIDITY, ATTRID_MS_RELATIVE_HUMIDITY_MEASURED_VALUE);
          zclRep_BME280HumidityReport();
        }
}

static void zclApp_Report(void) { osal_start_reload_timer(zclApp_TaskID, APP_READ_SENSORS_EVT, 200); }

static void zclApp_BasicResetCB(void) {
    LREPMaster("BasicResetCB\r\n");
    zclApp_ResetAttributesToDefaultValues();
    zclApp_SaveAttributesToNV();
}

static ZStatus_t zclApp_ReadWriteAuthCB(afAddrType_t *srcAddr, zclAttrRec_t *pAttr, uint8 oper) {
    LREPMaster("AUTH CB called\r\n");

    osal_start_timerEx(zclApp_TaskID, APP_SAVE_ATTRS_EVT, 2000);
    return ZSuccess;
}

static void zclApp_SaveAttributesToNV(void) {
    uint8 writeStatus = osal_nv_write(NW_APP_CONFIG, 0, sizeof(application_config_t), &zclApp_Config);
    LREP("Saving attributes to NV write=%d\r\n", writeStatus);
    
    zclApp_GenTime_old = zclApp_GenTime_TimeUTC;    
    osal_setClock(zclApp_GenTime_TimeUTC);
    
    if(scd4xDetect == 1 && zclApp_scd4xCO2Sensor_ForcedRecalibration == 1){
      float *correction = 0;
      SCD4x_performForcedRecalibrationArg(0x01e0, correction);
      zclApp_scd4xCO2Sensor_ForcedRecalibration = 0;
    }
    
#if defined(EPD3IN7)     
//    zclApp_EpdUpdateClock();  
//    EpdTimeDateWeek(); // update time, date, weekday
//    EpdTimeDateWeek(); // update time, date, weekday
    EpdRefresh();
#endif // EPD3IN7
#if defined(TFT3IN5)    
    if (!zcl_game){
//      TftTimeDateWeek(); // time, date, weekday
      TftUpdateRefresh();
    }
#endif // TFT3IN5
    zclApp_bh1750setMTreg();
    zclApp_StopReloadTimer();
    zclApp_StartReloadTimer();
}

static void zclApp_StopReloadTimer(void) {
    osal_stop_timerEx(zclApp_TaskID, APP_REPORT_BATTERY_EVT);
    osal_clear_event(zclApp_TaskID, APP_REPORT_BATTERY_EVT);    
  
    if (bmeDetect == 1){
      osal_stop_timerEx(zclApp_TaskID, APP_REPORT_TEMPERATURE_EVT);
      osal_clear_event(zclApp_TaskID, APP_REPORT_TEMPERATURE_EVT);
      osal_stop_timerEx(zclApp_TaskID, APP_REPORT_PRESSURE_EVT);
      osal_clear_event(zclApp_TaskID, APP_REPORT_PRESSURE_EVT);
      osal_stop_timerEx(zclApp_TaskID, APP_REPORT_HUMIDITY_EVT);
      osal_clear_event(zclApp_TaskID, APP_REPORT_HUMIDITY_EVT);
    }
    if (bh1750Detect == 1){
      osal_stop_timerEx(zclApp_TaskID, APP_REPORT_ILLUMINANCE_EVT);
      osal_clear_event(zclApp_TaskID, APP_REPORT_ILLUMINANCE_EVT);
    }
    if (scd4xDetect == 1){
      osal_stop_timerEx(zclApp_TaskID, APP_REPORT_CO2_EVT);
      osal_clear_event(zclApp_TaskID, APP_REPORT_CO2_EVT);
    }
}

static void zclApp_StartReloadTimer(void) {
  if (zclApp_Config.CfgBatteryPeriod != 0) {
    osal_start_reload_timer(zclApp_TaskID, APP_REPORT_BATTERY_EVT, (uint32)zclApp_Config.CfgBatteryPeriod * 60000);
  }  
  if (bmeDetect == 1){
    if (zclApp_Config.MsTemperaturePeriod != 0) {
      osal_start_reload_timer(zclApp_TaskID, APP_REPORT_TEMPERATURE_EVT, (uint32)zclApp_Config.MsTemperaturePeriod * 1000);
    }
    if (zclApp_Config.MsPressurePeriod != 0) {
      osal_start_reload_timer(zclApp_TaskID, APP_REPORT_PRESSURE_EVT, (uint32)zclApp_Config.MsPressurePeriod * 1000);
    }
    if (zclApp_Config.MsHumidityPeriod != 0) {
      osal_start_reload_timer(zclApp_TaskID, APP_REPORT_HUMIDITY_EVT, (uint32)zclApp_Config.MsHumidityPeriod * 1000);
    }
  }
  if (bh1750Detect == 1){
    if (zclApp_Config.MsIlluminancePeriod != 0) {
      osal_start_reload_timer(zclApp_TaskID, APP_REPORT_ILLUMINANCE_EVT, (uint32)zclApp_Config.MsIlluminancePeriod * 1000);
    }
  } 
  if (scd4xDetect == 1){
    if (zclApp_Config.CO2Period != 0) {
      osal_start_reload_timer(zclApp_TaskID, APP_REPORT_CO2_EVT, (uint32)zclApp_Config.CO2Period * 1000);
    }
  } 
}

static void zclApp_RestoreAttributesFromNV(void) {
    uint8 status = osal_nv_item_init(NW_APP_CONFIG, sizeof(application_config_t), NULL);
    LREP("Restoring attributes from NV  status=%d \r\n", status);
    if (status == NV_ITEM_UNINIT) {
        uint8 writeStatus = osal_nv_write(NW_APP_CONFIG, 0, sizeof(application_config_t), &zclApp_Config);
        LREP("NV was empty, writing %d\r\n", writeStatus);
    }
    if (status == ZSUCCESS) {
        LREPMaster("Reading from NV\r\n");
        osal_nv_read(NW_APP_CONFIG, 0, sizeof(application_config_t), &zclApp_Config);
    }
}

static void zclApp_bh1750setMTreg(void) {
    if (bh1750Detect == 1){
      uint8 MTreg = (uint8)zclApp_Config.MsIlluminanceLevelSensingSensitivity;
      bh1750_setMTreg(MTreg);
    }
}

#if defined(EPD3IN7)
static void EpdRefresh(void){
  if (EpdDetect == 1) {
//    if(zclApp_Occupied == 1 || bdbAttributes.bdbNodeIsOnANetwork == 0) {
      osal_start_timerEx(zclApp_TaskID, APP_EPD_DELAY_EVT, 2000);
//    }
  }
}
#endif // EPD3IN7

#if defined(TFT3IN5) 
/*
static void TftRefresh(void){
//  if (EpdDetect == 1) {
    if(zclApp_Occupied == 1 || bdbAttributes.bdbNodeIsOnANetwork == 0) {
      osal_start_timerEx(zclApp_TaskID, APP_TFT_DELAY_EVT, 2000);
    }
//  }
}
*/
#endif // TFT3IN5

#if defined(EPD3IN7)
static void _delay_us(uint16 microSecs)
{
  while(microSecs--)
  {
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
  }
}

static void _delay_ms(uint16 milliSecs)
{
  while(milliSecs--)
  {
    _delay_us(1000);
  }
}
#endif // EPD3IN7

#if defined(TFT3IN5)
static void TftTimeDateWeek(void){
  // clock init Firmware build date 20/08/2021 13:47
  // Update RTC and get new clock values
  osalTimeUpdate();
  UTCTimeStruct time;
  osal_ConvertUTCTime(&time, osal_getClock());
  uint16 foreground = WHITE;

  char time_string[] = {'0', '0', ':', '0', '0', '\0'};
  time_string[0] = time.hour / 10 % 10 + '0';
  time_string[1] = time.hour % 10 + '0';
  time_string[3] = time.minutes / 10 % 10 + '0';
  time_string[4] = time.minutes % 10 + '0';

  GUI_DrawRectangle(100, 16, 220, 64, zclApp_lcd_background, DRAW_FULL , DOT_PIXEL_DFT );
  GUI_DisString_EN(100, 16, time_string, &Font48, zclApp_lcd_background, foreground);
  
  // covert UTCTimeStruct date and month to display
  time.day = time.day + 1;
  time.month = time.month + 1;  
  char date_string[] = {'0', '0', '.', '0', '0', '.', '0', '0', '\0'};
  date_string[0] = time.day /10 % 10  + '0';
  date_string[1] = time.day % 10 + '0';
  date_string[3] = time.month / 10 % 10 + '0';
  date_string[4] = time.month % 10 + '0';
  date_string[6] = time.year / 10 % 10 + '0';
  date_string[7] = time.year % 10 + '0';

  GUI_DrawRectangle(116, 64, 212, 80, zclApp_lcd_background, DRAW_FULL , DOT_PIXEL_DFT );
  GUI_DisString_EN(116, 64, date_string, &Font16, zclApp_lcd_background, foreground);

  uint8 day_week = (uint16)floor((float)(zclApp_GenTime_TimeUTC/86400)) % 7;
  char* day_string = "";
  if (day_week == 5) {
    day_string = "Thursday";
  } else if (day_week == 6) {
    day_string = " Friday ";
  } else if (day_week == 0) {
    day_string = "Saturday";
  } else if (day_week == 1) {
    day_string = " Sunday";
  } else if (day_week == 2) {
    day_string = " Monday";
  } else if (day_week == 3) {
    day_string = "Tuesday";
  } else if (day_week == 4) {
    day_string = "Wednesday";
  }
  
  GUI_DrawRectangle(116, 80, 224, 96, zclApp_lcd_background, DRAW_FULL , DOT_PIXEL_DFT );
  GUI_DisString_EN(116, 80, day_string, &Font16, zclApp_lcd_background, foreground);
}
#endif

#if defined(EPD3IN7)
static void EpdTimeDateWeek(void){
//  EpdReset(); //disable sleep EPD
//  PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
  
  // clock init Firmware build date 20/08/2021 13:47
  // Update RTC and get new clock values
  osalTimeUpdate();
  UTCTimeStruct time;
  osal_ConvertUTCTime(&time, osal_getClock());

  char time_string[] = {'0', '0', ':', '0', '0', '\0'};
  time_string[0] = time.hour / 10 % 10 + '0';
  time_string[1] = time.hour % 10 + '0';
  time_string[3] = time.minutes / 10 % 10 + '0';
  time_string[4] = time.minutes % 10 + '0';

  if (zclApp_Config.HvacUiDisplayMode & DM_ROTATE){ // portrait
    PaintSetWidth(120);
    PaintSetHeight(48);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 4, time_string, &Font48, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 72, 16, PaintGetWidth(), PaintGetHeight());      
  } else { // landscape
    PaintSetWidth(48);
    PaintSetHeight(120);
    PaintSetRotate(ROTATE_90);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 4, time_string, &Font48, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 80, 40, PaintGetWidth(), PaintGetHeight());
  }
  
  // covert UTCTimeStruct date and month to display
  time.day = time.day + 1;
  time.month = time.month + 1;  
  char date_string[] = {'0', '0', '.', '0', '0', '.', '0', '0', '\0'};
  date_string[0] = time.day /10 % 10  + '0';
  date_string[1] = time.day % 10 + '0';
  date_string[3] = time.month / 10 % 10 + '0';
  date_string[4] = time.month % 10 + '0';
  date_string[6] = time.year / 10 % 10 + '0';
  date_string[7] = time.year % 10 + '0';

  if (zclApp_Config.HvacUiDisplayMode & DM_ROTATE){ // portrait
    PaintSetWidth(88);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 4, date_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 88, 64, PaintGetWidth(), PaintGetHeight());
  } else { //landscape
    PaintSetWidth(16);
    PaintSetHeight(136);
    PaintSetRotate(ROTATE_90);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 4, date_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 64, 57, PaintGetWidth(), PaintGetHeight());
  }

  uint8 day_week = (uint16)floor((float)(zclApp_GenTime_TimeUTC/86400)) % 7;
  char* day_string = "";
  if (day_week == 5) {
    day_string = "Thursday";
  } else if (day_week == 6) {
    day_string = " Friday ";
  } else if (day_week == 0) {
    day_string = "Saturday";
  } else if (day_week == 1) {
    day_string = " Sunday";
  } else if (day_week == 2) {
    day_string = " Monday";
  } else if (day_week == 3) {
    day_string = "Tuesday";
  } else if (day_week == 4) {
    day_string = "Wednesday";
  }
  
  if (zclApp_Config.HvacUiDisplayMode & DM_ROTATE){ // portrait
    PaintSetWidth(99);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, day_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 88, 80, PaintGetWidth(), PaintGetHeight());
  } else { //landscape
    PaintSetWidth(16);
    PaintSetHeight(136);
    PaintSetRotate(ROTATE_90);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, day_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 48, 57, PaintGetWidth(), PaintGetHeight());
  }
//  EpdRefresh();
//  EpdDisplayFramePartial();
//  EpdSleep();
}
#endif

#if defined(TFT3IN5)
static void TftStatus(uint8 temp_s){
  uint8 row = temp_s *120;
  
  //status network
  if ( bdbAttributes.bdbNodeIsOnANetwork ){
      GUI_DrawRectangle(8, 8, 24, 24, zclApp_lcd_background, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_Disbitmap(8, 8 + row , IMAGE_ONNETWORK, 16, 16, BLUE, 0);
  } else {
      GUI_DrawRectangle(8, 8, 24, 24, zclApp_lcd_background, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_Disbitmap(8, 8 + row , IMAGE_OFFNETWORK, 16, 16, BLUE, 0);
  } 
}
#endif

#if defined(EPD3IN7)
static void EpdStatus(uint8 temp_s){
//  EpdReset(); //disable sleep EPD
//  PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
  
  uint8 row = temp_s *120;
  
  //status network
  if (zclApp_Config.HvacUiDisplayMode & DM_ROTATE){ // portrait
    
    PaintSetWidth(16);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_90);
    PaintSetInvert(!(zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT));
    PaintClear(UNCOLORED);
    if ( bdbAttributes.bdbNodeIsOnANetwork ){
      PaintDrawImage(IMAGE_ONNETWORK, 0, 0, 16, 16, COLORED);
//      EpdSetFrameMemoryXY(PaintGetImage(), 8, 128 + row, PaintGetWidth(), PaintGetHeight());
      EpdSetFrameMemoryXY(PaintGetImage(), 8, 8 + row, PaintGetWidth(), PaintGetHeight());
    } else {
      PaintDrawImage(IMAGE_OFFNETWORK, 0, 0, 16, 16, COLORED);
//      EpdSetFrameMemoryXY(PaintGetImage(), 8, 128 + row, PaintGetWidth(), PaintGetHeight());   
      EpdSetFrameMemoryXY(PaintGetImage(), 8, 8 + row, PaintGetWidth(), PaintGetHeight());
    }
    PaintSetInvert(zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);    
  } else { // landscape    
    if ( bdbAttributes.bdbNodeIsOnANetwork ){
      EpdSetFrameMemoryImageXY(IMAGE_ONNETWORK, 264, 1, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_OFFNETWORK, 264, 1, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    }
    PaintSetWidth(8);
    PaintSetHeight(480);
    PaintSetRotate(ROTATE_90); 
    PaintClear(UNCOLORED);
    PaintDrawHorizontalLine(0, 4, 480, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 248, 0, PaintGetWidth(), PaintGetHeight());
    EpdSetFrameMemoryXY(PaintGetImage(), 4, 0, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(240);
    PaintSetHeight(8);
    PaintSetRotate(ROTATE_90); 
    PaintClear(UNCOLORED);  
    PaintDrawVerticalLine(0, 0, 240, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 8, 216, PaintGetWidth(), PaintGetHeight());    
  }
//  EpdDisplayFramePartial();
//  EpdSleep();
}
#endif

#if defined(TFT3IN5)
static void TftBindStatus(uint8 temp_s){
    //status bind
  uint8 row = 0;
  uint8 col = 0;
  char* bind_string = " ";
  uint16 foreground = WHITE;
  
  if(temp_bindClusterDev[temp_s] & CB_TABLE) {
    bind_string = "B";
  }

  if (!(zclApp_Config.HvacUiDisplayMode & DM_ROTATE)){ // portrait
    row = temp_s *120;
    col = 0;  
  } else { // landscape    
    row = 0;
    col = temp_s *106;
  }
  GUI_DrawRectangle(8 + col, 120 + row, 16 + col, 120 + row + 16, zclApp_lcd_background, DRAW_FULL , DOT_PIXEL_DFT );
  GUI_DisString_EN(8 + col, 120 + row, bind_string, &Font16, zclApp_lcd_background, foreground);
}
#endif

#if defined(EPD3IN7)
static void EpdBindStatus(uint8 temp_s){
//  EpdReset(); //disable sleep EPD
//  PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
    //status bind
  uint8 row = temp_s *120;
  char* bind_string = " ";
  if(temp_bindClusterDev[temp_s] & CB_TABLE) {
    bind_string = "B";
  }

  if (zclApp_Config.HvacUiDisplayMode & DM_ROTATE){ // portrait

    PaintSetWidth(16);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
   
    PaintDrawStringAt(0, 2, bind_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 8, 128 + row, PaintGetWidth(), PaintGetHeight());    
  } else { // landscape    
    
  }
//  EpdDisplayFramePartial();
//  EpdSleep();
}
#endif

#if defined(TFT3IN5)
static void TftLqi(uint8 temp_l){
#ifdef LQI_REQ  
  // LQI
  TftWidgetMeasuredValue(temp_l, CB_LQI); //0x00 LQI
#endif  //LQI_REQ
}
#endif  // TFT3IN5

#if defined(EPD3IN7)
static void EpdLqi(uint8 temp_l){
//  EpdReset(); //disable sleep EPD
//  PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
#ifdef LQI_REQ  
  // LQI 
  uint8 row = temp_l *120;
  char lqi_string[] = {' ', ' ', ' ', '\0'};
  if (temp_Sender_shortAddr[temp_l] != 0xFFFE) {
    if (temp_lqi[temp_l] != 255) { 
      lqi_string[0] = temp_lqi[temp_l] / 100 % 10 + '0';
      lqi_string[1] = temp_lqi[temp_l] / 10 % 10 + '0';
      lqi_string[2] = temp_lqi[temp_l] % 10 + '0';
    }
  }
  
  if (zclApp_Config.HvacUiDisplayMode & DM_ROTATE){ // portrait
    if (temp_Sender_shortAddr[temp_l] != 0xFFFE) {
      if (temp_lqi[temp_l] != 255) { 
        PaintSetWidth(24);
        PaintSetHeight(16);
        PaintSetRotate(ROTATE_270);
        PaintSetInvert(!(zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT));
        PaintClear(UNCOLORED);
        if(temp_lqi[temp_l] > 40){
          PaintDrawImage(IMAGE_LQI_100, 0, 0, 16, 24, COLORED);
        } else if (temp_lqi[temp_l] <= 40 && temp_lqi[temp_l] > 30) {
          PaintDrawImage(IMAGE_LQI_80, 0, 0, 16, 24, COLORED);
        } else if (temp_lqi[temp_l] <= 30 && temp_lqi[temp_l] > 20) {
          PaintDrawImage(IMAGE_LQI_60, 0, 0, 16, 24, COLORED);
        } else if (temp_lqi[temp_l] <= 20 && temp_lqi[temp_l] > 10) {
          PaintDrawImage(IMAGE_LQI_40, 0, 0, 16, 24, COLORED);
        } else if (temp_lqi[temp_l] <= 10 && temp_lqi[temp_l] > 0) {
          PaintDrawImage(IMAGE_LQI_20, 0, 0, 16, 24, COLORED);
        } else if (temp_lqi[temp_l] == 0) {
          PaintDrawImage(IMAGE_LQI_0, 0, 0, 16, 24, COLORED);
        }
        EpdSetFrameMemoryXY(PaintGetImage(), 32, 128 + row, PaintGetWidth(), PaintGetHeight());
        PaintSetInvert(zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      }
    } else {
      PaintSetWidth(36);
      PaintSetHeight(16);
      PaintSetRotate(ROTATE_0);
      PaintClear(UNCOLORED);
      PaintDrawStringAt(0, 2, " ", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 32, 128 + row, PaintGetWidth(), PaintGetHeight());
      
    }
    PaintSetWidth(36);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 2, lqi_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 64, 128 + row, PaintGetWidth(), PaintGetHeight());
  } else { // lanscape
    if (temp_lqi[0] != 255) {
      PaintSetWidth(16);
      PaintSetHeight(36);
      PaintSetRotate(ROTATE_90);
      PaintClear(UNCOLORED);
      PaintDrawStringAt(0, 0, lqi_string, &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 256, 64, PaintGetWidth(), PaintGetHeight());
      if(temp_lqi[0] > 40){
        EpdSetFrameMemoryImageXY(IMAGE_LQI_100, 256, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      } else if (temp_lqi[0] <= 40 && temp_lqi[0] > 30) {
        EpdSetFrameMemoryImageXY(IMAGE_LQI_80, 256, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      } else if (temp_lqi[0] <= 30 && temp_lqi[0] > 20) {
        EpdSetFrameMemoryImageXY(IMAGE_LQI_60, 256, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      } else if (temp_lqi[0] <= 20 && temp_lqi[0] > 10) {
        EpdSetFrameMemoryImageXY(IMAGE_LQI_40, 256, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      } else if (temp_lqi[0] <= 10 && temp_lqi[0] > 0) {
        EpdSetFrameMemoryImageXY(IMAGE_LQI_20, 256, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      } else if (temp_lqi[0] == 0) {
        EpdSetFrameMemoryImageXY(IMAGE_LQI_0, 256, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      }
    }
  }
#endif  //LQI_REQ
//  EpdDisplayFramePartial();
//  EpdSleep();
}
#endif  // EPD3IN7

#if defined(TFT3IN5)
static void TftNwk(uint8 temp_n){  
  // nwkDevAddress
    uint8 row = 0;
    uint8 col = 0;
    uint16 foreground = WHITE;
    char nwk_string[] = {' ', ' ', ' ', ' ', ' ', ' ', '\0'};
    if (temp_Sender_shortAddr[temp_n] != 0xFFFE) {
      nwk_string[0] = '0';
      nwk_string[1] = 'x';
      nwk_string[2] = temp_Sender_shortAddr[temp_n] / 4096 %16 + '0';
      if ((temp_Sender_shortAddr[temp_n]/4096 %16) > 9){
        nwk_string[2] = nwk_string[2]+7;
      }  
      nwk_string[3] = temp_Sender_shortAddr[temp_n] / 256 %16 + '0';
      if ((temp_Sender_shortAddr[temp_n]/256 %16) > 9){
        nwk_string[3] = nwk_string[3]+7;
      }
      nwk_string[4] = temp_Sender_shortAddr[temp_n] / 16 %16 + '0';
      if ((temp_Sender_shortAddr[temp_n]/16 %16) > 9){
        nwk_string[4] = nwk_string[4]+7;
      }
      nwk_string[5] = temp_Sender_shortAddr[temp_n] %16 + '0';
      if ((temp_Sender_shortAddr[temp_n] %16) > 9){
        nwk_string[5] = nwk_string[5]+7;
      }
    }

  if (!(zclApp_Config.HvacUiDisplayMode & DM_ROTATE)){ // portrait
    row = temp_n *120;
    col = 0;
  } else { // landscape
    row = 0;
    col = temp_n *106;
  }
  GUI_DrawRectangle(32 + col, 120 + row, 32 + col + 72, 120 + row + 16, zclApp_lcd_background, DRAW_FULL , DOT_PIXEL_DFT );
  GUI_DisString_EN(32 + col, 120 + row, nwk_string, &Font16, zclApp_lcd_background, foreground); 
}
#endif

#if defined(EPD3IN7)
static void EpdNwk(uint8 temp_n){
//  EpdReset(); //disable sleep EPD
//  PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);  
  // nwkDevAddress, nwkPanId, nwkLogicalChannel
    uint8 row = temp_n *120;
    char nwk_string[] = {' ', ' ', ' ', ' ', ' ', ' ', '\0'};
    if (temp_Sender_shortAddr[temp_n] != 0xFFFE) {
      nwk_string[0] = '0';
      nwk_string[1] = 'x';
      nwk_string[2] = temp_Sender_shortAddr[temp_n] / 4096 %16 + '0';
      if ((temp_Sender_shortAddr[temp_n]/4096 %16) > 9){
        nwk_string[2] = nwk_string[2]+7;
      }  
      nwk_string[3] = temp_Sender_shortAddr[temp_n] / 256 %16 + '0';
      if ((temp_Sender_shortAddr[temp_n]/256 %16) > 9){
        nwk_string[3] = nwk_string[3]+7;
      }
      nwk_string[4] = temp_Sender_shortAddr[temp_n] / 16 %16 + '0';
      if ((temp_Sender_shortAddr[temp_n]/16 %16) > 9){
        nwk_string[4] = nwk_string[4]+7;
      }
      nwk_string[5] = temp_Sender_shortAddr[temp_n] %16 + '0';
      if ((temp_Sender_shortAddr[temp_n] %16) > 9){
        nwk_string[5] = nwk_string[5]+7;
      }
    }

  if (zclApp_Config.HvacUiDisplayMode & DM_ROTATE){ // portrait
    PaintSetWidth(264);
    PaintSetHeight(8);
    PaintSetRotate(ROTATE_90); 
    PaintClear(UNCOLORED);  
    PaintDrawVerticalLine(0, 0, 264, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 8, 120 + row, PaintGetWidth(), PaintGetHeight());
           
    PaintSetWidth(72);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 2, nwk_string, &Font16, COLORED); 
    EpdSetFrameMemoryXY(PaintGetImage(), 208, 128 + row, PaintGetWidth(), PaintGetHeight());   
  } else { // landscape  
    PaintSetWidth(16);
    PaintSetHeight(192);
    PaintSetRotate(ROTATE_90);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, nwk_string, &Font16, COLORED); 
    EpdSetFrameMemoryXY(PaintGetImage(), 256, 216, PaintGetWidth(), PaintGetHeight()); 
  }
//  EpdDisplayFramePartial();
//  EpdSleep();
}
#endif

#if defined(TFT3IN5)
static void TftBattery(uint8 temp_b){
  //percentage
  TftWidgetMeasuredValue(temp_b, CB_POWER_CFG); //0x01 battery 
}
#endif

#if defined(EPD3IN7)
static void EpdBattery(uint8 temp_b){
//  EpdReset(); //disable sleep EPD
//  PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
  //percentage
  uint8 row = temp_b *120;
  char perc_string[] = {' ', ' ', ' ', ' ', '\0'};
  if (old_bindClusterDev[temp_b] & CB_POWER_CFG){
    if (temp_Battery_PercentageRemainig[temp_b] != 0xFF) {
      perc_string[0] = temp_Battery_PercentageRemainig[temp_b]/2 / 100 % 10 + '0';
      perc_string[1] = temp_Battery_PercentageRemainig[temp_b]/2 / 10 % 10 + '0';
      perc_string[2] = temp_Battery_PercentageRemainig[temp_b]/2 % 10 + '0';
      perc_string[3] = '%';
    }
  }

  if (zclApp_Config.HvacUiDisplayMode & DM_ROTATE){ // portrait
    PaintSetWidth(48);
    PaintSetHeight(18);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 2, perc_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 144, 128 + row, PaintGetWidth(), PaintGetHeight());
    if (old_bindClusterDev[temp_b] & CB_POWER_CFG){
      if (temp_Battery_PercentageRemainig[temp_b] != 0xFF) {
        PaintSetWidth(24);
        PaintSetHeight(16);
        PaintSetRotate(ROTATE_270);
        PaintSetInvert(!(zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT));
        PaintClear(UNCOLORED);
        if(temp_Battery_PercentageRemainig[temp_b]/2 > 75){
          PaintDrawImage(IMAGE_BATTERY_100, 0, 0, 16, 24, COLORED);
        } else if (temp_Battery_PercentageRemainig[temp_b]/2 <= 75 && temp_Battery_PercentageRemainig[temp_b]/2 > 50) {
          PaintDrawImage(IMAGE_BATTERY_75, 0, 0, 16, 24, COLORED);
        } else if (temp_Battery_PercentageRemainig[temp_b]/2 <= 50 && temp_Battery_PercentageRemainig[temp_b]/2 > 25) {
          PaintDrawImage(IMAGE_BATTERY_50, 0, 0, 16, 24, COLORED);
        } else if (temp_Battery_PercentageRemainig[temp_b]/2 <= 25 && temp_Battery_PercentageRemainig[temp_b]/2 > 6) {
          PaintDrawImage(IMAGE_BATTERY_25, 0, 0, 16, 24, COLORED);
        } else if (temp_Battery_PercentageRemainig[temp_b]/2 <= 6 && temp_Battery_PercentageRemainig[temp_b]/2 > 0) {
          PaintDrawImage(IMAGE_BATTERY_0, 0, 0, 16, 24, COLORED);
        }
        EpdSetFrameMemoryXY(PaintGetImage(), 112, 128 + row, PaintGetWidth(), PaintGetHeight());
        PaintSetInvert(zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT); 
      }
    } else {
      PaintDrawStringAt(0, 2, " ", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 112, 128 + row, PaintGetWidth(), PaintGetHeight());
    }      
  } else { // landscape
    PaintSetWidth(16);
    PaintSetHeight(48);
    PaintSetRotate(ROTATE_90);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, perc_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 256, 144, PaintGetWidth(), PaintGetHeight());
    if (zclBattery_PercentageRemainig != 0xFF) {   
      if(zclBattery_PercentageRemainig/2 > 75){
        EpdSetFrameMemoryImageXY(IMAGE_BATTERY_100, 256, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      } else if (zclBattery_PercentageRemainig/2 <= 75 && zclBattery_PercentageRemainig/2 > 50) {
        EpdSetFrameMemoryImageXY(IMAGE_BATTERY_75, 256, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      } else if (zclBattery_PercentageRemainig/2 <= 50 && zclBattery_PercentageRemainig/2 > 25) {
        EpdSetFrameMemoryImageXY(IMAGE_BATTERY_50, 256, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      } else if (zclBattery_PercentageRemainig/2 <= 25 && zclBattery_PercentageRemainig/2 > 6) {
        EpdSetFrameMemoryImageXY(IMAGE_BATTERY_25, 256, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      } else if (zclBattery_PercentageRemainig/2 <= 6 && zclBattery_PercentageRemainig/2 > 0) {
        EpdSetFrameMemoryImageXY(IMAGE_BATTERY_0, 256, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      }
    }
  }
//  EpdDisplayFramePartial();
//  EpdSleep();
}
#endif

#if defined(TFT3IN5)
static void TftOccupancy(uint8 temp_oc){
  // Occupancy
  TftWidgetMeasuredValue(temp_oc, CB_OCCUPANCY); //0x20 occupancy 
}
#endif

#if defined(EPD3IN7)
static void EpdOccupancy(uint8 temp_oc){
//  EpdReset(); //disable sleep EPD
//  PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
  // Occupancy
  char* occup_string = "";

  if (zclApp_Occupied == 0) {
//  if (zclApp_Occupied_OnOff == 0) {
    occup_string = "UnOccupied";
  } else {
    occup_string = " Occupied ";
  }
  uint8 row = temp_oc *120;
  
  if (zclApp_Config.HvacUiDisplayMode & DM_ROTATE){ // portrait 
    if (old_bindClusterDev[temp_oc] & CB_OCCUPANCY){
      PaintSetWidth(64);
      PaintSetHeight(64);
      PaintSetRotate(ROTATE_270);
      PaintSetInvert(!(zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT));
      PaintClear(UNCOLORED);    
      if (temp_Occupied[temp_oc] == 0) {
        PaintDrawImage(IMAGE_MOTION_NOT, 0, 0, 64, 64, COLORED);
        EpdSetFrameMemoryXY(PaintGetImage(), 0, 160 + row, PaintGetWidth(), PaintGetHeight());     
      } else {
        PaintDrawImage(IMAGE_MOTION, 0, 0, 64, 64, COLORED);
        EpdSetFrameMemoryXY(PaintGetImage(), 0, 160 + row, PaintGetWidth(), PaintGetHeight()); 
      }
      PaintSetInvert(zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    } else {
      PaintSetWidth(64);
      PaintSetHeight(64);
      PaintSetRotate(ROTATE_0);    
      PaintClear(UNCOLORED);
      PaintDrawStringAt(0, 0, " ", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 0, 160 + row, PaintGetWidth(), PaintGetHeight());       
    }
  } else { // landscape
    PaintSetWidth(16);
    PaintSetHeight(110);
    PaintSetRotate(ROTATE_90);    
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, occup_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 128, 46, PaintGetWidth(), PaintGetHeight()); 
    if (zclApp_Occupied == 0) {
      EpdSetFrameMemoryImageXY(IMAGE_MOTION_NOT, 144, 74, 64, 64, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_MOTION,     144, 74, 64, 64, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    }
  }
//  EpdDisplayFramePartial();
//  EpdSleep();
}
#endif

#if defined(TFT3IN5)
static void TftCO2(uint8 temp_co2){
  //CO2
  TftWidgetMeasuredValue(temp_co2, CB_CO2); //0x80 CO2
}
#endif

#if defined(TFT3IN5)
static void TftIlluminance(uint8 temp_i){
  //Illuminance
  TftWidgetMeasuredValue(temp_i, CB_ILLUMINANCE); //0x02 illuminance
}
#endif

#if defined(EPD3IN7)
static void EpdIlluminance(uint8 temp_i){
//  EpdReset(); //disable sleep EPD
//  PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
  //Illuminance
  uint8 row = temp_i *120;
  char illum_string[] = {' ',' ', ' ', ' ', ' ', '\0'};
  if (old_bindClusterDev[temp_i] & CB_ILLUMINANCE){
    illum_string[0] = temp_bh1750IlluminanceSensor_MeasuredValue[temp_i] / 10000 % 10 + '0';
    illum_string[1] = temp_bh1750IlluminanceSensor_MeasuredValue[temp_i] / 1000 % 10 + '0';
    illum_string[2] = temp_bh1750IlluminanceSensor_MeasuredValue[temp_i] / 100 % 10 + '0';
    illum_string[3] = temp_bh1750IlluminanceSensor_MeasuredValue[temp_i] / 10 % 10 + '0';
    illum_string[4] = temp_bh1750IlluminanceSensor_MeasuredValue[temp_i] % 10 + '0';
  }

  if (zclApp_Config.HvacUiDisplayMode & DM_ROTATE){ // portrait
    PaintSetWidth(80);
    PaintSetHeight(32);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, illum_string, &Font32, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 184, 144 + row, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(33);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    if (old_bindClusterDev[temp_i] & CB_ILLUMINANCE){
      PaintDrawStringAt(0, 0, "Lux", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 184, 176 + row, PaintGetWidth(), PaintGetHeight());
      if (zclApp_UpDown[temp_i] & CB_ILLUMINANCE){
        EpdSetFrameMemoryImageXY(IMAGE_LEFT, 168, 144 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
        EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 168, 160 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      } else {
        EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 168, 160 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
        EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 168, 144 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      }
    } else {
      PaintDrawStringAt(0, 0, "   ", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 184, 176 + row, PaintGetWidth(), PaintGetHeight());
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 168, 160 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 168, 144 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    }   
  } else { //landscape
    PaintSetWidth(32);
    PaintSetHeight(80);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, illum_string, &Font32, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 200, 300, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(16);
    PaintSetHeight(33);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, "Lux", &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 200, 388, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(16);
    PaintSetHeight(121);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, "Illuminance", &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 184, 300, PaintGetWidth(), PaintGetHeight());
    EpdSetFrameMemoryImageXY(IMAGE_ILLUMINANCE, 184, 240, 48, 48, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    if (zclApp_UpDown[temp_i] & CB_ILLUMINANCE){
      EpdSetFrameMemoryImageXY(IMAGE_UP, 216, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 184, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_DOWN, 184, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 216, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    }
  }
//  EpdDisplayFramePartial();
//  EpdSleep();

}
#endif

#if defined(TFT3IN5)
// enable/disable display of values
// bit 0 - 0x0001 POWER_CFG, 1 - 0x0400 ILLUMINANCE, 2 - 0x0402 TEMP,         3 - 0x0403 PRESSURE, 
//     4 - 0x0405 HUMIDITY,  5 - 0x0406 OCCUPANCY,   6 - 0x000F BINARY_INPUT, 7 - table received
static void TftWidgetMeasuredValue(uint8 dev_num, uint16 cluster_bit){
  uint16 row = 0;
  uint16 col = 0;
  if (!(zclApp_Config.HvacUiDisplayMode & DM_ROTATE)){ // landscape
    row = dev_num *120;
    col = 0;
  } else { // portrait
    row = 0;
    col = dev_num *106;
  }
  uint16 x;
  uint16 y;
  uint16 w = 100; // widget width
  uint16 h = 48; // widget height
  uint8 wb = 4; // width between widgets
  uint8 hb = 4; // height between widgets
  uint8 wi = 48; // image width
  uint8 hi = 48; // image height
  char* string_uint = "";
  const unsigned char* image_uint = "";
  uint16 value;
  uint8 scale;
  uint16 background = zclApp_lcd_background;
  uint16 foreground = BLUE;
  uint16 textcolor = zclApp_lcd_background;
  if (cluster_bit == CB_LQI){ // 0x00 LQI
    if (!(zclApp_Config.HvacUiDisplayMode & DM_ROTATE)){ 
      x = 160 - (w/2) - wb - w;
      y = 136 + row;
    } else {// portrait
      x = 4 + col;
      y = 136 + ((48 + hb) * 0);
    }
    w = 48;
    h = 48;
    wi = 25;
    hi = 16;
    string_uint = "";
    value = temp_lqi[dev_num];
    scale = 0;
    foreground = color_scheme[scheme][0];
    textcolor = color_scheme[scheme][4];
//    textcolor = color_scheme[scheme][2];
        if(value > 40){
          image_uint = IMAGE_LQI_100;          
        } else if (value <= 40 && value > 30) {
          image_uint = IMAGE_LQI_80;
        } else if (value <= 30 && value > 20) {
          image_uint = IMAGE_LQI_60;
        } else if (value <= 20 && value > 10) {
          image_uint = IMAGE_LQI_40;
        } else if (value <= 10 && value > 0) {
          image_uint = IMAGE_LQI_20;
        } else if (value == 0) {
          image_uint = IMAGE_LQI_0;
        } 
  }
  if (cluster_bit == CB_POWER_CFG){ // 0x01 battery
    if (!(zclApp_Config.HvacUiDisplayMode & DM_ROTATE)){ 
      x = 160 - (w/2) - wb - w;
      y = 188 + row;
    } else {// portrait
      x = 4 + col + 48 + wb;
      y = 136 + ((48 + hb) * 0);      
    }
    w = 48;
    h = 48;
    wi = 25;
    hi = 16;
    value = temp_Battery_PercentageRemainig[dev_num]/2;
    string_uint = "";    
    scale = 0;
    foreground = color_scheme[scheme][0];
    textcolor = color_scheme[scheme][4];
//    textcolor = color_scheme[scheme][2];    
    if(value > 75){
      image_uint = IMAGE_BATTERY_100;
    } else if (value <= 75 && value > 50) {
      image_uint = IMAGE_BATTERY_75;
    } else if (value <= 50 && value > 25) {
      image_uint = IMAGE_BATTERY_50;
    } else if (value <= 25 && value > 6) {
      image_uint = IMAGE_BATTERY_25;
    } else if (value <= 6 && value > 0) {
      image_uint = IMAGE_BATTERY_0;
    }
  }
  if (cluster_bit == CB_OCCUPANCY){ // 0x20 occupancy
    if (!(zclApp_Config.HvacUiDisplayMode & DM_ROTATE)){ 
      x = 160 - (w/2) - wb - (w/2) + (wb/2);
      y = 136 + row;
    } else {// portrait
      x = 4 + col;
      y = 136 + ((48 + hb) * 1);
    }
    w = 48;
    h = 48;
    string_uint = "Motion";
    value = temp_Occupied[dev_num];
    scale = 0;
    foreground = color_scheme[scheme][1];
    textcolor = color_scheme[scheme][4];
    if (value == 0) {
      image_uint = IMAGE_MOTION_NOT;     
    } else {
      image_uint = IMAGE_MOTION;  
    }
  }
  if (cluster_bit == CB_BINARY_INPUT){ // 0x40 binary
    if (!(zclApp_Config.HvacUiDisplayMode & DM_ROTATE)){ 
      x = 160 - (w/2) - wb - (w/2) + (wb/2);
      y = 136 + row + hb + h;
    } else {                    // portrait
      x = 4 + col + 48 + wb;
      y = 136 + ((48 + hb) * 1);
    }
    w = 48;
    h = 48;
    string_uint = "Magnet";
    value = temp_Binary[dev_num];
    scale = 0;
    foreground = color_scheme[scheme][1];
    textcolor = color_scheme[scheme][4];
    if (value == 1) {
      image_uint = IMAGE_DOOR_CLOSE;     
    } else {
      image_uint = IMAGE_DOOR_OPEN;  
    }
  }
  if (cluster_bit == CB_ILLUMINANCE){ // 0x02 illuminance
    if (!(zclApp_Config.HvacUiDisplayMode & DM_ROTATE)){ 
      x = 160 + (w/2) + wb;
      y = 136 + row;
    } else {// portrait
      x = 4 + col;
      y = 136 + ((48 + hb) * 5);
    }
    string_uint = "Lux";
    value = temp_bh1750IlluminanceSensor_MeasuredValue[dev_num];
    scale = 0;
//    foreground = MAGENTA;
    foreground = color_scheme[scheme][6];
    textcolor = color_scheme[scheme][7];
  }
  if (cluster_bit == CB_TEMP) { // 0x04 temperature
    if (!(zclApp_Config.HvacUiDisplayMode & DM_ROTATE)){ 
      x = 160 - (w/2);
      y = 136 + row;
    } else {// portrait
      x = 4 + col;
      y = 136 + ((48 + hb) * 2);
    }
    string_uint = "^C";
    value = temp_Temperature_Sensor_MeasuredValue[dev_num];
    scale = 2;
    foreground = color_scheme[scheme][3];
    textcolor = color_scheme[scheme][2];
  }  
  if (cluster_bit == CB_PRESSURE){ // 0x08 pressure
    if (!(zclApp_Config.HvacUiDisplayMode & DM_ROTATE)){ 
      x = 160 + (w/2) + wb;
      y = 188 + row;
    } else {// portrait
      x = 4 + col;
      y = 136 + ((48 + hb) * 4);
    }
    string_uint = "hPa";
    value = temp_PressureSensor_MeasuredValue[dev_num];
    scale = 0;
    foreground = color_scheme[scheme][5];
    textcolor = color_scheme[scheme][4];
  }
  if (cluster_bit == CB_CO2){ // 0x80 CO2
    if (!(zclApp_Config.HvacUiDisplayMode & DM_ROTATE)){ 
      x = 160 + (w/2) + wb;
      y = 188 + row;
    } else {// portrait
      x = 4 + col;
      y = 136 + ((48 + hb) * 4);
    }
    string_uint = "ppm";
    value = temp_scd4xCO2_Sensor_MeasuredValue[dev_num];
    scale = 0;
    foreground = color_scheme[scheme][5];
    textcolor = color_scheme[scheme][4];
  }
  if (cluster_bit == CB_HUMIDITY){ // 0x10 humidity
    if (!(zclApp_Config.HvacUiDisplayMode & DM_ROTATE)){ 
      x = 160 - (w/2);
      y = 188 + row;
    } else {// portrait
      x = 4 + col;
      y = 136 + ((48 + hb) * 3);
    }
    string_uint = "%Ha";
    value = temp_HumiditySensor_MeasuredValue[dev_num];
    scale = 2; 
    foreground = color_scheme[scheme][4];
    textcolor = color_scheme[scheme][2];
  }
  
  LCD_SetArealColorWH(x-1, y-1, w+2, h+2, background);
  
      if (zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT) {        
        uint16 tfore = foreground;
        foreground = textcolor;
        textcolor = tfore;
      } else {  
        
      }
  
  if ((old_bindClusterDev[dev_num] & cluster_bit) || (cluster_bit == CB_LQI && temp_lqi[dev_num] != 255 && temp_Sender_shortAddr[dev_num] != 0xFFFE) ){    
      GUI_DrawRoundRectangle(8, x, y, x+w, y+h, foreground, DRAW_FULL , DOT_PIXEL_DFT );      
      if (image_uint != "") {
        GUI_Disbitmap((x+w/2) - (wi/2), (y+h/2) - (hi/2), image_uint, wi, hi, textcolor, 0);
        if (string_uint == "") {
          GUI_DisNumDP(x+8, y+32, value, scale, &Font16, foreground, textcolor );
        }
      } else {
        GUI_DisNumDP(x+16, y+2, value, scale, &Font32, foreground, textcolor );
        GUI_DisString_EN(x+16, y+32, string_uint, &Font16, foreground, textcolor);
        if (zclApp_UpDown[dev_num] & cluster_bit){
          GUI_Disbitmap(x, y+6, IMAGE_LEFT, 16, 16, textcolor, 1);
        } else {
          GUI_Disbitmap(x, y+16+6 , IMAGE_RIGHT, 16, 16, textcolor, 1);
        }         
      }
  }
  
}

static void TftTemperature(uint8 temp_t){
  TftWidgetMeasuredValue(temp_t, CB_TEMP); //0x04 temperature
}
#endif

#if defined(EPD3IN7)
static void EpdTemperature(uint8 temp_t){
//  EpdReset(); //disable sleep EPD
//  PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
  //temperature
  uint8 row = temp_t *120;
  char temp_string[] = {' ', ' ', ' ', ' ', ' ', '\0'};
  if (old_bindClusterDev[temp_t] & CB_TEMP){
    temp_string[0] = temp_Temperature_Sensor_MeasuredValue[temp_t] / 1000 % 10 + '0';
    temp_string[1] = temp_Temperature_Sensor_MeasuredValue[temp_t] / 100 % 10 + '0';
    temp_string[2] = '.';
    temp_string[3] = temp_Temperature_Sensor_MeasuredValue[temp_t] / 10 % 10 + '0';
    temp_string[4] = temp_Temperature_Sensor_MeasuredValue[temp_t] % 10 + '0';
  }

  if (zclApp_Config.HvacUiDisplayMode & DM_ROTATE){ // portrait
    PaintSetWidth(80);
    PaintSetHeight(32);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, temp_string, &Font32, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 80, 144 + row, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(22);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    if (old_bindClusterDev[temp_t] & CB_TEMP){
      PaintDrawStringAt(0, 0, "^C", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 80, 176 + row, PaintGetWidth(), PaintGetHeight());
      if (zclApp_UpDown[temp_t] & CB_TEMP){
        EpdSetFrameMemoryImageXY(IMAGE_LEFT, 64, 144 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
        EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 64, 160 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      } else {
        EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 64, 160 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
        EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 64, 144 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      }
    } else {
      PaintDrawStringAt(0, 0, "  ", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 80, 176 + row, PaintGetWidth(), PaintGetHeight());
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 64, 160 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 64, 144 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    }
  } else { //landscape
    PaintSetWidth(32);
    PaintSetHeight(80);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, temp_string, &Font32, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 144, 300, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(16);
    PaintSetHeight(33);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, "^C", &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 144, 388, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(16);
    PaintSetHeight(121);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, "Temperature", &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 128, 300, PaintGetWidth(), PaintGetHeight());
    EpdSetFrameMemoryImageXY(IMAGE_TEMPERATURE, 128, 240, 48, 48, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    if (zclApp_UpDown[temp_t] & CB_TEMP){
      EpdSetFrameMemoryImageXY(IMAGE_UP, 160, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 128, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_DOWN, 128, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 160, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    }
  }
//  EpdDisplayFramePartial();
//  EpdSleep();
}
#endif

#if defined(TFT3IN5)
static void TftHumidity(uint8 temp_h){
  //humidity
  TftWidgetMeasuredValue(temp_h, CB_HUMIDITY); //0x10 humidity
}
#endif

#if defined(EPD3IN7)
static void EpdHumidity(uint8 temp_h){
//  EpdReset(); //disable sleep EPD
//  PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
  //humidity
  uint8 row = temp_h * 120;
  char hum_string[] = {' ', ' ', ' ', ' ', ' ', '\0'};
  if (old_bindClusterDev[temp_h] & CB_HUMIDITY){
    hum_string[0] = temp_HumiditySensor_MeasuredValue[temp_h] / 1000 % 10 + '0';
    hum_string[1] = temp_HumiditySensor_MeasuredValue[temp_h] / 100 % 10 + '0';
    hum_string[2] = '.';
    hum_string[3] = temp_HumiditySensor_MeasuredValue[temp_h] / 10 % 10 + '0';
    hum_string[4] = temp_HumiditySensor_MeasuredValue[temp_h] % 10 + '0';
  }

  if (zclApp_Config.HvacUiDisplayMode & DM_ROTATE){ // portrait
    PaintSetWidth(80);
    PaintSetHeight(32);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, hum_string, &Font32, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 80, 192 + row, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(33);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    if (old_bindClusterDev[temp_h] & CB_HUMIDITY){
      PaintDrawStringAt(0, 0, "%Ha", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 80, 224 + row, PaintGetWidth(), PaintGetHeight());
      if (zclApp_UpDown[temp_h] & CB_HUMIDITY){
        EpdSetFrameMemoryImageXY(IMAGE_LEFT, 64, 192 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
        EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 64, 208 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      } else {
        EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 64, 208 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
        EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 64, 192 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      } 
    } else {
      PaintDrawStringAt(0, 0, "   ", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 80, 224 + row, PaintGetWidth(), PaintGetHeight());
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 64, 208 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 64, 192 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    }
  } else { // landscape
    PaintSetWidth(32);
    PaintSetHeight(80);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, hum_string, &Font32, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 88, 300, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(16);
    PaintSetHeight(33);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, "%Ha", &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 88, 388, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(16);
    PaintSetHeight(121);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, "Humidity", &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 72, 300, PaintGetWidth(), PaintGetHeight());
    EpdSetFrameMemoryImageXY(IMAGE_HUMIDITY, 72, 240, 48, 48, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    if (zclApp_UpDown[temp_h] & CB_HUMIDITY){
      EpdSetFrameMemoryImageXY(IMAGE_UP, 104, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 72, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_DOWN, 72, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 104, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    }
  }
//  EpdDisplayFramePartial();
//  EpdSleep();
}
#endif

#if defined(TFT3IN5)
static void TftPressure(uint8 temp_p){
  //pressure
  TftWidgetMeasuredValue(temp_p, CB_PRESSURE); //0x08 pressure 
}
#endif

#if defined(TFT3IN5)
static void TftBinary(uint8 temp_b){
  //binary
  TftWidgetMeasuredValue(temp_b, CB_BINARY_INPUT); //0x40 binary 
}
#endif

#if defined(EPD3IN7)
static void EpdPressure(uint8 temp_p){
//  EpdReset(); //disable sleep EPD
//  PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
  //pressure
  uint8 row = temp_p*120;
  char pres_string[] = {' ', ' ', ' ', ' ', ' ', ' ', '\0'};
  if (old_bindClusterDev[temp_p] & CB_PRESSURE){
    pres_string[0] = temp_PressureSensor_MeasuredValue[temp_p] / 1000 % 10 + '0';
    pres_string[1] = temp_PressureSensor_MeasuredValue[temp_p] / 100 % 10 + '0';
    pres_string[2] = temp_PressureSensor_MeasuredValue[temp_p] / 10 % 10 + '0';
    pres_string[3] = temp_PressureSensor_MeasuredValue[temp_p] % 10 + '0';
    pres_string[4] = '.';
    pres_string[5] = temp_PressureSensor_ScaledValue[temp_p] % 10 + '0';
  }

  if (zclApp_Config.HvacUiDisplayMode & DM_ROTATE){ // portrait
    PaintSetWidth(96);
    PaintSetHeight(32);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, pres_string, &Font32, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 184, 192 + row, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(33);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    if (old_bindClusterDev[temp_p] & CB_PRESSURE){
      PaintDrawStringAt(0, 0, "hPa", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 184, 224 + row, PaintGetWidth(), PaintGetHeight());
      if (zclApp_UpDown[temp_p] & CB_PRESSURE){
        EpdSetFrameMemoryImageXY(IMAGE_LEFT, 168, 192 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
        EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 168, 208 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      } else {
        EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 168, 208 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
        EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 168, 192 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      } 
    } else {
      PaintDrawStringAt(0, 0, "   ", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 184, 224 + row, PaintGetWidth(), PaintGetHeight());
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 168, 208 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 168, 192 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    }
  } else { //landscape
    PaintSetWidth(32);
    PaintSetHeight(96);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, pres_string, &Font32, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 32, 300, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(16);
    PaintSetHeight(33);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, "hPa", &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 32, 404, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(16);
    PaintSetHeight(121);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, "Pressure", &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 16, 300, PaintGetWidth(), PaintGetHeight());
    EpdSetFrameMemoryImageXY(IMAGE_PRESSURE, 16, 240, 48, 48, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    if (zclApp_UpDown[temp_p] & CB_PRESSURE){
      EpdSetFrameMemoryImageXY(IMAGE_UP, 48, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 16, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_DOWN, 16, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 48, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
    }
  }
//  EpdDisplayFramePartial();
//  EpdSleep();
}
#endif

#if defined(TFT3IN5)
static void TfttestRefresh(uint8 i)
{
      LREP("bindClusterDev=0x%X 0x%X\r\n", temp_bindClusterDev[i], old_bindClusterDev[i]);     
      TftBindStatus(i);  // status bind
      TftLqi(i);         // lqi
      TftBattery(i);     // percentage battery
      TftNwk(i);         // nwk
      TftOccupancy(i);   // occupancy
      TftBinary(i);      // binary
      TftIlluminance(i); // illuminance
      TftTemperature(i); // temperature
      TftHumidity(i);    // humidity
      TftPressure(i);    // pressure
      if (!(temp_bindClusterDev[i] & CB_PRESSURE)) {
        TftCO2(i);       // co2
      }
}

void TftUpdateRefresh(void) { 
            if (!zcl_game) {
              if (zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT){
                zclApp_lcd_background = color_scheme[scheme][3];
              } else {
                zclApp_lcd_background = color_scheme[scheme][2];
              }
              GUI_Clear(zclApp_lcd_background);
              zclApp_menu = 0;
              zclApp_create_butt_main(zclApp_menu);
              for (uint8 i = 0; i <= 2; i++) {               
                if (temp_Sender_shortAddr[i] != 0xFFFE){
                  TfttestRefresh(i);
                }
              }
            }
}
#endif

#if defined(EPD3IN7)
static void EpdtestRefresh(void)
{   
  EpdReset(); //disable sleep EPD
  PaintSetInvert(zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT);
  
  EpdTimeDateWeek(); // time, date, weekday
  EpdStatus(0); // status network device

  for(uint8 i = 0; i <= 2; i++ ){ 
    LREP("bindClusterDev=0x%X 0x%X\r\n", temp_bindClusterDev[i], old_bindClusterDev[i]);
    zclApp_EpdSensors(i);
  }
  EpdDisplayFramePartial();
  EpdSleep();
}

static void zclApp_EpdSensors(uint8 i) { 
      EpdBindStatus(i);  // status bind   
      EpdLqi(i);         // lqi
      EpdBattery(i);     // percentage battery
      EpdNwk(i);         // nwk
      EpdOccupancy(i);   // occupancy
      EpdIlluminance(i); // illuminance
      EpdTemperature(i); // temperature
      EpdHumidity(i);    // humidity
      EpdPressure(i);    // pressure
}
#endif

void zclApp_SetTimeDate(void){
  // Set Time and Date
  UTCTimeStruct time;
   time.seconds = 00;   
   time.minutes = (zclApp_DateCode[15]-48)*10 + (zclApp_DateCode[16]-48);
   time.hour = (zclApp_DateCode[12]-48)*10 + (zclApp_DateCode[13]-48);
   time.day = (zclApp_DateCode[1]-48)*10 + (zclApp_DateCode[2]-48) - 1;
   time.month = (zclApp_DateCode[4]-48)*10 + (zclApp_DateCode[5]-48) - 1;
   time.year = 2000+(zclApp_DateCode[9]-48)*10 + (zclApp_DateCode[10]-48);
   
  // Update OSAL time
  osal_setClock( osal_ConvertUTCSecs( &time ) );
  // Get time structure from OSAL
  osal_ConvertUTCTime( &time, osal_getClock() );
  osalTimeUpdate();
  zclApp_GenTime_TimeUTC = osal_getClock();
  
  zclApp_GenTime_old = zclApp_GenTime_TimeUTC;
}

static void zclApp_OnOffCB(uint8 cmd)
{
  afIncomingMSGPacket_t *pPtr = zcl_getRawAFMsg();
//  zclDIYRuZRT_DstAddr.addr.shortAddr = pPtr->srcAddr.addr.shortAddr;
//  inderect_DstAddr.addr.shortAddr = pPtr->srcAddr.addr.shortAddr;
//  inderect_DstAddr.endPoint = pPtr->srcAddr.endPoint;
  
  if (cmd == COMMAND_ON) {
    zclApp_Occupied_OnOff = 1;
//    zclGeneral_SendOnOff_CmdOn(zclApp_ThirdEP.EndPoint, &inderect_DstAddr, TRUE, bdb_getZCLFrameCounter());
  }

  else if (cmd == COMMAND_OFF) {
    zclApp_Occupied_OnOff = 0;
//    zclGeneral_SendOnOff_CmdOff(zclApp_ThirdEP.EndPoint, &inderect_DstAddr, TRUE, bdb_getZCLFrameCounter());
  }

  else if (cmd == COMMAND_TOGGLE) {

  }
#if defined(EPD3IN7)  
  EpdRefresh();
#endif //  EPD3IN7
#if defined(TFT3IN5)  
//  TftRefresh();
#endif //  TFT3IN5
}

#ifdef ZCL_REPORT_DESTINATION_DEVICE
static void zclApp_AttrIncomingReport( zclIncomingMsg_t *pInMsg )
{
//  HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK);
 
  uint8 i = 0;
  uint8 save = 0;
  for (uint8 x = 0; x < 3 ; x++) {
    if (temp_Sender_shortAddr[x] == pInMsg->srcAddr.addr.shortAddr) {
      i = x;
      save = 1;
    }
  }
  for (uint8 y = 0; y < 3 && save == 0; y++) {
      if (temp_Sender_shortAddr[y] == 0xFFFE) {
        temp_Sender_shortAddr[y] = pInMsg->srcAddr.addr.shortAddr;
        i = y;
        save = 1;
        
        zclApp_RequestAddr(temp_Sender_shortAddr[y]);
#if defined(TFT3IN5) 
        if (!zcl_game){
          TftNwk(i);
        }
#endif // TFT3IN5
#if defined(EPD3IN7)      
//        EpdNwk(i);
//        EpdNwk(i);
#endif // EPD3IN7  
      }
  }
  
  zclReportCmd_t *pInAttrReport;
  pInAttrReport = (zclReportCmd_t *)pInMsg->attrCmd; 
  
  for (uint8 n = 0; n < (pInAttrReport->numAttr); n++ ){
    if (pInMsg->clusterId == TEMP && pInAttrReport->attrList[n].attrID == ATTRID_MS_TEMPERATURE_MEASURED_VALUE){
      if (temp_Temperature_Sensor_MeasuredValue[i] > BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1])){
        zclApp_UpDown[i] &= ~CB_TEMP; // down
      } else {
        zclApp_UpDown[i] |=  CB_TEMP; // up
      }
      temp_Temperature_Sensor_MeasuredValue[i] = BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1]);
      temp_bindClusterDev[i] |= CB_TEMP;
      old_bindClusterDev[i]  |= CB_TEMP;
#if defined(TFT3IN5)
      if (!zcl_game){      
        TftTemperature(i);
      }
#endif // TFT3IN5       
    }
    if (pInMsg->clusterId == HUMIDITY && pInAttrReport->attrList[n].attrID == ATTRID_MS_RELATIVE_HUMIDITY_MEASURED_VALUE){
      if (temp_HumiditySensor_MeasuredValue[i] > BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1])){
        zclApp_UpDown[i] &= ~CB_HUMIDITY; // down
      } else {
        zclApp_UpDown[i] |=  CB_HUMIDITY; // up
      }
      temp_HumiditySensor_MeasuredValue[i] = BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1]);
      temp_bindClusterDev[i] |= CB_HUMIDITY;
      old_bindClusterDev[i]  |= CB_HUMIDITY;
#if defined(TFT3IN5) 
      if (!zcl_game){      
        TftHumidity(i);
      }
#endif // TFT3IN5  
    }
    if (pInMsg->clusterId == PRESSURE && pInAttrReport->attrList[n].attrID == ATTRID_MS_PRESSURE_MEASUREMENT_MEASURED_VALUE){
      if (temp_PressureSensor_MeasuredValue[i] > BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1])){
        zclApp_UpDown[i] &= ~CB_PRESSURE; // down
      } else {
        zclApp_UpDown[i] |=  CB_PRESSURE; // up
      }
      temp_PressureSensor_MeasuredValue[i] = BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1]);
      temp_bindClusterDev[i] |= CB_PRESSURE;
      old_bindClusterDev[i]  |= CB_PRESSURE;
#if defined(TFT3IN5)
      if (!zcl_game){      
        TftPressure(i);
      }
#endif // TFT3IN5        
    }
    if (pInMsg->clusterId == PRESSURE && pInAttrReport->attrList[n].attrID == ATTRID_MS_PRESSURE_MEASUREMENT_SCALE){
      temp_PressureSensor_ScaledValue[i] = BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1]);
//      temp_bindClusterDev[i] |= CB_PRESSURE;
//      old_bindClusterDev[i]  |= CB_PRESSURE;
    }
    if (pInMsg->clusterId == ILLUMINANCE && pInAttrReport->attrList[n].attrID == ATTRID_MS_ILLUMINANCE_MEASURED_VALUE){
      if (temp_bh1750IlluminanceSensor_MeasuredValue[i] > BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1])){
        zclApp_UpDown[i] &= ~CB_ILLUMINANCE; // down
      } else {
        zclApp_UpDown[i] |=  CB_ILLUMINANCE; // up
      }
      temp_bh1750IlluminanceSensor_MeasuredValue[i] = BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1]);
      temp_bindClusterDev[i] |= CB_ILLUMINANCE;
      old_bindClusterDev[i]  |= CB_ILLUMINANCE;
#if defined(TFT3IN5) 
      if (!zcl_game){
        TftIlluminance(i);
      }
#endif // TFT3IN5    
    }
    if (pInMsg->clusterId == POWER_CFG && pInAttrReport->attrList[n].attrID == ATTRID_POWER_CFG_BATTERY_PERCENTAGE_REMAINING){
      temp_Battery_PercentageRemainig[i] = BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1]);
      temp_bindClusterDev[i] |= CB_POWER_CFG;
      old_bindClusterDev[i]  |= CB_POWER_CFG; 
#if defined(TFT3IN5) 
      if (!zcl_game){
        TftBattery(i);
      }
#endif // TFT3IN5        
    }
    if (pInMsg->clusterId == OCCUPANCY && pInAttrReport->attrList[n].attrID == ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY){
      if (temp_Occupied[i] > BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1])){
        zclApp_UpDown[i] &= ~CB_OCCUPANCY; // down
      } else {
        zclApp_UpDown[i] |=  CB_OCCUPANCY; // up
      }
      temp_Occupied[i] = BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1]);
      temp_bindClusterDev[i] |= CB_OCCUPANCY;
      old_bindClusterDev[i]  |= CB_OCCUPANCY;
#if defined(TFT3IN5)
      if (!zcl_game){      
        TftOccupancy(i);
      }
#endif // TFT3IN5         
    }
    if (pInMsg->clusterId == BINARY_INPUT && pInAttrReport->attrList[n].attrID == ATTRID_GEN_BINARY_INPUT_PRESENTVALUE){
      if (temp_Binary[i] > BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1])){
        zclApp_UpDown[i] &= ~CB_BINARY_INPUT; // down
      } else {
        zclApp_UpDown[i] |=  CB_BINARY_INPUT; // up
      }
      temp_Binary[i] = BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1]);
      temp_bindClusterDev[i] |= CB_BINARY_INPUT;
      old_bindClusterDev[i]  |= CB_BINARY_INPUT;
#if defined(TFT3IN5)
      if (!zcl_game){      
        TftBinary(i);
      }
#endif // TFT3IN5       
    }
    
    if (pInMsg->clusterId == ZCL_CO2 && pInAttrReport->attrList[n].attrID == ATTRID_CO2_MEASURED_VALUE){
      float value_float;
      memcpy(&value_float, pInAttrReport->attrList[n].attrData, sizeof(float) );
      if (temp_scd4xCO2_Sensor_MeasuredValue[i] > (uint16)(value_float * 1000000)){
        zclApp_UpDown[i] &= ~CB_CO2; // down
      } else {
        zclApp_UpDown[i] |=  CB_CO2; // up
      }
      temp_scd4xCO2_Sensor_MeasuredValue[i] = (uint16)(value_float * 1000000);
      temp_bindClusterDev[i] |= CB_CO2;
      old_bindClusterDev[i]  |= CB_CO2;
#if defined(TFT3IN5) 
      if (!zcl_game){
        TftCO2(i);
      }
#endif // TFT3IN5    
    }
    
#if defined(EPD3IN7) 
    EpdRefresh();
#endif // EPD3IN7    
  }
}
#endif  // ZCL_REPORT_DESTINATION_DEVICE

static void zclApp_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg )
{
  LREP("zclHdr.commandID=0x%X\r\n", pInMsg->zclHdr.commandID);
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
#endif
    case ZCL_CMD_CONFIG_REPORT:
    case ZCL_CMD_CONFIG_REPORT_RSP:
    case ZCL_CMD_READ_REPORT_CFG:
    case ZCL_CMD_READ_REPORT_CFG_RSP:
#ifdef ZCL_REPORT_DESTINATION_DEVICE
    case ZCL_CMD_REPORT:
      switch ( ((zclIncomingMsg_t *)pInMsg)->clusterId )
      {
        case ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT:
          zclApp_AttrIncomingReport( pInMsg );
          break;
        case ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY:
          zclApp_AttrIncomingReport( pInMsg );
          break;
        case ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT:
          zclApp_AttrIncomingReport( pInMsg );
          break;
        case ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT:
          zclApp_AttrIncomingReport( pInMsg );
          break;
        case ZCL_CLUSTER_ID_GEN_POWER_CFG:
          zclApp_AttrIncomingReport( pInMsg );
          break;
        case ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING:
          zclApp_AttrIncomingReport( pInMsg );
          break;
        case ZCL_CLUSTER_ID_GEN_BINARY_INPUT_BASIC:
          zclApp_AttrIncomingReport( pInMsg );
          break;
        case ZCL_CO2:
          zclApp_AttrIncomingReport( pInMsg );
          break;
        default:
          break;
      }
      break;  
#endif // ZCL_REPORT_DESTINATION_DEVICE
    case ZCL_CMD_DEFAULT_RSP:
      zclApp_DefaultRspCmd( pInMsg );
      
      break;
    default:
      break;
  }
}

static void zclApp_DefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
      zclDefaultRspCmd_t *pInDefaultRspCmd;
      pInDefaultRspCmd = (zclDefaultRspCmd_t *)pInMsg;
      LREP("commandID=0x%X\r\n", pInDefaultRspCmd->commandID);
      LREP("statusCode=0x%X\r\n", pInDefaultRspCmd->statusCode);
}

#ifdef HAL_LCD_PWM_PORT1
static void InitLedPWM(uint8 level){
    PERCFG &= ~0x20; //select of alternative 1 for timer 3
    P2SEL |= 0x20; // Timer 3 priority over USART1
//    P2DIR |= 0xC0; // priority timer 1 channels 2-3
    P1SEL |= BV(4); // p1.4 periferal
    P1DIR |= BV(4); // p1.4 output
  
    T3CTL &= ~BV(4); // Stop timer 3 (if it was running)
    T3CTL |= BV(2);  // Clear timer 3
    T3CTL &= ~0x08;  // Disable Timer 3 overflow interrupts
    T3CTL |= 0x03;   // Timer 3 mode = 3 - Up/Down

    T3CCTL1 &= ~0x40; // Disable channel 0 interrupts
    T3CCTL1 |= BV(2); // Ch0 mode = compare
    T3CCTL1 |= BV(4); // Ch0 output compare mode = toggle on compare

    T3CTL &= ~(BV(7) | BV(6) | BV(5)); // Clear Prescaler divider value
//    T3CTL |= 0xA0; // Set prescaler divider value = tick frequency/32
    T3CTL |= 0x20; // Set prescaler divider value = tick frequency/2
    T3CC0 = 0xFF;  // Set ticks PWM signal period
    T3CC1 = level;   // Set ticks PWM Duty Cycle
    
    T3CTL |= BV(4);  // Start timer 3
}
#endif

#ifdef TFT3IN5

const char buttonlabels_const[14][6][6] = { // [menu][butt][number]
                                       " ", " ", "MENU", " ",  " ", " ",     //0
                                       "NET", "DISP", "GAME", "BEEP", " ", "EXIT", //1
                                       "COLOR", "INVER", "ROTAT", " ", "BACKL", "EXIT", //2
                                       "Red", "Green", "Blue", " ", " ", "EXIT", //3
                                       " ", "LIGHT", "DARK", " ", " ", "EXIT", //4
                                       " ", "PORTR", "LANDS", " ", " ", "EXIT", //5
                                       "ALL", "TEMP", "HUMID", "PRESS", ">>>", "EXIT", //6
                                       "BATT", "ILLUM", "CO2", "OCCUP", "MODE", "EXIT", //7
                                       "REMOV", "REPOR", "TIME", "LQI", "BIND", "EXIT", //8
                                       "MAX", "AUTO", " ", " ", " ", "EXIT", //9
                                       "LQI0", "LQI1", "LQI2", " ", " ", "EXIT", //10
                                       "BND0", "BND1", "BND2", " ", " ", "EXIT", //11
                                       "JOIN", " ", " ", " ", " ", "EXIT", //12
                                       "SONG1", "SONG2", "STOP", " ", " ", "EXIT", //13
}; 

static char buttonlabels[1][6][6] = { // [menu][butt][number]
                                       " ", " ", "MENU", " ",  " ", " ",     //0
}; 
static char zclApp_status_string[20] = "";
static uint8 zclApp_butt = 0;

static void zclApp_create_butt_main(uint8 block) {
  zclApp_automenu();
  
  LCD_SetArealColor(0, 0, 320, 119, zclApp_lcd_background);

  if (block != 0){
    GUI_DisString_EN(32, 0, zclApp_status_string, &Font16, zclApp_lcd_background, WHITE);
    for (uint8 row=0; row<2; row++) {
      for (uint8 col=0; col<3; col++) {
          initButton(col + row*3, 4+(100/2)+col*(100+6), 16+(48/2)+row*(48+4),    // i, x, y, w, h, outline, fill, text
                  100, 48,  WHITE, color_scheme[scheme][4], color_scheme[scheme][2],
                  buttonlabels[0][col + row*3], 2);
        if (buttonlabels[0][col + row*3][0] != ' ') {
          drawButton(col + row*3, 0);
        }  
      }
    }
  } else {
    memset(zclApp_status_string,0,20);
    TftStatus(0);
    TftTimeDateWeek();
    initButton(2, 160+2+48+4+48+4+24 , 16+24,    // index, x, y
               48, 48,  WHITE, color_scheme[scheme][4],  color_scheme[scheme][2], //w, h, outline color, fill color, text color
               "M", 2); 
    drawButton(2, 0);
  }
}

static void zclApp_automenu(void) {
  //copy from constant to variable
  for (uint8 m = 0; m < 6; m++){
    for (uint8 n = 0; n < 6; n++){
      buttonlabels[0][m][n] = buttonlabels_const[zclApp_menu][m][n];
    }
  }
  //form the address of the device in the button
  if (zclApp_menu == 10 || zclApp_menu == 11){
    for (uint8 i = 0; i < 3; i++){
      if (temp_Sender_shortAddr[i] == 0xFFFE) {
                        buttonlabels[0][i][0] = ' ';
      } else {                        
        buttonlabels[0][i][0] = temp_Sender_shortAddr[i] / 4096 %16 + '0';
        if ((temp_Sender_shortAddr[i]/4096 %16) > 9){
          buttonlabels[0][i][0] = buttonlabels[0][i][0]+7;
        }  
        buttonlabels[0][i][1] = temp_Sender_shortAddr[i] / 256 %16 + '0';
        if ((temp_Sender_shortAddr[i]/256 %16) > 9){
          buttonlabels[0][i][1] = buttonlabels[0][i][1]+7;
        }
        buttonlabels[0][i][2] = temp_Sender_shortAddr[i] / 16 %16 + '0';
        if ((temp_Sender_shortAddr[i]/16 %16) > 9){
          buttonlabels[0][i][2] = buttonlabels[0][i][2]+7;
        }
        buttonlabels[0][i][3] = temp_Sender_shortAddr[i] %16 + '0';
        if ((temp_Sender_shortAddr[i] %16) > 9){
          buttonlabels[0][i][3] = buttonlabels[0][i][3]+7;
        }                        
      }                    
    }
  }
  if (zclApp_menu == 3){
    if (zclApp_Config.HvacUiDisplayMode & DM_RED) {
      buttonlabels[0][0][0] = ' ';
    }
    if (zclApp_Config.HvacUiDisplayMode & DM_GREEN) {
      buttonlabels[0][1][0] = ' ';
    }
    if (zclApp_Config.HvacUiDisplayMode & DM_BLUE) {
      buttonlabels[0][2][0] = ' ';
    }     
  }
  if (zclApp_menu == 4){
    if (zclApp_Config.HvacUiDisplayMode & DM_INVERT_NOT) {
      buttonlabels[0][1][0] = ' ';
    } else {
      buttonlabels[0][2][0] = ' ';
    }   
  }
  if (zclApp_menu == 5){
    if (zclApp_Config.HvacUiDisplayMode & DM_ROTATE) {
      buttonlabels[0][1][0] = ' ';
    } else {
      buttonlabels[0][2][0] = ' ';
    }   
  }
  if (zclApp_menu == 8){
    if (temp_Sender_shortAddr[0] == 0xFFFE && temp_Sender_shortAddr[1] == 0xFFFE && temp_Sender_shortAddr[2] == 0xFFFE) {
      buttonlabels[0][3][0] = ' ';
      buttonlabels[0][4][0] = ' ';
    }  
  }
  if (zclApp_menu == 9){
    if (zclApp_Config.HvacUiDisplayMode & DM_BACKLIGHT) {
      buttonlabels[0][0][0] = ' ';
    } else {
      buttonlabels[0][1][0] = ' ';
    }   
  }
  
}

void zclApp_keyprocessing(void) { 
            int8 butt = pressButton();
            //beep at press button
            if (butt != -1 && buttonlabels[0][butt][0] != ' '){
              beeping_beep_delay(400,20);
            } else {
              return;
            }
            zclApp_butt = butt;
            //status string
            if ((zclApp_menu !=6 && zclApp_menu !=7) || (zclApp_menu == 0 && butt == 2)){
              uint8 n = osal_strlen(buttonlabels[0][zclApp_butt]);
              uint8 m = osal_strlen(zclApp_status_string);
              for (uint8 i=0; i < n; i++) {
                zclApp_status_string[m+i] = buttonlabels[0][zclApp_butt][i];
              }
              n = osal_strlen(zclApp_status_string);
              zclApp_status_string[n] = ':';
              zclApp_status_string[n + 1] = '\0';
            }
            // update inverse press button
            drawButton(butt, 0); // update press
            
            switch (zclApp_menu){
              case 0: //zclApp_menu=0
                switch (butt){
                  case 0:
                    
                    break;
                  case 1:
                    
                    break;
                  case 2:
                    zclApp_menu = 1;
                    zclApp_create_butt_main(zclApp_menu);                    
                    break;
                }
                break;
              case 1://zclApp_menu=1
                switch (butt){
                  case 0:
                    if (bdbAttributes.bdbNodeIsOnANetwork == 1) {
                      zclApp_menu = 8;
                      zclApp_create_butt_main(zclApp_menu);
                    } else {
                      zclApp_menu = 12;
                      zclApp_create_butt_main(zclApp_menu);
                    }
                    break;
                  case 1:
                    zclApp_menu = 2;
                    zclApp_create_butt_main(zclApp_menu);
                    break;
                  case 2:
#if defined(BREAKOUT)                
                    zcl_game = 1;
                    xpt2046_mode = 0;
                    breakout_start();
#endif                     
                    break;
                  case 3:
                    zclApp_menu = 13;
                    zclApp_create_butt_main(zclApp_menu);
                    break;
                  case 4:
                    
                    break;
                }
                break;
              case 2://zclApp_menu=2
               switch (butt){
                  case 0:
                    zclApp_menu = 3;
                    zclApp_create_butt_main(zclApp_menu); //COLOR
                    break;
                  case 1:
                    zclApp_menu = 4;
                    zclApp_create_butt_main(zclApp_menu); //INVERT
                    break;
                  case 2:
                    zclApp_menu = 5;
                    zclApp_create_butt_main(zclApp_menu); //ROTATE                   
                    break;
                  case 3:
                    
                    break;
                  case 4:
                    zclApp_menu = 9;
                    zclApp_create_butt_main(zclApp_menu); //BACKL                   
                    break;
                }
                break;
              case 3://zclApp_menu=3
                zclApp_Config.HvacUiDisplayMode &= ~(DM_GREEN | DM_RED | DM_BLUE);//clear color scheme
                switch (butt){
                  case 0:
                    scheme = 1;  // RED
                    zclApp_Config.HvacUiDisplayMode |= DM_RED;
                    zclApp_ConfigDisplay();
                    osal_start_timerEx(zclApp_TaskID, APP_SAVE_ATTRS_EVT, 200);
                    break;
                  case 1:
                    scheme = 2; // GREEN
                    zclApp_Config.HvacUiDisplayMode |= DM_GREEN;
                    zclApp_ConfigDisplay();
                    osal_start_timerEx(zclApp_TaskID, APP_SAVE_ATTRS_EVT, 200);
                    break;
                  case 2:
                    scheme = 0; // BLUE
                    zclApp_Config.HvacUiDisplayMode |= DM_BLUE;
                    zclApp_ConfigDisplay();
                    osal_start_timerEx(zclApp_TaskID, APP_SAVE_ATTRS_EVT, 200);
                    break;
                }
                break;
              case 4://zclApp_menu=4
                switch (butt){
                  case 0:

                    break;
                  case 1:
                    zclApp_Config.HvacUiDisplayMode |= DM_INVERT_NOT; //invert
                    zclApp_ConfigDisplay();
                    osal_start_timerEx(zclApp_TaskID, APP_SAVE_ATTRS_EVT, 200);
                    break;
                  case 2:
                    zclApp_Config.HvacUiDisplayMode &= ~DM_INVERT_NOT;//invert
                    zclApp_ConfigDisplay();
                    osal_start_timerEx(zclApp_TaskID, APP_SAVE_ATTRS_EVT, 200);
                    break;
                }
                break;
              case 5://zclApp_menu=5
                switch (butt){
                  case 0:

                    break;
                  case 1:
                    zclApp_Config.HvacUiDisplayMode |= DM_ROTATE; //rotate
                    zclApp_ConfigDisplay();
                    osal_start_timerEx(zclApp_TaskID, APP_SAVE_ATTRS_EVT, 200);
                    break;
                  case 2:
                    zclApp_Config.HvacUiDisplayMode &= ~DM_ROTATE; //rotate
                    zclApp_ConfigDisplay();
                    osal_start_timerEx(zclApp_TaskID, APP_SAVE_ATTRS_EVT, 200);
                    break;
                }
                break;
              case 6://zclApp_menu=6
                switch (butt){
                  case 0:
                    report = 1;
                    zclApp_Report();
                    break;
                  case 1:
                    report = 1;
                    zclApp_ReadBME280Temperature();
                    report = 0;
                    break;
                  case 2:
                    report = 1;
                    zclApp_ReadBME280Humidity();
                    report = 0;
                    break;
                  case 3:
                    report = 1;
                    zclApp_ReadBME280Pressure();
                    report = 0;
                    break;
                  case 4:
                    zclApp_menu = 7;
                    zclApp_create_butt_main(zclApp_menu);
                    break;                  
                }
                break;
              case 7://zclApp_menu=7
                switch (butt){
                  case 0:
                    report = 1;
                    zclBattery_Report();
                    report = 0;
                    break;
                  case 1:
                    report = 1;
                    osal_stop_timerEx(zclApp_TaskID, APP_BH1750_DELAY_EVT);
                    osal_clear_event(zclApp_TaskID, APP_BH1750_DELAY_EVT);
                    zclApp_bh1750StartLumosity();
                    break;
                  case 2:
                    report = 1;
                    zclApp_scd4xReadCO2();
                    report = 0;
                    break;
                  case 3:
//                    bdb_RepChangedAttrValue(zclApp_ThirdEP.EndPoint, OCCUPANCY, ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY);
                    zclRep_Occupancy();
                    break;
                  case 4:
                    zclApp_ConfigDisplay();
                    break;                  
                }
                break;
              case 8://zclApp_menu=8                
                switch (butt){
                  case 0:
                    if (bdbAttributes.bdbNodeIsOnANetwork == 1) {
                      bdb_resetLocalAction();
                    }
                    break;
                  case 1:
                    zclApp_menu = 6;
                    zclApp_create_butt_main(zclApp_menu);
                    break;
                  case 2:
                    zclApp_LocalTime();
                    break;
                  case 3:
                    zclApp_menu = 10;
                    zclApp_create_butt_main(zclApp_menu);                    
                    break;
                  case 4:
                    zclApp_menu = 11;
                    zclApp_create_butt_main(zclApp_menu);
                    break;                  
                }
                break;
              case 9://zclApp_menu=9
                switch (butt){
                  case 0:
                    InitLedPWM(254 - MAX_LEVEL_PWM);
                    zclApp_Config.HvacUiDisplayMode |= DM_BACKLIGHT; //backlight
                    zclApp_ConfigDisplay();
                    osal_start_timerEx(zclApp_TaskID, APP_SAVE_ATTRS_EVT, 200);
                    break;
                  case 1:
                    zclApp_Config.HvacUiDisplayMode &= ~DM_BACKLIGHT; //backlight
                    zclApp_ConfigDisplay();
                    osal_start_timerEx(zclApp_TaskID, APP_SAVE_ATTRS_EVT, 200);
                    break;
                  case 2:
                    
                    break;
                  case 3:

                    break;
                  case 4:

                    break;                  
                }
                break;
              case 10://zclApp_menu=10                
                switch (butt){                 
                  case 0:
                    if ((bdbAttributes.bdbNodeIsOnANetwork == 1) && (temp_Sender_shortAddr[0] != 0xFFFE) ) {
#ifdef LQI_REQ                      
                      temp_countReqLqi = 0;
                      zclApp_RequestLqi();
                      zclApp_menu = 0;
                      zclApp_create_butt_main(zclApp_menu);  
#endif                      
                    }                      
                    break;
                  case 1:
                    if ((bdbAttributes.bdbNodeIsOnANetwork == 1) && (temp_Sender_shortAddr[1] != 0xFFFE)) {
#ifdef LQI_REQ                      
                      temp_countReqLqi = 1;
                      zclApp_RequestLqi();
                      zclApp_menu = 0;
                      zclApp_create_butt_main(zclApp_menu); 
#endif                      
                    }  
                    break;
                  case 2:
                    if ((bdbAttributes.bdbNodeIsOnANetwork == 1) && (temp_Sender_shortAddr[2] != 0xFFFE)) {
#ifdef LQI_REQ                      
                      temp_countReqLqi = 2;
                      zclApp_RequestLqi();
                      zclApp_menu = 0;
                      zclApp_create_butt_main(zclApp_menu);
#endif                      
                    }                     
                    break;
                  case 3:

                    break;
                  case 4:

                    break;                  
                }
                break;
              case 11://zclApp_menu=11                
                switch (butt){                 
                  case 0:
                    if ((bdbAttributes.bdbNodeIsOnANetwork == 1) && (temp_Sender_shortAddr[0] != 0xFFFE) ) {
#ifdef BIND_REQ                      
                      temp_countReqBind = 0;
                      temp_bindingCount[temp_countReqBind] = 0;
                      temp_bindingStartIndex[temp_countReqBind] = 0;
                      temp_bindClusterDev[temp_countReqBind] = 0x0000;
//                      osal_start_timerEx(zclApp_TaskID, APP_REQ_BIND_EVT, 100);
                      zclApp_RequestBind();
                      zclApp_menu = 0;
                      zclApp_create_butt_main(zclApp_menu);
#endif                      
                    }                      
                    break;
                  case 1:
                    if ((bdbAttributes.bdbNodeIsOnANetwork == 1) && (temp_Sender_shortAddr[1] != 0xFFFE)) {
#ifdef BIND_REQ                      
                      temp_countReqBind = 1;
                      temp_bindingCount[temp_countReqBind] = 0;
                      temp_bindingStartIndex[temp_countReqBind] = 0;
                      temp_bindClusterDev[temp_countReqBind] = 0x0000;
//                      osal_start_timerEx(zclApp_TaskID, APP_REQ_BIND_EVT, 100);
                      zclApp_RequestBind();
                      zclApp_menu = 0;
                      zclApp_create_butt_main(zclApp_menu);
#endif                      
                    }  
                    break;
                  case 2:
                    if ((bdbAttributes.bdbNodeIsOnANetwork == 1) && (temp_Sender_shortAddr[2] != 0xFFFE)) {
#ifdef BIND_REQ                      
                      temp_countReqBind = 2;
                      temp_bindingCount[temp_countReqBind] = 0;
                      temp_bindingStartIndex[temp_countReqBind] = 0;
                      temp_bindClusterDev[temp_countReqBind] = 0x0000;
//                      osal_start_timerEx(zclApp_TaskID, APP_REQ_BIND_EVT, 100);
                      zclApp_RequestBind();
                      zclApp_menu = 0;
                      zclApp_create_butt_main(zclApp_menu);
#endif                      
                    }                     
                    break;
                  case 3:

                    break;
                  case 4:

                    break;                  
                }
                break;
              case 12://zclApp_menu=12                
                switch (butt){                 
                  case 0:
                    if (bdbAttributes.bdbNodeIsOnANetwork == 0) {
                      bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_STEERING | BDB_COMMISSIONING_MODE_FINDING_BINDING);
                      TftUpdateRefresh();
                    }                    
                    break;
                  case 1:
 
                    break;
                  case 2:
                   
                    break;
                  case 3:

                    break;
                  case 4:

                    break;                  
                }
                break;
              case 13://zclApp_menu=13                
                switch (butt){                 
                  case 0:
                    beeping_seq_start(1);
                    break;
                  case 1:
                    beeping_seq_start(2);
                    break;
                  case 2:
                    beeping_seq_stop();
                    break;
                  case 3:

                    break;
                  case 4:

                    break;                  
                }
                break;
            }

            if (butt == 5 && zclApp_menu != 0) {
              zclApp_menu = 0;
              zclApp_create_butt_main(zclApp_menu);
            } 
         
}

void zclApp_TPkeyprocessing(void) {          
          if (!zcl_game){
            xpt2046_mode = 1;
            zclApp_keyprocessing();
          } else {
#if defined(BREAKOUT)
            xpt2046_mode = 0;
            breakout_keyprocessing();
            TftUpdateRefresh(); 
#endif // BREAKOUT 
          }
}

#endif


/****************************************************************************
****************************************************************************/
