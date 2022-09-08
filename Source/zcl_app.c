
#include "AF.h"
#include "OSAL.h"
#include "OSAL_Clock.h"
#include "OSAL_PwrMgr.h"
#include "ZComDef.h"
#include "ZDApp.h"
#include "ZDNwkMgr.h"
#include "ZDObject.h"
#include "math.h"

#include "nwk_util.h"
#include "zcl.h"
#include "zcl_app.h"
#include "zcl_diagnostic.h"
#include "zcl_general.h"
//#include "zcl_lighting.h"
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
#include "battery.h"
#include "commissioning.h"
#include "factory_reset.h"
#include "utils.h"
#include "version.h"

#ifdef EPD2IN9
#include "epd2in9.h"
#endif
#ifdef EPD2IN9V2
#include "epd2in9v2.h"
#endif
#ifdef EPD2IN13V2
#include "epd2in13v2.h"
#endif
#ifdef EPD1IN54V2
#include "epd1in54v2.h"
#endif
#ifdef EPD3IN7
#include "epd3in7.h"
#include "epdpaint.h"
#endif
#include "imagedata.h"
#ifdef TFT3IN5
#include "tft3in5.h"
#include "xpt2046.h"
#include "lcdgui.h"
#endif

#define HAL_LCD_BUSY BNAME(HAL_LCD_BUSY_PORT, HAL_LCD_BUSY_PIN)

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
bool bmeDetect = 0;
uint8 motionDetect = 0;
uint8 bh1750Detect = 0;
uint8 BH1750_mode = ONE_TIME_HIGH_RES_MODE;

uint16 temp_bh1750IlluminanceSensor_MeasuredValue[3];
uint16 old_bh1750IlluminanceSensor_MeasuredValue;
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

bool EpdDetect = 1; 
uint8 fullupdate_hour = 0;
uint8 zclApp_color = 1;
uint8 zclApp_EpdUpDown[3] = {0, 0, 0}; 
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
// enable/disable display of values
// bit 0 - 0x0001 POWER_CFG, 1 - 0x0400 ILLUMINANCE, 2 - 0x0402 TEMP, 3 - 0x0403 PRESSURE, 
//     4 - 0x0405 HUMIDITY,  5 - 0x0406 OCCUPANCY,   6 - none       , 7 - table received 
uint8 temp_bindClusterDev[3] = {0x00, 0x00, 0x00};
uint8 old_bindClusterDev[3]  = {0x00, 0x00, 0x00};
uint16 zlcApp_ExtAddr = 0xFFFE;
#endif

afAddrType_t inderect_DstAddr = {.addrMode = (afAddrMode_t)AddrNotPresent, .endPoint = 0, .addr.shortAddr = 0};

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

#if defined(EPD3IN7)
static void zclApp_EpdUpdateClock(void);
static void EpdRefresh(void);
static void EpdtestRefresh(void);
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
static void TftRefresh(void);
static void TfttestRefresh(void);
static void TftTimeDateWeek(void);
static void TftStatus(uint8 temp_s);
static void TftBindStatus(uint8 temp_s);
static void TftLqi(uint8 temp_l);
static void TftNwk(uint8 temp_n);
static void TftBattery(uint8 temp_b);
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
    
    // this is important to allow connects throught routers
    // to make this work, coordinator should be compiled with this flag #define TP2_LEGACY_ZC
    requestNewTrustCenterLinkKey = FALSE;

    zclApp_TaskID = task_id;

    zclGeneral_RegisterCmdCallbacks(1, &zclApp_CmdCallbacks);
    zcl_registerAttrList(zclApp_FirstEP.EndPoint, zclApp_AttrsFirstEPCount, zclApp_AttrsFirstEP);
    bdb_RegisterSimpleDescriptor(&zclApp_FirstEP);
    zcl_registerReadWriteCB(zclApp_FirstEP.EndPoint, NULL, zclApp_ReadWriteAuthCB);
/*
    zcl_registerAttrList(zclApp_SecondEP.EndPoint, zclApp_AttrsSecondEPCount, zclApp_AttrsSecondEP);
    bdb_RegisterSimpleDescriptor(&zclApp_SecondEP);
    zcl_registerReadWriteCB(zclApp_SecondEP.EndPoint, NULL, zclApp_ReadWriteAuthCB);
*/    
    zcl_registerAttrList(zclApp_ThirdEP.EndPoint, zclApp_AttrsThirdEPCount, zclApp_AttrsThirdEP);
    bdb_RegisterSimpleDescriptor(&zclApp_ThirdEP);
    zcl_registerReadWriteCB(zclApp_ThirdEP.EndPoint, NULL, zclApp_ReadWriteAuthCB);
    
    zcl_registerAttrList(zclApp_FourthEP.EndPoint, zclApp_AttrsFourthEPCount, zclApp_AttrsFourthEP);
    bdb_RegisterSimpleDescriptor(&zclApp_FourthEP);   
    zcl_registerReadWriteCB(zclApp_FourthEP.EndPoint, NULL, zclApp_ReadWriteAuthCB);

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
  LCD_SCAN_DIR Lcd_ScanDir = SCAN_DIR_DFT;    //SCAN_DIR_DFT = D2U_L2R
  LCD_Init( Lcd_ScanDir, 1);
  GUI_Clear(WHITE);
  zclApp_SetTimeDate();  
  TftRefresh();
/*  
  GUI_Show(); 
  DEV_TIME sDev_time;
  sDev_time.Hour = 23;
  sDev_time.Min = 38;
  sDev_time.Sec = 56;
  GUI_Showtime(200, 150, 327, 197, &sDev_time, RED);
*/
/*
  TP_Init( Lcd_ScanDir );
  TP_GetAdFac();
  TP_Dialog();
*/
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
    if (zclApp_Config.HvacUiDisplayMode & 0x01){
      zclApp_color = 0xFF;
    } else {
      zclApp_color = 0x00;
    }
    // epd full screen
    EpdInitFull();
    EpdSetFrameMemoryBase(IMAGE_DATA, zclApp_Config.HvacUiDisplayMode & 0x01);
    EpdDisplayFrame();
    _delay_ms(2000);

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
            // bit 0 - 0x0001 POWER_CFG, 1 - 0x0400 ILLUMINANCE, 2 - 0x0402 TEMP, 3 - 0x0403 PRESSURE, 
            //     4 - 0x0405 HUMIDITY,  5 - 0x0406 OCCUPANCY,   6 - none       , 7 - table received
            switch (BindRsp->list[x].clusterID){
              case POWER_CFG:
                temp_bindClusterDev[bdev] |= BV(0);
                break;
              case ILLUMINANCE:
                temp_bindClusterDev[bdev] |= BV(1);
                break;
              case TEMP:
                temp_bindClusterDev[bdev] |= BV(2);
                break;
              case PRESSURE:
                temp_bindClusterDev[bdev] |= BV(3);
                break;
              case HUMIDITY:
                temp_bindClusterDev[bdev] |= BV(4);
                break;
              case OCCUPANCY:
                temp_bindClusterDev[bdev] |= BV(5);
                break;
            }           
          }
        }
        
        if (temp_bindingCount[bdev] != 0 && temp_bindingCount[bdev] - temp_bindingStartIndex[bdev] > 3){
//          LREPMaster("BIND_BIND_BIND\r\n");
          temp_bindingStartIndex[bdev] = temp_bindingStartIndex[bdev] +3;
          temp_countReqBind = bdev;
          zclApp_RequestBind();
//          osal_start_timerEx(zclApp_TaskID, APP_REQ_BIND_EVT, 1000);
        } else {
          LREPMaster("BIND_BIND_0\r\n");
          temp_bindingCount[bdev] = 0;
          temp_bindingStartIndex[bdev] = 0;
          if (temp_bindClusterDev[bdev] == 0x00) {
            temp_Sender_shortAddr[bdev] = 0xFFFE;
            temp_LqiStartIndex[bdev] = 0;
            temp_lqi[bdev] = 255;
          } else {
            temp_bindClusterDev[bdev] |= BV(7);
          }
          if (temp_bindClusterDev[bdev] != old_bindClusterDev[bdev]) {
            old_bindClusterDev[bdev] = temp_bindClusterDev[bdev];
          }
#if defined(EPD3IN7)
          EpdRefresh();
#endif
#if defined(TFT3IN5)
          TftRefresh();
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
#endif
#if defined(TFT3IN5)      
      TftRefresh();
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
        if (EpdDetect == 1) {
          if (zclApp_Config.HvacUiDisplayMode & 0x01){
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
        }
        
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
#endif //  EPD3IN7 
#if defined(TFT3IN5)                  
                  TftRefresh();
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
      
      if(zclApp_Occupied == 1 || bdbAttributes.bdbNodeIsOnANetwork == 0) {  
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
        temp_bindClusterDev[temp_countReqBind] = 0x00;
        zclApp_RequestBind();
#endif 
#if defined(EPD3IN7)        
        fullupdate_hour = fullupdate_hour +1;
        if (fullupdate_hour == 5){ // over 5 min clear          
          zclApp_EpdUpdateClock();          
          fullupdate_hour = 0;
        }
        EpdRefresh();
#endif // EPD3IN7
#if defined(TFT3IN5)        
        TftRefresh();
#endif        
      }
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
#if defined(EPD3IN7)          
          EpdRefresh();
#endif // EPD3IN7
#if defined(TFT3IN5)          
          TftRefresh();
#endif // TFT3IN5           
        }
        zclApp_Occupied = 1;
        zclApp_Occupied_OnOff = 1;
        zclGeneral_SendOnOff_CmdOn(zclApp_ThirdEP.EndPoint, &inderect_DstAddr, TRUE, bdb_getZCLFrameCounter());
        bdb_RepChangedAttrValue(zclApp_ThirdEP.EndPoint, OCCUPANCY, ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY);
//        EpdRefresh();
        
        return (events ^ APP_MOTION_ON_EVT);
    }
    
    if (events & APP_MOTION_OFF_EVT) {
        LREPMaster("APP_MOTION_OFF_EVT\r\n");
        //report
#if defined(EPD3IN7)        
        EpdRefresh();
#endif // EPD3IN7
#if defined(TFT3IN5)        
        TftRefresh();
#endif // TFT3IN5
        zclApp_Occupied = 0;
        zclApp_Occupied_OnOff = 0;
        zclGeneral_SendOnOff_CmdOff(zclApp_ThirdEP.EndPoint, &inderect_DstAddr, TRUE, bdb_getZCLFrameCounter());
        bdb_RepChangedAttrValue(zclApp_ThirdEP.EndPoint, OCCUPANCY, ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY);
//        EpdRefresh();
        
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
#if defined(TFT3IN5)
    if (events & APP_TFT_DELAY_EVT) {
        LREPMaster("APP_TFT_DELAY_EVT\r\n");
        TfttestRefresh();
        
        return (events ^ APP_TFT_DELAY_EVT);
    }
#endif // TFT3IN5     
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
    LREP("zclApp_HandleKeys portAndAction=0x%X keyCode=0x%X\r\n", portAndAction, keyCode);
    zclFactoryResetter_HandleKeys(portAndAction, keyCode);
    zclCommissioning_HandleKeys(portAndAction, keyCode);
    if (portAndAction & HAL_KEY_PRESS) {
        LREPMaster("Key press\r\n");
    }

    bool contact = portAndAction & HAL_KEY_PRESS ? TRUE : FALSE;
    uint8 endPoint = 0;
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
     LREP("contact=%d endpoint=%d\r\n", contact, endPoint);
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
    case 1:

          break;
    case 2:
      if (bmeDetect == 1){
          zclApp_ReadBME280Temperature();
      }
        break;
    case 3:
      if (bmeDetect == 1){
          zclApp_ReadBME280Pressure();
      }
        break;
    case 4:
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
      if (EpdDetect == 1){
        bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, HVAC_UI_CONFIG, ATTRID_HVAC_THERMOSTAT_UI_CONFIG_DISPLAY_MODE);
      }
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
        temp_countReqBind = 0;
        temp_bindingCount[temp_countReqBind] = 0;
        temp_bindingStartIndex[temp_countReqBind] = 0;
        temp_bindClusterDev[temp_countReqBind] = 0x00;
//        old_bindClusterDev[temp_countReqBind] = 0x00;
        zclApp_RequestBind();
#endif
        break;
/*        
    case 10:      
#ifdef IEEE_ADDR_REQ
        zclApp_RequestAddr();
#endif
        break;
*/
    default:
        osal_stop_timerEx(zclApp_TaskID, APP_READ_SENSORS_EVT);
        osal_clear_event(zclApp_TaskID, APP_READ_SENSORS_EVT);
        currentSensorsReadingPhase = 0;
        break;
    }
  }
  zclApp_StartReloadTimer();
}

static void zclApp_LocalTime(void) {
  bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, GEN_TIME, ATTRID_TIME_LOCAL_TIME);
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
//      if (temp_bh1750IlluminanceSensor_MeasuredValue[0] > zclApp_bh1750IlluminanceSensor_MeasuredValue){
//        zclApp_EpdUpDown &= ~BV(0); // down
//      } else {
//        zclApp_EpdUpDown |=  BV(0); // up
//      }
//      temp_bh1750IlluminanceSensor_MeasuredValue[0] = zclApp_bh1750IlluminanceSensor_MeasuredValue;
      old_bh1750IlluminanceSensor_MeasuredValue = zclApp_bh1750IlluminanceSensor_MeasuredValue;
      bdb_RepChangedAttrValue(zclApp_FourthEP.EndPoint, ILLUMINANCE, ATTRID_MS_ILLUMINANCE_MEASURED_VALUE);
#if defined(EPD3IN7)
      EpdRefresh();
#endif // EPD3IN7
    }
//    LREP("bh1750IlluminanceSensor_MeasuredValue value=%X\r\n", zclApp_bh1750IlluminanceSensor_MeasuredValue);
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
//          if (temp_Temperature_Sensor_MeasuredValue[0] > zclApp_Temperature_Sensor_MeasuredValue){
//            zclApp_EpdUpDown &= ~BV(1); // down
//          } else {
//            zclApp_EpdUpDown |=  BV(1); // up
//          }
//          temp_Temperature_Sensor_MeasuredValue[0] = zclApp_Temperature_Sensor_MeasuredValue;
          old_Temperature_Sensor_MeasuredValue = zclApp_Temperature_Sensor_MeasuredValue;
          bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, TEMP, ATTRID_MS_TEMPERATURE_MEASURED_VALUE);
#if defined(EPD3IN7)          
          EpdRefresh();
#endif // EPD3IN7
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
//          if (temp_PressureSensor_MeasuredValue[0] > zclApp_PressureSensor_MeasuredValue){
//            zclApp_EpdUpDown &= ~BV(3); // down
//          } else {
//            zclApp_EpdUpDown |=  BV(3); // up
//          }
//          temp_PressureSensor_MeasuredValue[0] = zclApp_PressureSensor_MeasuredValue;
//          temp_PressureSensor_ScaledValue[0] = zclApp_PressureSensor_ScaledValue;
          old_PressureSensor_MeasuredValue = zclApp_PressureSensor_MeasuredValue;
          old_PressureSensor_ScaledValue = zclApp_PressureSensor_ScaledValue;
          bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, PRESSURE, ATTRID_MS_PRESSURE_MEASUREMENT_MEASURED_VALUE);
#if defined(EPD3IN7)
          EpdRefresh();
#endif // EPD3IN7
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
        if (humid > zclApp_Config.MsHumidityMinAbsoluteChange || report == 1){ //10%
//          if (temp_HumiditySensor_MeasuredValue[0] > zclApp_HumiditySensor_MeasuredValue){
//            zclApp_EpdUpDown &= ~BV(2); // down
//          } else {
//            zclApp_EpdUpDown |=  BV(2); // up
//          }
//          temp_HumiditySensor_MeasuredValue[0] = zclApp_HumiditySensor_MeasuredValue;
          old_HumiditySensor_MeasuredValue = zclApp_HumiditySensor_MeasuredValue;
          bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, HUMIDITY, ATTRID_MS_RELATIVE_HUMIDITY_MEASURED_VALUE);
#if defined(EPD3IN7)          
          EpdRefresh();
#endif // EPD3IN7
        }
}

static void zclApp_Report(void) { osal_start_reload_timer(zclApp_TaskID, APP_READ_SENSORS_EVT, 100); }

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
#if defined(EPD3IN7)     
    zclApp_EpdUpdateClock();   
    EpdRefresh();
#endif // EPD3IN7
#if defined(TFT3IN5)    
    TftRefresh();
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
    if(zclApp_Occupied == 1 || bdbAttributes.bdbNodeIsOnANetwork == 0) {
      osal_start_timerEx(zclApp_TaskID, APP_EPD_DELAY_EVT, 2000);
    }
  }
}
#endif // EPD3IN7

#if defined(TFT3IN5) 
static void TftRefresh(void){
//  if (EpdDetect == 1) {
    if(zclApp_Occupied == 1 || bdbAttributes.bdbNodeIsOnANetwork == 0) {
      osal_start_timerEx(zclApp_TaskID, APP_TFT_DELAY_EVT, 2000);
    }
//  }
}
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

  char time_string[] = {'0', '0', ':', '0', '0', '\0'};
  time_string[0] = time.hour / 10 % 10 + '0';
  time_string[1] = time.hour % 10 + '0';
  time_string[3] = time.minutes / 10 % 10 + '0';
  time_string[4] = time.minutes % 10 + '0';

  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    GUI_DrawRectangle(100, 16, 220, 64, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
    GUI_DisString_EN(100, 16, time_string, &Font48, LCD_BACKGROUND, BLUE);
  } else { // landscape

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

  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    GUI_DrawRectangle(116, 64, 212, 80, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
    GUI_DisString_EN(116, 64, date_string, &Font16, LCD_BACKGROUND, BLUE);
  } else { //landscape

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
  
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    GUI_DrawRectangle(116, 80, 224, 96, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
    GUI_DisString_EN(116, 80, day_string, &Font16, LCD_BACKGROUND, BLUE);
  } else { //landscape

  }

}
#endif

#if defined(EPD3IN7)
static void EpdTimeDateWeek(void){
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

  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
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

  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
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
  
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
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

}
#endif

#if defined(TFT3IN5)
static void TftStatus(uint8 temp_s){
  uint8 row = temp_s *120;
  
  //status network
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    if ( bdbAttributes.bdbNodeIsOnANetwork ){
      GUI_DrawRectangle(8, 8, 24, 24, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_Disbitmap(8, 8 + row , IMAGE_ONNETWORK, 16, 16, BLUE, 0);
    } else {
      GUI_DrawRectangle(8, 8, 24, 24, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_Disbitmap(8, 8 + row , IMAGE_OFFNETWORK, 16, 16, BLUE, 0);
//      GUI_DrawRectangle(8, 8, 24, 30, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
//      GUI_Disbitmap(8, 8 + row , IMAGE_LQI_100, 25, 16, BLUE, 0);
    } 
  } else { // landscape    
    
  }
}
#endif

#if defined(EPD3IN7)
static void EpdStatus(uint8 temp_s){
  uint8 row = temp_s *120;
  
  //status network
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    
    PaintSetWidth(16);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_90);
    PaintSetInvert(!(zclApp_Config.HvacUiDisplayMode & 0x01));
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
    PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);    
  } else { // landscape    
    if ( bdbAttributes.bdbNodeIsOnANetwork ){
      EpdSetFrameMemoryImageXY(IMAGE_ONNETWORK, 264, 1, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_OFFNETWORK, 264, 1, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
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
}
#endif

#if defined(TFT3IN5)
static void TftBindStatus(uint8 temp_s){
    //status bind
  uint8 row = temp_s *120;
  char* bind_string = " ";
  if(temp_bindClusterDev[temp_s] & 0x80) {
    bind_string = "B";
  }

  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    GUI_DrawRectangle(8, 128 + row, 20, 128 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
    GUI_DisString_EN(8, 128 + row, bind_string, &Font16, LCD_BACKGROUND, BLUE);  
  } else { // landscape    
    
  }
}
#endif

#if defined(EPD3IN7)
static void EpdBindStatus(uint8 temp_s){
    //status bind
  uint8 row = temp_s *120;
  char* bind_string = " ";
  if(temp_bindClusterDev[temp_s] & 0x80) {
    bind_string = "B";
  }

  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait

    PaintSetWidth(16);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
   
    PaintDrawStringAt(0, 2, bind_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 8, 128 + row, PaintGetWidth(), PaintGetHeight());    
  } else { // landscape    
    
  }
}
#endif

#if defined(TFT3IN5)
static void TftLqi(uint8 temp_l){
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
  
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    if (temp_Sender_shortAddr[temp_l] != 0xFFFE) {
      if (temp_lqi[temp_l] != 255) { 
        GUI_DrawRectangle(32-1, 128-1 + row, 32+24, 128 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
        
        if(temp_lqi[temp_l] > 40){
          GUI_Disbitmap(32, 128 + row , IMAGE_LQI_100, 25, 16, BLUE, 0);          
        } else if (temp_lqi[temp_l] <= 40 && temp_lqi[temp_l] > 30) {
          GUI_Disbitmap(32, 128 + row , IMAGE_LQI_80, 25, 16, BLUE, 0);
        } else if (temp_lqi[temp_l] <= 30 && temp_lqi[temp_l] > 20) {
          GUI_Disbitmap(32, 128 + row , IMAGE_LQI_60, 25, 16, BLUE, 0);
        } else if (temp_lqi[temp_l] <= 20 && temp_lqi[temp_l] > 10) {
          GUI_Disbitmap(32, 128 + row , IMAGE_LQI_40, 25, 16, BLUE, 0);
        } else if (temp_lqi[temp_l] <= 10 && temp_lqi[temp_l] > 0) {
          GUI_Disbitmap(32, 128 + row , IMAGE_LQI_20, 25, 16, BLUE, 0);
        } else if (temp_lqi[temp_l] == 0) {
          GUI_Disbitmap(32, 128 + row , IMAGE_LQI_0, 25, 16, BLUE, 0);
        }       
      }
    } else {
      GUI_DrawRectangle(32-1, 128-1 + row, 64+36, 128 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );      
    }   
    GUI_DrawRectangle(64, 128 + row, 64+36, 128 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
    GUI_DisString_EN(64, 128 + row, lqi_string, &Font16, LCD_BACKGROUND, BLUE);    
  } else { // lanscape

  }
#endif  //LQI_REQ
}
#endif  // TFT3IN5

#if defined(EPD3IN7)
static void EpdLqi(uint8 temp_l){
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
  
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    if (temp_Sender_shortAddr[temp_l] != 0xFFFE) {
      if (temp_lqi[temp_l] != 255) { 
        PaintSetWidth(24);
        PaintSetHeight(16);
        PaintSetRotate(ROTATE_270);
        PaintSetInvert(!(zclApp_Config.HvacUiDisplayMode & 0x01));
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
        PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
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
        EpdSetFrameMemoryImageXY(IMAGE_LQI_100, 256, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else if (temp_lqi[0] <= 40 && temp_lqi[0] > 30) {
        EpdSetFrameMemoryImageXY(IMAGE_LQI_80, 256, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else if (temp_lqi[0] <= 30 && temp_lqi[0] > 20) {
        EpdSetFrameMemoryImageXY(IMAGE_LQI_60, 256, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else if (temp_lqi[0] <= 20 && temp_lqi[0] > 10) {
        EpdSetFrameMemoryImageXY(IMAGE_LQI_40, 256, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else if (temp_lqi[0] <= 10 && temp_lqi[0] > 0) {
        EpdSetFrameMemoryImageXY(IMAGE_LQI_20, 256, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else if (temp_lqi[0] == 0) {
        EpdSetFrameMemoryImageXY(IMAGE_LQI_0, 256, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      }
    }
  }
#endif  //LQI_REQ
}
#endif  // EPD3IN7

#if defined(TFT3IN5)
static void TftNwk(uint8 temp_n){  
  // nwkDevAddress
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

  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait   
    GUI_DrawLine(8, 120+1 + row, 312, 120+1 + row, BLUE, LINE_SOLID, DOT_PIXEL_DFT);
    
    GUI_DrawRectangle(208, 128 + row, 208+72, 128 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
    GUI_DisString_EN(208, 128 + row, nwk_string, &Font16, LCD_BACKGROUND, BLUE); 
  } else { // landscape  

  }
}
#endif

#if defined(EPD3IN7)
static void EpdNwk(uint8 temp_n){  
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

  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
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
}
#endif

#if defined(TFT3IN5)
static void TftBattery(uint8 temp_b){
  //percentage
  uint8 row = temp_b *120;
  char perc_string[] = {' ', ' ', ' ', ' ', '\0'};
  if (old_bindClusterDev[temp_b] & 0x01){
    if (temp_Battery_PercentageRemainig[temp_b] != 0xFF) {
      perc_string[0] = temp_Battery_PercentageRemainig[temp_b]/2 / 100 % 10 + '0';
      perc_string[1] = temp_Battery_PercentageRemainig[temp_b]/2 / 10 % 10 + '0';
      perc_string[2] = temp_Battery_PercentageRemainig[temp_b]/2 % 10 + '0';
      perc_string[3] = '%';
    }
  }

  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait    
    GUI_DrawRectangle(148, 128 + row, 148+48, 128 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
    GUI_DisString_EN(148, 128 + row, perc_string, &Font16, LCD_BACKGROUND, BLUE);  
    
    if (old_bindClusterDev[temp_b] & 0x01){
      if (temp_Battery_PercentageRemainig[temp_b] != 0xFF) {
        GUI_DrawRectangle(112-1, 128-1 + row, 112+24, 128 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
        
        if(temp_Battery_PercentageRemainig[temp_b]/2 > 75){
          GUI_Disbitmap(112, 128 + row , IMAGE_BATTERY_100, 25, 16, BLUE, 0);
        } else if (temp_Battery_PercentageRemainig[temp_b]/2 <= 75 && temp_Battery_PercentageRemainig[temp_b]/2 > 50) {
          GUI_Disbitmap(112, 128 + row , IMAGE_BATTERY_75, 25, 16, BLUE, 0);
        } else if (temp_Battery_PercentageRemainig[temp_b]/2 <= 50 && temp_Battery_PercentageRemainig[temp_b]/2 > 25) {
          GUI_Disbitmap(112, 128 + row , IMAGE_BATTERY_50, 25, 16, BLUE, 0);
        } else if (temp_Battery_PercentageRemainig[temp_b]/2 <= 25 && temp_Battery_PercentageRemainig[temp_b]/2 > 6) {
          GUI_Disbitmap(112, 128 + row , IMAGE_BATTERY_25, 25, 16, BLUE, 0);
        } else if (temp_Battery_PercentageRemainig[temp_b]/2 <= 6 && temp_Battery_PercentageRemainig[temp_b]/2 > 0) {
          GUI_Disbitmap(112, 128 + row , IMAGE_BATTERY_0, 25, 16, BLUE, 0);
        }
      }
    } else {
      GUI_DrawRectangle(112-1, 128-1 + row, 148+48, 128 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
    }      
  } else { // landscape

  }
}
#endif

#if defined(EPD3IN7)
static void EpdBattery(uint8 temp_b){
  //percentage
  uint8 row = temp_b *120;
  char perc_string[] = {' ', ' ', ' ', ' ', '\0'};
  if (old_bindClusterDev[temp_b] & 0x01){
    if (temp_Battery_PercentageRemainig[temp_b] != 0xFF) {
      perc_string[0] = temp_Battery_PercentageRemainig[temp_b]/2 / 100 % 10 + '0';
      perc_string[1] = temp_Battery_PercentageRemainig[temp_b]/2 / 10 % 10 + '0';
      perc_string[2] = temp_Battery_PercentageRemainig[temp_b]/2 % 10 + '0';
      perc_string[3] = '%';
    }
  }

  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    PaintSetWidth(48);
    PaintSetHeight(18);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 2, perc_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 144, 128 + row, PaintGetWidth(), PaintGetHeight());
    if (old_bindClusterDev[temp_b] & 0x01){
      if (temp_Battery_PercentageRemainig[temp_b] != 0xFF) {
        PaintSetWidth(24);
        PaintSetHeight(16);
        PaintSetRotate(ROTATE_270);
        PaintSetInvert(!(zclApp_Config.HvacUiDisplayMode & 0x01));
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
        PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01); 
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
        EpdSetFrameMemoryImageXY(IMAGE_BATTERY_100, 256, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else if (zclBattery_PercentageRemainig/2 <= 75 && zclBattery_PercentageRemainig/2 > 50) {
        EpdSetFrameMemoryImageXY(IMAGE_BATTERY_75, 256, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else if (zclBattery_PercentageRemainig/2 <= 50 && zclBattery_PercentageRemainig/2 > 25) {
        EpdSetFrameMemoryImageXY(IMAGE_BATTERY_50, 256, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else if (zclBattery_PercentageRemainig/2 <= 25 && zclBattery_PercentageRemainig/2 > 6) {
        EpdSetFrameMemoryImageXY(IMAGE_BATTERY_25, 256, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else if (zclBattery_PercentageRemainig/2 <= 6 && zclBattery_PercentageRemainig/2 > 0) {
        EpdSetFrameMemoryImageXY(IMAGE_BATTERY_0, 256, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      }
    }
  }
}
#endif

#if defined(TFT3IN5)
static void TftOccupancy(uint8 temp_oc){
  // Occupancy
  uint8 row = temp_oc *120;
  
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait 
    if (old_bindClusterDev[temp_oc] & 0x20){     
      GUI_DrawRectangle(0, 160 + row, 0+64, 160 + row + 64, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      if (temp_Occupied[temp_oc] == 0) {
        GUI_Disbitmap(0, 160 + row , IMAGE_MOTION_NOT, 64, 64, BLUE, 0);     
      } else {
        GUI_Disbitmap(0, 160 + row , IMAGE_MOTION, 64, 64, BLUE, 0);  
      }
    } else {
      GUI_DrawRectangle(0, 160 + row, 0+64, 160 + row + 64, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
    }
  } else { // landscape

  }
}
#endif

#if defined(EPD3IN7)
static void EpdOccupancy(uint8 temp_oc){
  // Occupancy
  char* occup_string = "";

  if (zclApp_Occupied == 0) {
//  if (zclApp_Occupied_OnOff == 0) {
    occup_string = "UnOccupied";
  } else {
    occup_string = " Occupied ";
  }
  uint8 row = temp_oc *120;
  
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait 
    if (old_bindClusterDev[temp_oc] & 0x20){
      PaintSetWidth(64);
      PaintSetHeight(64);
      PaintSetRotate(ROTATE_270);
      PaintSetInvert(!(zclApp_Config.HvacUiDisplayMode & 0x01));
      PaintClear(UNCOLORED);    
      if (temp_Occupied[temp_oc] == 0) {
        PaintDrawImage(IMAGE_MOTION_NOT, 0, 0, 64, 64, COLORED);
        EpdSetFrameMemoryXY(PaintGetImage(), 0, 160 + row, PaintGetWidth(), PaintGetHeight());     
      } else {
        PaintDrawImage(IMAGE_MOTION, 0, 0, 64, 64, COLORED);
        EpdSetFrameMemoryXY(PaintGetImage(), 0, 160 + row, PaintGetWidth(), PaintGetHeight()); 
      }
      PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
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
      EpdSetFrameMemoryImageXY(IMAGE_MOTION_NOT, 144, 74, 64, 64, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_MOTION,     144, 74, 64, 64, zclApp_Config.HvacUiDisplayMode & 0x01);
    }
  }
}
#endif

#if defined(TFT3IN5)
static void TftIlluminance(uint8 temp_i){
  //Illuminance
  uint8 row = temp_i *120;
  char illum_string[] = {' ',' ', ' ', ' ', ' ', '\0'};
  if (old_bindClusterDev[temp_i] & 0x02){
    illum_string[0] = temp_bh1750IlluminanceSensor_MeasuredValue[temp_i] / 10000 % 10 + '0';
    illum_string[1] = temp_bh1750IlluminanceSensor_MeasuredValue[temp_i] / 1000 % 10 + '0';
    illum_string[2] = temp_bh1750IlluminanceSensor_MeasuredValue[temp_i] / 100 % 10 + '0';
    illum_string[3] = temp_bh1750IlluminanceSensor_MeasuredValue[temp_i] / 10 % 10 + '0';
    illum_string[4] = temp_bh1750IlluminanceSensor_MeasuredValue[temp_i] % 10 + '0';
  }

  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait      
    if (old_bindClusterDev[temp_i] & 0x02){
      GUI_DrawRectangle(184, 144 + row, 184+80, 144 + row + 32, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DisString_EN(184, 144 + row, illum_string, &Font32, LCD_BACKGROUND, BLUE);

      GUI_DrawRectangle(184, 176 + row, 184+36, 176 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DisString_EN(184, 176 + row, "Lux", &Font16, LCD_BACKGROUND, BLUE);
      
      if (zclApp_EpdUpDown[temp_i] & 0x02){
        GUI_Disbitmap(168, 144 + row , IMAGE_LEFT, 16, 16, BLUE, 1);
        GUI_DrawRectangle(168-1, 160 + row, 168+16, 160 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      } else {
        GUI_Disbitmap(168, 160 + row , IMAGE_RIGHT, 16, 16, BLUE, 1);
        GUI_DrawRectangle(168-1, 144 + row, 168+16, 144 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      }
    } else {
      GUI_DrawRectangle(168-1, 160 + row, 168+16, 160 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DrawRectangle(168-1, 144 + row, 168+16, 144 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DrawRectangle(184, 144 + row, 184+80, 144 + row + 32, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DrawRectangle(184, 176 + row, 184+36, 176 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
    }   
  } else { //landscape

  }

}
#endif

#if defined(EPD3IN7)
static void EpdIlluminance(uint8 temp_i){
  //Illuminance
  uint8 row = temp_i *120;
  char illum_string[] = {' ',' ', ' ', ' ', ' ', '\0'};
  if (old_bindClusterDev[temp_i] & 0x02){
    illum_string[0] = temp_bh1750IlluminanceSensor_MeasuredValue[temp_i] / 10000 % 10 + '0';
    illum_string[1] = temp_bh1750IlluminanceSensor_MeasuredValue[temp_i] / 1000 % 10 + '0';
    illum_string[2] = temp_bh1750IlluminanceSensor_MeasuredValue[temp_i] / 100 % 10 + '0';
    illum_string[3] = temp_bh1750IlluminanceSensor_MeasuredValue[temp_i] / 10 % 10 + '0';
    illum_string[4] = temp_bh1750IlluminanceSensor_MeasuredValue[temp_i] % 10 + '0';
  }

  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
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
    if (old_bindClusterDev[temp_i] & 0x02){
      PaintDrawStringAt(0, 0, "Lux", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 184, 176 + row, PaintGetWidth(), PaintGetHeight());
      if (zclApp_EpdUpDown[temp_i] & 0x02){
        EpdSetFrameMemoryImageXY(IMAGE_LEFT, 168, 144 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
        EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 168, 160 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else {
        EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 168, 160 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
        EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 168, 144 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      }
    } else {
      PaintDrawStringAt(0, 0, "   ", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 184, 176 + row, PaintGetWidth(), PaintGetHeight());
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 168, 160 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 168, 144 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
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
    EpdSetFrameMemoryImageXY(IMAGE_ILLUMINANCE, 184, 240, 48, 48, zclApp_Config.HvacUiDisplayMode & 0x01);
    if (zclApp_EpdUpDown[temp_i] & 0x02){
      EpdSetFrameMemoryImageXY(IMAGE_UP, 216, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 184, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_DOWN, 184, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 216, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    }
  }

}
#endif

#if defined(TFT3IN5)
static void TftTemperature(uint8 temp_t){
  //temperature
  uint8 row = temp_t *120;
  char temp_string[] = {' ', ' ', ' ', ' ', ' ', '\0'};
  if (old_bindClusterDev[temp_t] & 0x04){
    temp_string[0] = temp_Temperature_Sensor_MeasuredValue[temp_t] / 1000 % 10 + '0';
    temp_string[1] = temp_Temperature_Sensor_MeasuredValue[temp_t] / 100 % 10 + '0';
    temp_string[2] = '.';
    temp_string[3] = temp_Temperature_Sensor_MeasuredValue[temp_t] / 10 % 10 + '0';
    temp_string[4] = temp_Temperature_Sensor_MeasuredValue[temp_t] % 10 + '0';
  }

  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
      if (old_bindClusterDev[temp_t] & 0x04){
      GUI_DrawRectangle(80, 144 + row, 80+80, 144 + row + 32, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DisString_EN(80, 144 + row, temp_string, &Font32, LCD_BACKGROUND, BLUE);

      GUI_DrawRectangle(80, 176 + row, 80+24, 176 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DisString_EN(80, 176 + row, "^C", &Font16, LCD_BACKGROUND, BLUE);
      
      if (zclApp_EpdUpDown[temp_t] & 0x04){
        GUI_Disbitmap(64, 144 + row , IMAGE_LEFT, 16, 16, BLUE, 1);
        GUI_DrawRectangle(64-1, 160 + row, 64+16, 160 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      } else {
        GUI_Disbitmap(64, 160 + row , IMAGE_RIGHT, 16, 16, BLUE, 1);
        GUI_DrawRectangle(64-1, 144 + row, 64+16, 144 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      }
    } else {
      GUI_DrawRectangle(64-1, 160 + row, 64+16, 160 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DrawRectangle(64-1, 144 + row, 64+16, 144 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DrawRectangle(80, 144 + row, 80+80, 144 + row + 32, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DrawRectangle(80, 176 + row, 80+24, 176 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
    }
  } else { //landscape

  }

}
#endif

#if defined(EPD3IN7)
static void EpdTemperature(uint8 temp_t){
  //temperature
  uint8 row = temp_t *120;
  char temp_string[] = {' ', ' ', ' ', ' ', ' ', '\0'};
  if (old_bindClusterDev[temp_t] & 0x04){
    temp_string[0] = temp_Temperature_Sensor_MeasuredValue[temp_t] / 1000 % 10 + '0';
    temp_string[1] = temp_Temperature_Sensor_MeasuredValue[temp_t] / 100 % 10 + '0';
    temp_string[2] = '.';
    temp_string[3] = temp_Temperature_Sensor_MeasuredValue[temp_t] / 10 % 10 + '0';
    temp_string[4] = temp_Temperature_Sensor_MeasuredValue[temp_t] % 10 + '0';
  }

  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
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
    if (old_bindClusterDev[temp_t] & 0x04){
      PaintDrawStringAt(0, 0, "^C", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 80, 176 + row, PaintGetWidth(), PaintGetHeight());
      if (zclApp_EpdUpDown[temp_t] & 0x04){
        EpdSetFrameMemoryImageXY(IMAGE_LEFT, 64, 144 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
        EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 64, 160 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else {
        EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 64, 160 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
        EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 64, 144 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      }
    } else {
      PaintDrawStringAt(0, 0, "  ", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 80, 176 + row, PaintGetWidth(), PaintGetHeight());
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 64, 160 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 64, 144 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
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
    EpdSetFrameMemoryImageXY(IMAGE_TEMPERATURE, 128, 240, 48, 48, zclApp_Config.HvacUiDisplayMode & 0x01);
    if (zclApp_EpdUpDown[temp_t] & 0x04){
      EpdSetFrameMemoryImageXY(IMAGE_UP, 160, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 128, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_DOWN, 128, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 160, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    }
  }

}
#endif

#if defined(TFT3IN5)
static void TftHumidity(uint8 temp_h){
  //humidity
  uint8 row = temp_h * 120;
  char hum_string[] = {' ', ' ', ' ', ' ', ' ', '\0'};
  if (old_bindClusterDev[temp_h] & 0x10){
    hum_string[0] = temp_HumiditySensor_MeasuredValue[temp_h] / 1000 % 10 + '0';
    hum_string[1] = temp_HumiditySensor_MeasuredValue[temp_h] / 100 % 10 + '0';
    hum_string[2] = '.';
    hum_string[3] = temp_HumiditySensor_MeasuredValue[temp_h] / 10 % 10 + '0';
    hum_string[4] = temp_HumiditySensor_MeasuredValue[temp_h] % 10 + '0';
  }

  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    if (old_bindClusterDev[temp_h] & 0x10){
      GUI_DrawRectangle(80, 192 + row, 80+80, 192 + row + 32, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DisString_EN(80, 192 + row, hum_string, &Font32, LCD_BACKGROUND, BLUE);

      GUI_DrawRectangle(80, 224 + row, 80+36, 224 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DisString_EN(80, 224 + row, "%Ha", &Font16, LCD_BACKGROUND, BLUE);
      
      if (zclApp_EpdUpDown[temp_h] & 0x10){
        GUI_Disbitmap(64, 192 + row , IMAGE_LEFT, 16, 16, BLUE, 1);
        GUI_DrawRectangle(64-1, 208 + row, 64+16, 208 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      } else {
        GUI_Disbitmap(64, 208 + row , IMAGE_RIGHT, 16, 16, BLUE, 1);
        GUI_DrawRectangle(64-1, 192 + row, 64+16, 192 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      }
    } else {
      GUI_DrawRectangle(64-1, 208 + row, 64+16, 208 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DrawRectangle(64-1, 192 + row, 64+16, 192 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DrawRectangle(80, 192 + row, 80+80, 192 + row + 32, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DrawRectangle(80, 224 + row, 80+36, 224 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
    }
  } else { // landscape

  }

}
#endif

#if defined(EPD3IN7)
static void EpdHumidity(uint8 temp_h){
  //humidity
  uint8 row = temp_h * 120;
  char hum_string[] = {' ', ' ', ' ', ' ', ' ', '\0'};
  if (old_bindClusterDev[temp_h] & 0x10){
    hum_string[0] = temp_HumiditySensor_MeasuredValue[temp_h] / 1000 % 10 + '0';
    hum_string[1] = temp_HumiditySensor_MeasuredValue[temp_h] / 100 % 10 + '0';
    hum_string[2] = '.';
    hum_string[3] = temp_HumiditySensor_MeasuredValue[temp_h] / 10 % 10 + '0';
    hum_string[4] = temp_HumiditySensor_MeasuredValue[temp_h] % 10 + '0';
  }

  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
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
    if (old_bindClusterDev[temp_h] & 0x10){
      PaintDrawStringAt(0, 0, "%Ha", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 80, 224 + row, PaintGetWidth(), PaintGetHeight());
      if (zclApp_EpdUpDown[temp_h] & 0x10){
        EpdSetFrameMemoryImageXY(IMAGE_LEFT, 64, 192 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
        EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 64, 208 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else {
        EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 64, 208 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
        EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 64, 192 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      } 
    } else {
      PaintDrawStringAt(0, 0, "   ", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 80, 224 + row, PaintGetWidth(), PaintGetHeight());
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 64, 208 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 64, 192 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
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
    EpdSetFrameMemoryImageXY(IMAGE_HUMIDITY, 72, 240, 48, 48, zclApp_Config.HvacUiDisplayMode & 0x01);
  if (zclApp_EpdUpDown[temp_h] & 0x10){
    EpdSetFrameMemoryImageXY(IMAGE_UP, 104, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 72, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
  } else {
    EpdSetFrameMemoryImageXY(IMAGE_DOWN, 72, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 104, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
  }
  }

}
#endif

#if defined(TFT3IN5)
static void TftPressure(uint8 temp_p){
  //pressure
  uint8 row = temp_p*120;
  char pres_string[] = {' ', ' ', ' ', ' ', ' ', ' ', '\0'};
  if (old_bindClusterDev[temp_p] & 0x08){
    pres_string[0] = temp_PressureSensor_MeasuredValue[temp_p] / 1000 % 10 + '0';
    pres_string[1] = temp_PressureSensor_MeasuredValue[temp_p] / 100 % 10 + '0';
    pres_string[2] = temp_PressureSensor_MeasuredValue[temp_p] / 10 % 10 + '0';
    pres_string[3] = temp_PressureSensor_MeasuredValue[temp_p] % 10 + '0';
    pres_string[4] = '.';
    pres_string[5] = temp_PressureSensor_ScaledValue[temp_p] % 10 + '0';
  }

  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
   if (old_bindClusterDev[temp_p] & 0x08){
      GUI_DrawRectangle(184, 192 + row, 184+96, 192 + row + 32, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DisString_EN(184, 192 + row, pres_string, &Font32, LCD_BACKGROUND, BLUE);

      GUI_DrawRectangle(184, 224 + row, 184+36, 224 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DisString_EN(184, 224 + row, "hPa", &Font16, LCD_BACKGROUND, BLUE);
      
      if (zclApp_EpdUpDown[temp_p] & 0x08){
        GUI_Disbitmap(168, 192 + row , IMAGE_LEFT, 16, 16, BLUE, 1);
        GUI_DrawRectangle(168-1, 208 + row, 168+16, 208 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      } else {
        GUI_Disbitmap(168, 208 + row , IMAGE_RIGHT, 16, 16, BLUE, 1);
        GUI_DrawRectangle(168-1, 192 + row, 168+16, 192 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      }
    } else {
      GUI_DrawRectangle(168-1, 208 + row, 168+16, 208 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DrawRectangle(168-1, 192 + row, 168+16, 192 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DrawRectangle(184, 192 + row, 184+96, 192 + row + 32, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
      GUI_DrawRectangle(184, 224 + row, 184+36, 224 + row + 16, LCD_BACKGROUND, DRAW_FULL , DOT_PIXEL_DFT );
    }    
  } else { //landscape

  }
}
#endif

#if defined(EPD3IN7)
static void EpdPressure(uint8 temp_p){
  //pressure
  uint8 row = temp_p*120;
  char pres_string[] = {' ', ' ', ' ', ' ', ' ', ' ', '\0'};
  if (old_bindClusterDev[temp_p] & 0x08){
    pres_string[0] = temp_PressureSensor_MeasuredValue[temp_p] / 1000 % 10 + '0';
    pres_string[1] = temp_PressureSensor_MeasuredValue[temp_p] / 100 % 10 + '0';
    pres_string[2] = temp_PressureSensor_MeasuredValue[temp_p] / 10 % 10 + '0';
    pres_string[3] = temp_PressureSensor_MeasuredValue[temp_p] % 10 + '0';
    pres_string[4] = '.';
    pres_string[5] = temp_PressureSensor_ScaledValue[temp_p] % 10 + '0';
  }

  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
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
    if (old_bindClusterDev[temp_p] & 0x08){
      PaintDrawStringAt(0, 0, "hPa", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 184, 224 + row, PaintGetWidth(), PaintGetHeight());
      if (zclApp_EpdUpDown[temp_p] & 0x08){
        EpdSetFrameMemoryImageXY(IMAGE_LEFT, 168, 192 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
        EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 168, 208 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else {
        EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 168, 208 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
        EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 168, 192 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      } 
    } else {
      PaintDrawStringAt(0, 0, "   ", &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 184, 224 + row, PaintGetWidth(), PaintGetHeight());
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 168, 208 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 168, 192 + row, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
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
    EpdSetFrameMemoryImageXY(IMAGE_PRESSURE, 16, 240, 48, 48, zclApp_Config.HvacUiDisplayMode & 0x01);
    if (zclApp_EpdUpDown[temp_p] & 0x08){
      EpdSetFrameMemoryImageXY(IMAGE_UP, 48, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 16, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_DOWN, 16, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 48, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    }
  }

}
#endif

#if defined(TFT3IN5)
static void TfttestRefresh(void)
{     
  TftTimeDateWeek(); // time, date, weekday
  TftStatus(0); // status network device
// enable/disable display of values
// bit 0 - 0x0001 POWER_CFG, 1 - 0x0400 ILLUMINANCE, 2 - 0x0402 TEMP, 3 - 0x0403 PRESSURE, 
//     4 - 0x0405 HUMIDITY,  5 - 0x0406 OCCUPANCY,   6 - none       , 7 - table received
  for(uint8 i = 0; i <= 2; i++ ){ 
      LREP("bindClusterDev=0x%X 0x%X\r\n", temp_bindClusterDev[i], old_bindClusterDev[i]);
      
      TftBindStatus(i);  // status bind
      TftLqi(i);         // lqi
      TftBattery(i);     // percentage battery
      TftNwk(i);         // nwk
      TftOccupancy(i);   // occupancy
      TftIlluminance(i); // illuminance
      TftTemperature(i); // temperature
      TftHumidity(i);    // humidity
      TftPressure(i);    // pressure
  }

}
#endif

#if defined(EPD3IN7)
static void EpdtestRefresh(void)
{   
  EpdReset(); //disable sleep EPD
  PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
  
  EpdTimeDateWeek(); // time, date, weekday
  EpdStatus(0); // status network device
// enable/disable display of values
// bit 0 - 0x0001 POWER_CFG, 1 - 0x0400 ILLUMINANCE, 2 - 0x0402 TEMP, 3 - 0x0403 PRESSURE, 
//     4 - 0x0405 HUMIDITY,  5 - 0x0406 OCCUPANCY,   6 - none       , 7 - table received
  for(uint8 i = 0; i <= 2; i++ ){ 
      LREP("bindClusterDev=0x%X 0x%X\r\n", temp_bindClusterDev[i], old_bindClusterDev[i]);
      
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

  EpdDisplayFramePartial();

  EpdSleep();
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
  TftRefresh();
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
      }
  }
  
  zclReportCmd_t *pInAttrReport;
  pInAttrReport = (zclReportCmd_t *)pInMsg->attrCmd; 
// enable/disable display of values
// bit 0 - 0x0001 POWER_CFG, 1 - 0x0400 ILLUMINANCE, 2 - 0x0402 TEMP, 3 - 0x0403 PRESSURE, 
//     4 - 0x0405 HUMIDITY,  5 - 0x0406 OCCUPANCY,   6 - none       , 7 - table received
  for (uint8 n = 0; n < (pInAttrReport->numAttr); n++ ){
    if (pInMsg->clusterId == TEMP && pInAttrReport->attrList[n].attrID == ATTRID_MS_TEMPERATURE_MEASURED_VALUE){
      if (temp_Temperature_Sensor_MeasuredValue[i] > BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1])){
        zclApp_EpdUpDown[i] &= ~BV(2); // down
      } else {
        zclApp_EpdUpDown[i] |=  BV(2); // up
      }
      temp_Temperature_Sensor_MeasuredValue[i] = BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1]);
      temp_bindClusterDev[i] |= BV(2);
      old_bindClusterDev[i] |= BV(2);
    }
    if (pInMsg->clusterId == HUMIDITY && pInAttrReport->attrList[n].attrID == ATTRID_MS_RELATIVE_HUMIDITY_MEASURED_VALUE){
      if (temp_HumiditySensor_MeasuredValue[i] > BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1])){
        zclApp_EpdUpDown[i] &= ~BV(4); // down
      } else {
        zclApp_EpdUpDown[i] |=  BV(4); // up
      }
      temp_HumiditySensor_MeasuredValue[i] = BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1]);
      temp_bindClusterDev[i] |= BV(4);
      old_bindClusterDev[i] |= BV(4);
    }
    if (pInMsg->clusterId == PRESSURE && pInAttrReport->attrList[n].attrID == ATTRID_MS_PRESSURE_MEASUREMENT_MEASURED_VALUE){
      if (temp_PressureSensor_MeasuredValue[i] > BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1])){
        zclApp_EpdUpDown[i] &= ~BV(3); // down
      } else {
        zclApp_EpdUpDown[i] |=  BV(3); // up
      }
      temp_PressureSensor_MeasuredValue[i] = BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1]);
      temp_bindClusterDev[i] |= BV(3);
      old_bindClusterDev[i] |= BV(3);
    }
    if (pInMsg->clusterId == PRESSURE && pInAttrReport->attrList[n].attrID == ATTRID_MS_PRESSURE_MEASUREMENT_SCALE){
      temp_PressureSensor_ScaledValue[i] = BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1]);
      temp_bindClusterDev[i] |= BV(3);
      old_bindClusterDev[i] |= BV(3);
    }
    if (pInMsg->clusterId == ILLUMINANCE && pInAttrReport->attrList[n].attrID == ATTRID_MS_ILLUMINANCE_MEASURED_VALUE){
      if (temp_bh1750IlluminanceSensor_MeasuredValue[i] > BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1])){
        zclApp_EpdUpDown[i] &= ~BV(1); // down
      } else {
        zclApp_EpdUpDown[i] |=  BV(1); // up
      }
      temp_bh1750IlluminanceSensor_MeasuredValue[i] = BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1]);
      temp_bindClusterDev[i] |= BV(1);
      old_bindClusterDev[i] |= BV(1);
    }
    if (pInMsg->clusterId == POWER_CFG && pInAttrReport->attrList[n].attrID == ATTRID_POWER_CFG_BATTERY_PERCENTAGE_REMAINING){
      temp_Battery_PercentageRemainig[i] = BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1]);
      temp_bindClusterDev[i] |= BV(0);
      old_bindClusterDev[i] |= BV(0);      
    }
    if (pInMsg->clusterId == OCCUPANCY && pInAttrReport->attrList[n].attrID == ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY){
      temp_Occupied[i] = BUILD_UINT16(pInAttrReport->attrList[n].attrData[0], pInAttrReport->attrList[n].attrData[1]);
      temp_bindClusterDev[i] |= BV(5);
      old_bindClusterDev[i] |= BV(5);
    }
#if defined(EPD3IN7) 
    EpdRefresh();
#endif // EPD3IN7
#if defined(TFT3IN5)    
    TftRefresh();
#endif // TFT3IN5    
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

/****************************************************************************
****************************************************************************/
