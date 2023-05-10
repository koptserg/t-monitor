
#include <stdlib.h>

#include "zcl_rep.h"
#include "zcl.h"
#include "zcl_ms.h"

#include "zcl_app.h"
#include "bdb.h"
#include "bdb_interface.h"

void zclRep_Occupancy(void) {
#if BDB_REPORTING
    bdb_RepChangedAttrValue(zclApp_ThirdEP.EndPoint, OCCUPANCY, ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY);
#else
    const uint8 NUM_ATTRIBUTES = 3;
    zclReportCmd_t *pReportCmd;
    pReportCmd = osal_mem_alloc(sizeof(zclReportCmd_t) + (NUM_ATTRIBUTES * sizeof(zclReport_t)));
    if (pReportCmd != NULL) {
        pReportCmd->numAttr = NUM_ATTRIBUTES;

        pReportCmd->attrList[0].attrID = ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY;
        pReportCmd->attrList[0].dataType = ZCL_BITMAP8;
        pReportCmd->attrList[0].attrData = (void *)(&zclApp_Occupied);
        
        pReportCmd->attrList[1].attrID = ATTRID_MS_OCCUPANCY_SENSING_CONFIG_PIR_O_TO_U_DELAY;
        pReportCmd->attrList[1].dataType = ZCL_UINT16;
        pReportCmd->attrList[1].attrData = (void *)(&zclApp_Config.PirOccupiedToUnoccupiedDelay);
        
        pReportCmd->attrList[2].attrID = ATTRID_MS_OCCUPANCY_SENSING_CONFIG_PIR_U_TO_O_DELAY;
        pReportCmd->attrList[2].dataType = ZCL_UINT16;
        pReportCmd->attrList[2].attrData = (void *)(&zclApp_Config.PirUnoccupiedToOccupiedDelay);

        afAddrType_t inderect_DstAddr = {.addrMode = (afAddrMode_t)AddrNotPresent, .endPoint = 0, .addr.shortAddr = 0};
        zcl_SendReportCmd(3, &inderect_DstAddr, OCCUPANCY, pReportCmd, ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, bdb_getZCLFrameCounter());
    }
    osal_mem_free(pReportCmd);
#endif
}

void zclRep_LocalTime(void) {
#if BDB_REPORTING
    bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, GEN_TIME, ATTRID_TIME_LOCAL_TIME);
#else
    const uint8 NUM_ATTRIBUTES = 2;
    zclReportCmd_t *pReportCmd;
    pReportCmd = osal_mem_alloc(sizeof(zclReportCmd_t) + (NUM_ATTRIBUTES * sizeof(zclReport_t)));
    if (pReportCmd != NULL) {
        pReportCmd->numAttr = NUM_ATTRIBUTES;

        pReportCmd->attrList[0].attrID = ATTRID_TIME_TIME;
        pReportCmd->attrList[0].dataType = ZCL_UTC;
        pReportCmd->attrList[0].attrData = (void *)(&zclApp_GenTime_TimeUTC);
        
        pReportCmd->attrList[1].attrID = ATTRID_TIME_LOCAL_TIME;
        pReportCmd->attrList[1].dataType = ZCL_UINT32;
        pReportCmd->attrList[1].attrData = (void *)(&zclApp_GenTime_TimeUTC);

        afAddrType_t inderect_DstAddr = {.addrMode = (afAddrMode_t)AddrNotPresent, .endPoint = 0, .addr.shortAddr = 0};
        zcl_SendReportCmd(1, &inderect_DstAddr, GEN_TIME, pReportCmd, ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, bdb_getZCLFrameCounter());
    }
    osal_mem_free(pReportCmd);
#endif
}

void zclRep_ConfigDisplay(void) {
#if BDB_REPORTING
    bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, HVAC_UI_CONFIG, ATTRID_HVAC_THERMOSTAT_UI_CONFIG_DISPLAY_MODE);
#else
    const uint8 NUM_ATTRIBUTES = 1;
    zclReportCmd_t *pReportCmd;
    pReportCmd = osal_mem_alloc(sizeof(zclReportCmd_t) + (NUM_ATTRIBUTES * sizeof(zclReport_t)));
    if (pReportCmd != NULL) {
        pReportCmd->numAttr = NUM_ATTRIBUTES;

        pReportCmd->attrList[0].attrID = ATTRID_HVAC_THERMOSTAT_UI_CONFIG_DISPLAY_MODE;
        pReportCmd->attrList[0].dataType = ZCL_UINT8;
        pReportCmd->attrList[0].attrData = (void *)(&zclApp_Config.HvacUiDisplayMode);

        afAddrType_t inderect_DstAddr = {.addrMode = (afAddrMode_t)AddrNotPresent, .endPoint = 0, .addr.shortAddr = 0};
        zcl_SendReportCmd(1, &inderect_DstAddr, HVAC_UI_CONFIG, pReportCmd, ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, bdb_getZCLFrameCounter());
    }
    osal_mem_free(pReportCmd);
#endif
}

void zclRep_BME280TemperatureReport(void) {
#if BDB_REPORTING
    bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, TEMP, ATTRID_MS_TEMPERATURE_MEASURED_VALUE);
#else
    const uint8 NUM_ATTRIBUTES = 3;
    zclReportCmd_t *pReportCmd;
    pReportCmd = osal_mem_alloc(sizeof(zclReportCmd_t) + (NUM_ATTRIBUTES * sizeof(zclReport_t)));
    if (pReportCmd != NULL) {
        pReportCmd->numAttr = NUM_ATTRIBUTES;

        pReportCmd->attrList[0].attrID = ATTRID_MS_TEMPERATURE_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_UINT16;
        pReportCmd->attrList[0].attrData = (void *)(&zclApp_Temperature_Sensor_MeasuredValue);

        pReportCmd->attrList[1].attrID = ATTRID_TEMPERATURE_MIN_ABSOLUTE_CHANGE;
        pReportCmd->attrList[1].dataType = ZCL_INT16;
        pReportCmd->attrList[1].attrData = (void *)(&zclApp_Config.MsTemperatureMinAbsoluteChange);

        pReportCmd->attrList[2].attrID = ATTRID_TEMPERATURE_PERIOD;
        pReportCmd->attrList[2].dataType = ZCL_UINT16;
        pReportCmd->attrList[2].attrData = (void *)(&zclApp_Config.MsTemperaturePeriod);

        afAddrType_t inderect_DstAddr = {.addrMode = (afAddrMode_t)AddrNotPresent, .endPoint = 0, .addr.shortAddr = 0};
        zcl_SendReportCmd(1, &inderect_DstAddr, TEMP, pReportCmd, ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, bdb_getZCLFrameCounter());
    }
    osal_mem_free(pReportCmd);
#endif
}

void zclRep_BME280HumidityReport(void) {
#if BDB_REPORTING
    bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, HUMIDITY, ATTRID_MS_RELATIVE_HUMIDITY_MEASURED_VALUE);
#else
    const uint8 NUM_ATTRIBUTES = 3;
    zclReportCmd_t *pReportCmd;
    pReportCmd = osal_mem_alloc(sizeof(zclReportCmd_t) + (NUM_ATTRIBUTES * sizeof(zclReport_t)));
    if (pReportCmd != NULL) {
        pReportCmd->numAttr = NUM_ATTRIBUTES;

        pReportCmd->attrList[0].attrID = ATTRID_MS_RELATIVE_HUMIDITY_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_UINT16;
        pReportCmd->attrList[0].attrData = (void *)(&zclApp_HumiditySensor_MeasuredValue);

        pReportCmd->attrList[1].attrID = ATTRID_HUMIDITY_MIN_ABSOLUTE_CHANGE;
        pReportCmd->attrList[1].dataType = ZCL_INT16;
        pReportCmd->attrList[1].attrData = (void *)(&zclApp_Config.MsHumidityMinAbsoluteChange);

        pReportCmd->attrList[2].attrID = ATTRID_HUMIDITY_PERIOD;
        pReportCmd->attrList[2].dataType = ZCL_UINT16;
        pReportCmd->attrList[2].attrData = (void *)(&zclApp_Config.MsHumidityPeriod);

        afAddrType_t inderect_DstAddr = {.addrMode = (afAddrMode_t)AddrNotPresent, .endPoint = 0, .addr.shortAddr = 0};
        zcl_SendReportCmd(1, &inderect_DstAddr, HUMIDITY, pReportCmd, ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, bdb_getZCLFrameCounter());
    }
    osal_mem_free(pReportCmd);
#endif
}

void zclRep_BME280PressureReport(void) {
#if BDB_REPORTING
    bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, PRESSURE, ATTRID_MS_PRESSURE_MEASUREMENT_MEASURED_VALUE);
#else
    const uint8 NUM_ATTRIBUTES = 5;
    zclReportCmd_t *pReportCmd;
    pReportCmd = osal_mem_alloc(sizeof(zclReportCmd_t) + (NUM_ATTRIBUTES * sizeof(zclReport_t)));
    if (pReportCmd != NULL) {
        pReportCmd->numAttr = NUM_ATTRIBUTES;

        pReportCmd->attrList[0].attrID = ATTRID_MS_PRESSURE_MEASUREMENT_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_INT16;
        pReportCmd->attrList[0].attrData = (void *)(&zclApp_PressureSensor_MeasuredValue);

        pReportCmd->attrList[1].attrID = ATTRID_MS_PRESSURE_MEASUREMENT_SCALED_VALUE;
        pReportCmd->attrList[1].dataType = ZCL_INT16;
        pReportCmd->attrList[1].attrData = (void *)(&zclApp_PressureSensor_ScaledValue);

        pReportCmd->attrList[2].attrID = ATTRID_MS_PRESSURE_MEASUREMENT_SCALE;
        pReportCmd->attrList[2].dataType = ZCL_INT8;
        pReportCmd->attrList[2].attrData = (void *)(&zclApp_PressureSensor_Scale);
        
        pReportCmd->attrList[3].attrID = ATTRID_PRESSURE_MIN_ABSOLUTE_CHANGE;
        pReportCmd->attrList[3].dataType = ZCL_UINT16;
        pReportCmd->attrList[3].attrData = (void *)(&zclApp_Config.MsPressureMinAbsoluteChange);
        
        pReportCmd->attrList[4].attrID = ATTRID_PRESSURE_PERIOD;
        pReportCmd->attrList[4].dataType = ZCL_UINT16;
        pReportCmd->attrList[4].attrData = (void *)(&zclApp_Config.MsPressurePeriod);

        afAddrType_t inderect_DstAddr = {.addrMode = (afAddrMode_t)AddrNotPresent, .endPoint = 0, .addr.shortAddr = 0};
        zcl_SendReportCmd(1, &inderect_DstAddr, PRESSURE, pReportCmd, ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, bdb_getZCLFrameCounter());
    }
    osal_mem_free(pReportCmd);
#endif
}

void zclRep_CO2Report(void) {
#if BDB_REPORTING
    bdb_RepChangedAttrValue(zclApp_FifthEP.EndPoint, ZCL_CO2, ATTRID_CO2_MEASURED_VALUE);
#else
    const uint8 NUM_ATTRIBUTES = 4;
    zclReportCmd_t *pReportCmd;
    pReportCmd = osal_mem_alloc(sizeof(zclReportCmd_t) + (NUM_ATTRIBUTES * sizeof(zclReport_t)));
    if (pReportCmd != NULL) {
        pReportCmd->numAttr = NUM_ATTRIBUTES;

        pReportCmd->attrList[0].attrID = ATTRID_CO2_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_SINGLE;
        pReportCmd->attrList[0].attrData = (void *)(&zclApp_scd4xCO2Sensor_MeasuredValue);

        pReportCmd->attrList[1].attrID = ATTRID_CO2_FORCED_RECALIBRATION;
        pReportCmd->attrList[1].dataType = ZCL_BOOLEAN;
        pReportCmd->attrList[1].attrData = (void *)(&zclApp_scd4xCO2Sensor_ForcedRecalibration);

        pReportCmd->attrList[2].attrID = ATTRID_CO2_MIN_ABSOLUTE_CHANGE;
        pReportCmd->attrList[2].dataType = ZCL_UINT16;
        pReportCmd->attrList[2].attrData = (void *)(&zclApp_Config.CO2MinAbsoluteChange);
        
        pReportCmd->attrList[3].attrID = ATTRID_CO2_PERIOD;
        pReportCmd->attrList[3].dataType = ZCL_UINT16;
        pReportCmd->attrList[3].attrData = (void *)(&zclApp_Config.CO2Period);

        afAddrType_t inderect_DstAddr = {.addrMode = (afAddrMode_t)AddrNotPresent, .endPoint = 0, .addr.shortAddr = 0};
        zcl_SendReportCmd(5, &inderect_DstAddr, ZCL_CO2, pReportCmd, ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, bdb_getZCLFrameCounter());
    }
    osal_mem_free(pReportCmd);
#endif
}

void zclRep_IlluminanceReport(void) {
#if BDB_REPORTING
    bdb_RepChangedAttrValue(zclApp_FourthEP.EndPoint, ILLUMINANCE, ATTRID_MS_ILLUMINANCE_MEASURED_VALUE);
#else
    const uint8 NUM_ATTRIBUTES = 4;
    zclReportCmd_t *pReportCmd;
    pReportCmd = osal_mem_alloc(sizeof(zclReportCmd_t) + (NUM_ATTRIBUTES * sizeof(zclReport_t)));
    if (pReportCmd != NULL) {
        pReportCmd->numAttr = NUM_ATTRIBUTES;

        pReportCmd->attrList[0].attrID = ATTRID_MS_ILLUMINANCE_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_UINT16;
        pReportCmd->attrList[0].attrData = (void *)(&zclApp_bh1750IlluminanceSensor_MeasuredValue);

        pReportCmd->attrList[1].attrID = ATTRID_ILLUMINANCE_LEVEL_SENSING_SENSITIVITY;
        pReportCmd->attrList[1].dataType = ZCL_UINT16;
        pReportCmd->attrList[1].attrData = (void *)(&zclApp_Config.MsIlluminanceLevelSensingSensitivity);

        pReportCmd->attrList[2].attrID = ATTRID_ILLUMINANCE_MIN_ABSOLUTE_CHANGE;
        pReportCmd->attrList[2].dataType = ZCL_UINT16;
        pReportCmd->attrList[2].attrData = (void *)(&zclApp_Config.MsIlluminanceMinAbsoluteChange);
        
        pReportCmd->attrList[3].attrID = ATTRID_ILLUMINANCE_PERIOD;
        pReportCmd->attrList[3].dataType = ZCL_UINT16;
        pReportCmd->attrList[3].attrData = (void *)(&zclApp_Config.MsIlluminancePeriod);

        afAddrType_t inderect_DstAddr = {.addrMode = (afAddrMode_t)AddrNotPresent, .endPoint = 0, .addr.shortAddr = 0};
        zcl_SendReportCmd(4, &inderect_DstAddr, ILLUMINANCE, pReportCmd, ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, bdb_getZCLFrameCounter());
    }
    osal_mem_free(pReportCmd);
#endif
}

void zclRep_BatteryReport(void) {
#if BDB_REPORTING
    bdb_RepChangedAttrValue(1, POWER_CFG, ATTRID_POWER_CFG_BATTERY_PERCENTAGE_REMAINING);
#else
    const uint8 NUM_ATTRIBUTES = 3;
    zclReportCmd_t *pReportCmd;
    pReportCmd = osal_mem_alloc(sizeof(zclReportCmd_t) + (NUM_ATTRIBUTES * sizeof(zclReport_t)));
    if (pReportCmd != NULL) {
        pReportCmd->numAttr = NUM_ATTRIBUTES;

        pReportCmd->attrList[0].attrID = ATTRID_POWER_CFG_BATTERY_VOLTAGE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT8;
        pReportCmd->attrList[0].attrData = (void *)(&zclApp_BatteryVoltage);

        pReportCmd->attrList[1].attrID = ATTRID_POWER_CFG_BATTERY_PERCENTAGE_REMAINING;
        pReportCmd->attrList[1].dataType = ZCL_DATATYPE_UINT8;
        pReportCmd->attrList[1].attrData = (void *)(&zclApp_BatteryPercentageRemainig);

        pReportCmd->attrList[2].attrID = ATTRID_POWER_CFG_BATTERY_PERIOD;
        pReportCmd->attrList[2].dataType = ZCL_DATATYPE_UINT16;
        pReportCmd->attrList[2].attrData = (void *)(&zclApp_Config.CfgBatteryPeriod);

        afAddrType_t inderect_DstAddr = {.addrMode = (afAddrMode_t)AddrNotPresent, .endPoint = 0, .addr.shortAddr = 0};
        zcl_SendReportCmd(1, &inderect_DstAddr, POWER_CFG, pReportCmd, ZCL_FRAME_CLIENT_SERVER_DIR, TRUE, bdb_getZCLFrameCounter());
    }
    osal_mem_free(pReportCmd);
#endif
}