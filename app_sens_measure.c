
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "board.h"

#include "bsp/ads1114.h"
#include "bsp/lmp91000.h"
#include "bsp/sht3x.h"
#include "bsp/TES0902.h"

#include "nvm.h"
#include "filter.h"
#include "sensor.h"
#include "app_sens_measure.h"
#include "app_tes0902_task.h"


/* ******************* Global Variable ******************* */
extern s_Sens_Measure_value sens_value;
extern uint8_t writeBuffer[64];
extern s_Nvm g_nvm;
extern s_Sens_Measure g_sens_measure;
extern s_Tes0902_Measure g_tes0902_measure;
extern float gTemp_volt;


#ifdef USE_KALMAN_FILTER
    extern s_KalmanFilter_t kalmanFilter[4]; 
#endif
  
volatile uint32_t g_large_counter;

s_Sens_Measure_value Last_value;
uint8_t gChNum;


void sensMeasure_init(void)
{
    g_large_counter = 0;
    g_sens_measure.seq = SENS_INIT;
    g_tes0902_measure.seq = SYNC_MSB_CHECK;
            
    tes0902_ABC_OFF(g_nvm.flash.CO2_ABC);
    g_sens_measure.isLMP_InitDone[0] = false;
    g_sens_measure.isLMP_InitDone[1] = false;
    g_sens_measure.isADC_InitDone[0] = false;
    g_sens_measure.isADC_InitDone[1] = false;
    g_sens_measure.isTemp_InitDone = false;
    
    g_sens_measure.isGasSensor_Success[0] = false;
    g_sens_measure.isGasSensor_Success[1] = false;
    g_sens_measure.isGasSensor_Success[2] = false;
    g_sens_measure.isCO2_Success = false;
#ifndef CERTIFICATION_TEST
    Last_value.co2_gas = 400;
#endif
    
#ifdef USE_KALMAN_FILTER
    for (uint8_t chNum = 0; chNum < 4; chNum++) 
    {
        kalmanFilter[chNum]._current_estimate = 0.0f;
        kalmanFilter[chNum]._err_estimate = 0.0f;
        kalmanFilter[chNum]._err_measure = 0.0f;
        kalmanFilter[chNum]._kalman_gain = 0.0f;
        kalmanFilter[chNum]._last_estimate = 0.0f;
        kalmanFilter[chNum]._q_process = 0.0f;
        Kalman_setMeasurementError(&kalmanFilter[chNum],1);
        Kalman_setEstimateError(&kalmanFilter[chNum],1);
        Set_KalmanFilter_Sensitivity(&kalmanFilter[chNum], g_nvm.flash.window_size[chNum]);
    }
    // ch2: o2 kmf parameter
    Kalman_setMeasurementError(&kalmanFilter[1], g_nvm.flash.KMf_e_measure);    
#endif
}

void sensMeasure_task(void)
{
    switch (g_sens_measure.seq) 
    {   
        case SENS_INIT: 
            Sensors_initialize();  
            g_large_counter = 0;
            g_sens_measure.seq = SENS_READY;
            break;
        case SENS_READY:
            if (g_large_counter > 20) {     // 200ms             
                g_large_counter = 0;
                g_sens_measure.seq = SENS_TEMP_MODE;  
            } break;
        case SENS_TEMP_MODE:
            sensor_temp_mode(GAS_CH2);
            g_large_counter = 0;
            g_sens_measure.seq = SENS_WAIT;
            g_sens_measure.next_seq =  SENS_TEMP_MEASURE;
            g_sens_measure.seq_count = 20;  // 200ms
            break;
        case SENS_TEMP_MEASURE:
            g_sens_measure.isTemp_Success = tempSensor_read();
            CMP_temp_read(GAS_CH2);
            g_large_counter = 0;
            g_sens_measure.seq = SENS_WAIT;
            g_sens_measure.next_seq =  SENS_GAS_MODE; 
            g_sens_measure.seq_count = 10; 
            break;
        case SENS_GAS_MODE:
            sensor_gas_mode(GAS_CH2);
            g_large_counter = 0;
            g_sens_measure.seq = SENS_WAIT;
            g_sens_measure.next_seq =  SENS_GAS_MODE_MEASURE; 
            g_sens_measure.seq_count = 20;
            break;
        case SENS_GAS_MODE_MEASURE:
            g_sens_measure.isGasSensor_Success[GAS_CH2] = gasSensor_read(GAS_CH2);
            g_sens_measure.seq = SENS_GAS_INIT;
            break;
        case SENS_GAS_INIT:
            gas_init(gChNum);
            g_large_counter = 0;
            g_sens_measure.seq = SENS_WAIT;
            g_sens_measure.next_seq = SENS_GAS_MEASURE;
            g_sens_measure.seq_count = 20;
            break;            
        case SENS_GAS_MEASURE:
            g_sens_measure.isGasSensor_Success[gChNum] = gasSensor_read(gChNum);
            gChNum++;
            g_large_counter = 0;
            g_sens_measure.seq = SENS_WAIT;
            g_sens_measure.next_seq = SENS_GAS_INIT;
            g_sens_measure.seq_count = 10;   
            break;
        case SENS_MEASURE_CHECK:
            if (g_sens_measure.isGasSensor_Success[0] && g_sens_measure.isGasSensor_Success[1] && g_sens_measure.isGasSensor_Success[2]) 
                g_sens_measure.seq = SENS_MEASURE_APPLY; 
            else    g_sens_measure.seq = SENS_INIT;     
            break;    
        case SENS_MEASURE_APPLY:
            Last_value.toxic_gas[0][LV_GAS_VOLT] = sens_value.toxic_gas[0][LV_GAS_VOLT];
            Last_value.toxic_gas[1][LV_GAS_VOLT] = sens_value.toxic_gas[1][LV_GAS_VOLT];
            Last_value.toxic_gas[2][LV_GAS_VOLT] = sens_value.toxic_gas[2][LV_GAS_VOLT];
            Last_value.toxic_gas[0][LV_GAS_CONCEN] = sens_value.toxic_gas[0][LV_GAS_CONCEN];
            Last_value.toxic_gas[1][LV_GAS_CONCEN] = sens_value.toxic_gas[1][LV_GAS_CONCEN];
            Last_value.toxic_gas[2][LV_GAS_CONCEN] = sens_value.toxic_gas[2][LV_GAS_CONCEN];            
            if (g_sens_measure.isCO2_Success) 
                Last_value.co2_gas = sens_value.co2_gas;
            Last_value.tempHumi[LV_TEMP] = sens_value.tempHumi[LV_TEMP];
            Last_value.tempHumi[LV_HUMI] = sens_value.tempHumi[LV_HUMI];
            Last_value.tempHumi[CMP_TEMP] = sens_value.tempHumi[CMP_TEMP];           
            g_sens_measure.seq = SENS_DISPLAY;
            break;
        case SENS_DISPLAY:
            //sprintf(writeBuffer,"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", Last_value.toxic_gas[0][LV_GAS_VOLT],Last_value.toxic_gas[1][LV_GAS_VOLT],
            //                   Last_value.toxic_gas[2][LV_GAS_VOLT], Last_value.tempHumi[LV_TEMP],sens_value.tempHumi[CMP_TEMP], gTemp_volt);
            //write_command(writeBuffer);            
            g_large_counter = 0;
            g_sens_measure.seq = SENS_READY;   
            break;   
        case SENS_WAIT:
            if (gChNum == 2) {
                gChNum = 0;
                g_sens_measure.seq = SENS_MEASURE_CHECK;
            } else {
                if (g_large_counter > g_sens_measure.seq_count) {
                    g_large_counter = 0;
                    g_sens_measure.seq = g_sens_measure.next_seq;
                }
            } break;           
    }//end switch
}//end function