
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <xc.h>
#include <math.h>

#include "board.h"

#include "bsp/lmp91000.h"
#include "bsp/ads1114.h"
#include "bsp/sht3x.h"
#include "bsp/TES0902.h"

#include "nvm.h"
#include "filter.h"
#include "app_sens_measure.h"
#include "sensor.h"
#include "window_filter.h"


/* ******************* Global Variable ******************* */
extern s_Nvm g_nvm;
extern s_Sens_Measure g_sens_measure;
extern uint8_t writeBuffer[64];

#ifdef USE_KALMAN_FILTER
    extern s_KalmanFilter_t kalmanFilter[4]; 
#endif
    
s_Sens_Measure_value sens_value;
float gTemp_volt;


/* ****************** Funtion Prototype ****************** */
static bool sensor_sht3x_init(void);
static float FSR_set_LSB(uint8_t gain);
static float gasSensor_calib(uint8_t chNum, float gas_uVolt);
static float GasSensor_Zerofilter(uint8_t chNum, float gasConcent);
static float gasSensor_temperature_cmp_coefficient(uint8_t chNum, float temp);


void Sensors_initialize(void)
{
    g_sens_measure.isLMP_InitDone[0] = sensor_lmp_initialize(0);
    g_sens_measure.isLMP_InitDone[1] = sensor_lmp_initialize(1);
    sensor_lmp_initialize(2);
    g_sens_measure.isTemp_InitDone = sensor_sht3x_init();
}

static bool sensor_sht3x_init(void)
{
    if (sht3x_heaterOff() == false)
        return false;
    
    __delay_ms(50);
    
    if (sht3x_clear_status_reg() == false)
        return false;
    
    __delay_ms(50);

    //start periodic measurement, with low repeatability and 1 measurements per second
    return sht3x_measure_start();
}

bool sensor_lmp_initialize(uint8_t chNum)
{
    uint8_t lmp_gain;
    
    switch(chNum)
    {
        case GAS_CH0: lmp_gain = LMP_GAIN_CO;   break;
        case GAS_CH1: lmp_gain = LMP_GAIN_O2;   break;
        case GAS_CH2: lmp_gain = LMP_GAIN_H2S;  break;
    }
             
    if (lmp91000_isReady(chNum) == false)
       return false;

    if (lmp91000_unlock(chNum) == false)
       return false;

    if (lmp91000_set_GainRLoad(chNum, lmp_gain, LMP_RLOAD) == false)
        return false;

    // INTERNAL: reference voltage = vdd
    // EXTERNAL: reference voltage = 2.5v
    if (lmp91000_set_Reference(chNum, EXTERNAL, LMP_INTZ, NAGATIVE, B0P) == false)
        return false;
      
    if (lmp91000_set_Mode(chNum, DISABLE, AMPEROETRIC_3LEAD) == false)
        return false;
    
    //if (lmp91000_lock(chNum) == false)
    //   return false;

    return true;
}

bool sensor_adc_initialize(uint8_t chNum)
{
    uint8_t adc_gain;
    
    switch(chNum)
    {
        case GAS_CH0: adc_gain = ADC_GAIN_CO;   break;
        case GAS_CH1: adc_gain = ADC_GAIN_O2;   break;
        case GAS_CH2: adc_gain = ADC_GAIN_H2S;  break;
    }
    return ads1115_SetUp(chNum, CONTINUE_CONV, adc_gain, SPS128); 
}

bool gasSensor_read(uint8_t chNum)
{
    int16_t raw;
    uint8_t adc_gain;
    float cmp_coeff = 1.0f;
        
    if (chNum != GAS_CH2) {
        if (g_sens_measure.isLMP_InitDone[chNum] == false || g_sens_measure.isADC_InitDone[chNum] == false)
            return false;
    }
        
    switch(chNum)
    {
        case GAS_CH0: adc_gain = ADC_GAIN_CO;   break;
        case GAS_CH1: adc_gain = ADC_GAIN_O2;   break;
        case GAS_CH2: adc_gain = ADC_GAIN_H2S;  break;
    }
    
    if (sensor_ADC_read(chNum, &raw)) {
        float volt = ADCto_uVoltage(raw, adc_gain);
        if (chNum == GAS_CH1)
            volt = -volt;
        
#ifdef USE_KALMAN_FILTER
        volt = Kalman_updateEstimate(&kalmanFilter[chNum], volt);
#endif        
        /* Temperature compensation apply */
        if (g_nvm.flash.temp_cmp == TEMP_CMP_ON)  
            cmp_coeff = gasSensor_temperature_cmp_coefficient(chNum, sens_value.tempHumi[CMP_TEMP]);
            
        sens_value.toxic_gas[chNum][LV_GAS_VOLT] = volt * cmp_coeff;
        sens_value.toxic_gas[chNum][LV_GAS_CONCEN] = gasSensor_calib(chNum, sens_value.toxic_gas[chNum][LV_GAS_VOLT]);
        
        if (chNum == GAS_CH0 || chNum == GAS_CH2)
            sens_value.toxic_gas[chNum][LV_GAS_CONCEN] = floor(sens_value.toxic_gas[chNum][LV_GAS_CONCEN]);
        
        if (g_nvm.flash.filter_mode == FIL_MODE)
            sens_value.toxic_gas[chNum][LV_GAS_CONCEN] = GasSensor_Zerofilter(chNum, sens_value.toxic_gas[chNum][LV_GAS_CONCEN]);
       
        sens_value.toxic_gas[chNum][LV_GAS_CONCEN] += g_nvm.flash.gasOffset[chNum];
        return true;
    } else
        return false;
}

static float gasSensor_calib(uint8_t chNum, float gas_uVolt)
{
    float gas_ppm, gas_percent;
    float span[CHANNEL_COUNT];
    float baseline[CHANNEL_COUNT];
    float CalibrationGas[CHANNEL_COUNT];
    
    baseline[chNum] = g_nvm.flash.baseLine_uV[chNum];
    span[chNum] = g_nvm.flash.span_uV[chNum];  
    CalibrationGas[chNum] = g_nvm.flash.CalibrationGas[chNum];
    
    if (chNum == GAS_CH0) { //CO
        gas_ppm = CalibrationGas[chNum]/(span[chNum] - baseline[chNum])*(gas_uVolt - baseline[chNum]);
        return gas_ppm;
    } else if (chNum == GAS_CH1) { //O2
        gas_percent = ((20.8f - CalibrationGas[chNum])*(gas_uVolt - baseline[chNum])/(baseline[chNum] - span[chNum])+ 20.8f); 
        return gas_percent;
    } else if (chNum == GAS_CH2)  { //H2S
        gas_ppm = (CalibrationGas[chNum] /(span[chNum] - baseline[chNum])*(gas_uVolt - baseline[chNum]));
        return gas_ppm;
    }             
}

static float gasSensor_temperature_cmp_coefficient(uint8_t chNum, float temp)
{
    float coeff; 
    
    switch(chNum)
    {
        case GAS_CH0: //CO
        { 
            if (temp >= -20.0f && temp < 20.0f)
                coeff = 0.02*pow(temp,2.0f)+ -1.1*temp + 113.7;
            else if (temp >= 20.0f && temp < 30.0f)
                coeff = -1.8*temp +136;
            else if (temp >= 30.0f && temp < 40.0f)
                coeff = -0.2*temp +88;
            else if (temp >= 40.0f && temp <= 60.0f)
                coeff = -temp +120;
            else
               coeff = 100.0f; 
        } break;
        case GAS_CH1: //O2
        {
            if (temp < -20.0f || temp > 50.0f) coeff = 100.0f;
            else coeff = -0.27*temp + 105.2;
        } break;
        case GAS_CH2: //H2S
        {
            if (temp < -20.0f || temp > 20.0f) coeff = 100.0f;
            else coeff = 0.01*pow(temp,2.0f)+ -0.39*temp + 104.4;
        } break;
    }//end switch
    return coeff / 100.0f;
}

static float GasSensor_Zerofilter(uint8_t chNum, float gasConcent)
{
    float retval;
    if (chNum == GAS_CH0 || chNum == GAS_CH2) {
        if (gasConcent > -GAS_RESOLUTION && gasConcent < GAS_RESOLUTION)
            retval = 0.0f;
        else if (gasConcent > -1.0f && gasConcent <= -GAS_RESOLUTION)
            retval = 0.01f;
        else if (gasConcent > -5.0f && gasConcent <= -1.0f)
            retval = 0.02f;
        else if (gasConcent > -10.0f && gasConcent <= -5.0f)
            retval = 0.03f;
        else if (gasConcent <= -10.0f)
            retval = 0.04f;
        else
            retval = gasConcent;
    } else
        retval = gasConcent;
    return retval;
}

bool sensor_ADC_read(uint8_t chNum, int16_t* raw_adc)
{
    if (ads1114_read_ready(chNum) == false)
        return false;

    return ads1114_read(chNum, raw_adc);
}

float ADCto_uVoltage(int16_t raw_adc, uint8_t gain)
{
	float uVolt;
    float lsb = FSR_set_LSB(gain);
	uVolt = (float)raw_adc * lsb;
	return uVolt; 
}

static float FSR_set_LSB(uint8_t gain)
{
    if(gain == FSR_6p144V)
        return 187.5f;
    else if(gain == FSR_4p096V)
        return 125.0f;
    else if(gain == FSR_1p024V)
        return 31.25f;
    else if(gain == FSR_0p512V)
        return 15.62f;
    else if(gain == FSR_0p256V)
        return 7.81f;
    // default, FSR_2p048V
    return 62.5f;
}

//===================================================================

bool sensor_read_temp_humi(float* temp, float* humi)
{
#if (USE_SHT3X_COMPENSATE)
    return sht3x_measure_nonblock_compensate(temp, humi);
#else
    return sht3x_measure_nonblock_read(temp, humi);
#endif
}

void sensor_temp_fahrenheit(float* temp)
{
    *temp = (*temp * 1.8f) + 32.0f;
}

bool tempSensor_read(void)
{
    float temp, humi;
    
    if (g_sens_measure.isTemp_InitDone == false)
        return false;
        
    if (sensor_read_temp_humi(&temp, &humi)) {
        sens_value.tempHumi[LV_TEMP] = temp;
        sens_value.tempHumi[LV_TEMP] += g_nvm.flash.tempOffset;
        sens_value.tempHumi[LV_HUMI] = humi;
        sens_value.tempHumi[LV_HUMI] += g_nvm.flash.humiOffset;
        return true;
    }
    return false;
}

bool sensor_temp_mode(uint8_t chNum)
{
    if (lmp91000_set_Mode(chNum, DISABLE, TEMP_MEASURE_TIAON) == false)
        return false;
    
    if (ads1115_SetUp(chNum, CONTINUE_CONV, FSR_2p048V, SPS128) == false)
        return false;
    
    return true;
}

bool sensor_gas_mode(uint8_t chNum)
{
    uint8_t adc_gain;
    
    switch(chNum)
    {
        case GAS_CH0: adc_gain = ADC_GAIN_CO;   break;
        case GAS_CH1: adc_gain = ADC_GAIN_O2;   break;
        case GAS_CH2: adc_gain = ADC_GAIN_H2S;  break;
    }
    
    if (lmp91000_set_Mode(chNum, DISABLE, AMPEROETRIC_3LEAD) == false)
        return false;
    
    if (ads1115_SetUp(chNum, CONTINUE_CONV, adc_gain, SPS128) == false)
        return false;
}

// Refer to lmp91000 datasheet! 
bool CMP_temp_read(uint8_t chNum)
{
    int16_t raw;
    
    if (sensor_ADC_read(chNum, &raw)) {
        float volt = ADCto_uVoltage(raw, FSR_2p048V);
        gTemp_volt = volt;
        sens_value.tempHumi[CMP_TEMP] = -0.123f *(volt/1000.0f + 500.0f) + 192.1f;
        return true; 
    } else
        return false;       
}

bool gas_init(uint8_t chNum)
{
    g_sens_measure.isADC_InitDone[chNum] = sensor_adc_initialize(chNum);
    
    if (g_sens_measure.isADC_InitDone[chNum] == false)
        return false;
    
    return true;
}