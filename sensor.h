/* 
 * File:   sensor.h
 *
 */

#ifndef SENSOR_H
#define	SENSOR_H

#include "board.h"

#define SENS_COUNT LV_GAS_CONCEN+1

enum e_GasSensValue {
    LV_GAS_VOLT,
    LV_GAS_CONCEN
};

enum e_SensValue {
    LV_TEMP,
    LV_HUMI,
    CMP_TEMP
};

typedef struct {
    bool temp;
    bool gas_lmp[3];
} s_Sensor_init;

typedef struct {
   __pack float toxic_gas[CHANNEL_COUNT][SENS_COUNT];
    uint16_t co2_gas;
    float tempHumi[CMP_TEMP+1];  
} s_Sens_Measure_value;


#ifdef	__cplusplus
extern "C" {
#endif
    
    void Sensors_initialize(void);
    bool sensor_lmp_initialize(uint8_t chNum); 
    bool sensor_adc_initialize(uint8_t chNum); 
     
    bool sensor_ADC_read(uint8_t chNum, int16_t* raw_adc); 
    float ADCto_uVoltage(int16_t raw_adc, uint8_t gain);   
    bool gasSensor_read(uint8_t chNum);
    
    bool sensor_read_temp_humi(float* temp, float* humi);  
    void sensor_temp_fahrenheit(float* temp);  
    bool tempSensor_read(void);

    bool sensor_temp_mode(uint8_t chNum);
    bool sensor_gas_mode(uint8_t chNum);
    bool CMP_temp_read(uint8_t chNum);
    bool gas_init(uint8_t chNum);
    
#ifdef	__cplusplus
}
#endif

#endif	/* SENSOR_H */

