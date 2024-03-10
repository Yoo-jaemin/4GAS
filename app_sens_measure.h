/* 
 * File:   app_sens_measure.h
 *
 */

#ifndef APP_SENS_MEASURE_H
#define	APP_SENS_MEASURE_H


enum e_Measure_Seq {
    SENS_INIT,
    SENS_READY,
    SENS_TEMP_MODE,
    SENS_TEMP_MEASURE,
    SENS_GAS_MODE,
    SENS_GAS_MODE_MEASURE,
    SENS_GAS_INIT,
    SENS_GAS_MEASURE,
    SENS_MEASURE_APPLY,
    SENS_MEASURE_CHECK,
    SENS_DISPLAY,
    SENS_WAIT
};

typedef struct {
    uint8_t seq;
    uint8_t next_seq;
    uint8_t seq_count;
    
    bool isLMP_InitDone[2];
    bool isADC_InitDone[2];    
    bool isTemp_InitDone;
    
    bool isGasSensor_Success[3];
    bool isCO2_Success;
    bool isTemp_Success;
        
} s_Sens_Measure;


#ifdef	__cplusplus
extern "C" {
#endif
    
    void sensMeasure_init();
    void sensMeasure_task();
      
    
#ifdef	__cplusplus
}
#endif

#endif	/* APP_SENS_MEASURE_H */

