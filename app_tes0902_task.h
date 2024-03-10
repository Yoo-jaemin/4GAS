 
#ifndef APP_TES0902_TASK_H
#define	APP_TES0902_TASK_H

#include "bsp/TES0902.h"


enum e_Tes0902_seq {
    SYNC_MSB_CHECK,
    SYNC_LSB_CHECK,
    CMD_CHECK,
    LEN_CHECK,
    DATA_RECV,
    DATA_PARSING
};

typedef struct {
    uint8_t seq;      
    
} s_Tes0902_Measure;


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
 
    void co2Measure_task(void);
    void Co2_measure_task(void);
    
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	

