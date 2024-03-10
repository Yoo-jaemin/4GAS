#ifndef NVM_H
#define	NVM_H


#define FLASH_ALL_PARAM_SIZE     108  

typedef union {
    __pack uint8_t byte[FLASH_ALL_PARAM_SIZE];
    __pack struct {
        uint8_t filter_mode;
        uint8_t temp_cmp;
        uint8_t temp_corf;
        uint8_t CO2_ABC;
        __pack uint8_t window_size[4];
        
        __pack float baseLine_uV[CHANNEL_COUNT];
        __pack float span_uV[CHANNEL_COUNT];
        __pack float CalibrationGas[CHANNEL_COUNT];     
        __pack float gas_slope[CHANNEL_COUNT];
        __pack float gasOffset[CHANNEL_COUNT];
        
        int32_t co2Offset;
        float tempOffset;
        float humiOffset;
        float calib_temp;
        
        // kalman filter paremeter
        float KMf_e_measure;
        
        uint32_t exectime;
        uint32_t usedtime;
        uint32_t buildtime;
        __pack uint8_t serialNum[8];        
    };
} u_Flash_Nvm;


typedef enum {
    NVM_INIT_NONE,
    NVM_INIT_FLASH,
}e_Nvm_Status;

typedef struct {
    uint8_t status;
    u_Flash_Nvm flash;
} s_Nvm;

#ifdef	__cplusplus
extern "C" {
#endif

    void nvm_init(void);
    void nvm_clear(void);
    void nvm_clear_all(void);
    
    bool nvm_write_flash_all(u_Flash_Nvm* flash);
    void nvm_read_flash_all(u_Flash_Nvm* flash);

#ifdef	__cplusplus
}
#endif

#endif

