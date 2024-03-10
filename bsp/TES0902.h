#ifndef TES0902_H
#define	TES0902_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_rDATA_LEN   8
#define MAX_WR_LEN      8
#define CRC_LEN         2

#define REQUEST_SYNC_MSB        0xAA
#define REQUEST_SYNC_LSB        0x55

#define RESPONSE_SYNC_MSB       0xBB
#define RESPONSE_SYNC_LSB       0x66

#define RESPONSE_READ_PPM       0x15
#define RESPONSE_MANUAL_CAL     0x4D
#define RESPONSE_FW_VER         0x11
#define RESPONSE_ABC_ON         0x23

#define CMD_READ_GAS_PPM        0x14
#define CMD_GET_VERSION         0x10
#define CMD_GET_SERIAL_NUM      0x12
#define CMD_GET_ALARM           0x16
#define CMD_SET_ALARM           0x18
#define CMD_SET_MANUAL_CAL      0x4C
#define CMD_SET_ABC_ON          0x22


typedef struct {
     __pack uint8_t sync[2];
    uint8_t cmd;
    uint8_t len;
    __pack uint8_t data[MAX_rDATA_LEN];
    __pack uint8_t crc[2]; 
} s_Tes0902_frame;


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    void CMD_get_ppm(void);
    void CMD_manual_cal(void);
    void CMD_set_ABC_on(uint8_t value);
    
    void CMD_get_version(void);
    void CMD_get_serialNumber(void);
    
    uint16_t Calculate_CRC16(uint8_t *crc, int crc_length);  
    bool CRC_check(void);
    
    void tes0902_polling_mode(void);
    void tes0902_ABC_OFF(uint8_t status);
    
#ifdef	__cplusplus
}
#endif 

#endif	

