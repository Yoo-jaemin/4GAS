/*
 * File:   app_tes0902_task.c
 * Author: brian
 *
 * Created on 2021 6/15
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "util/buffer.h"
#include "util/_string.h"

#include "bsp/TES0902.h"
#include "board.h"
#include "nvm.h"
#include "filter.h"

#include "sensor.h"
#include "app_tes0902_task.h"
#include "app_sens_measure.h"



/* ******************* Global Variable ******************* */
#ifdef USE_KALMAN_FILTER
    extern s_KalmanFilter_t kalmanFilter[4]; 
#endif

extern s_Nvm g_nvm;
extern s_Queue g_usart_queue;

extern s_Tes0902_frame g_rDataFormat;
extern s_Tes0902_Measure g_tes0902_measure;
extern s_Sens_Measure g_sens_measure;
extern s_Sens_Measure_value sens_value;


/* ****************** Funtion Prototype ****************** */
static bool tes0902_data_receive(void);
static bool tes0902_data_parsing(uint8_t cmd);



void Co2_measure_task(void)
{
    uint8_t rev_data;
    switch (g_tes0902_measure.seq)
    {
        case SYNC_MSB_CHECK:
            if (buffer_de_queue_bool(&g_usart_queue, &rev_data) == false)
                break;
            if (rev_data == RESPONSE_SYNC_MSB) {
                //write_command("SYNC_MSB_OK\r\n");
                g_rDataFormat.sync[0] = RESPONSE_SYNC_MSB;
                g_tes0902_measure.seq = SYNC_LSB_CHECK;
            } else  g_tes0902_measure.seq = SYNC_MSB_CHECK;
                break;
        case SYNC_LSB_CHECK:
            if (buffer_de_queue_bool(&g_usart_queue, &rev_data) == false)
                break;
            if (rev_data == RESPONSE_SYNC_LSB) {
                //write_command("SYNC_LSB_OK\r\n");
                g_rDataFormat.sync[1] = RESPONSE_SYNC_LSB;
                g_tes0902_measure.seq = CMD_CHECK;
            } else  g_tes0902_measure.seq = SYNC_MSB_CHECK;
                break;
        case CMD_CHECK:
            if (buffer_de_queue_bool(&g_usart_queue, &rev_data) == false)
                break;
            if (rev_data == RESPONSE_READ_PPM || rev_data == RESPONSE_MANUAL_CAL || rev_data == RESPONSE_ABC_ON) {
                //write_command("CMD_OK\r\n");
                g_rDataFormat.cmd = rev_data;
                g_tes0902_measure.seq = LEN_CHECK;
            } else g_tes0902_measure.seq = SYNC_MSB_CHECK;  
                break;
        case LEN_CHECK:
            if (buffer_de_queue_bool(&g_usart_queue, &rev_data) == false)
                break;
            g_rDataFormat.len = rev_data;
            g_tes0902_measure.seq = DATA_RECV;
            break;
        case DATA_RECV:
            if (tes0902_data_receive())
                g_tes0902_measure.seq = DATA_PARSING;
            else    g_tes0902_measure.seq = SYNC_MSB_CHECK;    
            break;
        case DATA_PARSING:
            g_sens_measure.isCO2_Success = tes0902_data_parsing(g_rDataFormat.cmd);
            g_tes0902_measure.seq = SYNC_MSB_CHECK;  
            break;
    }// end switch
}// end function

static bool tes0902_data_receive(void)
{   
    uint8_t data_len = 0;
    uint8_t crc_len = 0;
    uint8_t rev_data[MAX_rDATA_LEN];
    uint8_t rev_crc[CRC_LEN];

    while (data_len != g_rDataFormat.len)
    {
        if (buffer_de_queue_bool(&g_usart_queue, &rev_data[data_len])) {
            g_rDataFormat.data[data_len] = rev_data[data_len];
            data_len++;
        }    
    }

    while (crc_len != CRC_LEN)
    {
        if (buffer_de_queue_bool(&g_usart_queue, &rev_crc[crc_len])) {
            g_rDataFormat.crc[crc_len] = rev_crc[crc_len];
            crc_len++;
        }    
    }
    return true;
}

static bool tes0902_data_parsing(uint8_t cmd)
{
    if (CRC_check() == false)   return false;
    
    switch (cmd)
    {
        case RESPONSE_READ_PPM:
            sens_value.co2_gas = g_rDataFormat.data[1] *256 + g_rDataFormat.data[0]; 
#ifndef CERTIFICATION_TEST        
            if (sens_value.co2_gas <= 400)  
                sens_value.co2_gas = 400;
#endif 
            
#ifdef USE_KALMAN_FILTER
            sens_value.co2_gas = Kalman_updateEstimate(&kalmanFilter[3], sens_value.co2_gas);    
#endif            
            sens_value.co2_gas += g_nvm.flash.co2Offset; 
            break;
            
        case RESPONSE_MANUAL_CAL:
        case RESPONSE_ABC_ON:
            break;
            
        default: break;                
    }// end switch
    return true;
}// end function

