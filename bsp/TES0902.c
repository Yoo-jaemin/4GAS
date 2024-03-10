/*
 * File:   TES0902.c
 * Author: dekist
 *
 * Created on 2021 6/11 
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "util/buffer.h"
#include "util/_string.h"

#include "usart.h"
#include "TES0902.h"
#include "board.h"


s_Tes0902_frame g_rDataFormat; 
uint8_t g_Wbuff[MAX_WR_LEN];
u_2byte_Conv convData;

uint8_t b[64];


void CMD_get_ppm(void)
{   //CMD: aa5514003eec
    uint8_t len = 0;
    g_Wbuff[len++] = REQUEST_SYNC_MSB;
    g_Wbuff[len++] = REQUEST_SYNC_LSB;
    g_Wbuff[len++] = CMD_READ_GAS_PPM;
    g_Wbuff[len++] = 0x00;
    convData.u16_data = Calculate_CRC16(g_Wbuff, len);
    g_Wbuff[len++] = convData.u8_Data[0];
    g_Wbuff[len++] = convData.u8_Data[1];
    
    usart_writeBytes(g_Wbuff, len);
    //sprintf(b,"convData: %x, %x, %x\r\n", convData.u16_data, g_Wbuff[4], g_Wbuff[5]);
    //write_command(b); 
}

void CMD_manual_cal(void)
{   //CMD: aa554c00052c
    uint8_t len = 0;
    g_Wbuff[len++] = REQUEST_SYNC_MSB;
    g_Wbuff[len++] = REQUEST_SYNC_LSB;
    g_Wbuff[len++] = CMD_SET_MANUAL_CAL;
    g_Wbuff[len++] = 0x00;
    convData.u16_data = Calculate_CRC16(g_Wbuff, len);
    g_Wbuff[len++] = convData.u8_Data[0];
    g_Wbuff[len++] = convData.u8_Data[1];
    
    usart_writeBytes(g_Wbuff, len);
    //sprintf(b,"convData: %x, %x, %x\r\n", convData.u16_data, g_Wbuff[4], g_Wbuff[5]);
    //write_command(b); 
}

void CMD_set_ABC_on(uint8_t value)
{   //CMD: aa5522020000bfa5 or aa55220200017e65  
    uint8_t len = 0;
    g_Wbuff[len++] = REQUEST_SYNC_MSB;
    g_Wbuff[len++] = REQUEST_SYNC_LSB;
    g_Wbuff[len++] = CMD_SET_ABC_ON;
    g_Wbuff[len++] = 0x02;
    g_Wbuff[len++] = 0x00;
    g_Wbuff[len++] = value;
    convData.u16_data = Calculate_CRC16(g_Wbuff, len);
    g_Wbuff[len++] = convData.u8_Data[0];
    g_Wbuff[len++] = convData.u8_Data[1];
    usart_writeBytes(g_Wbuff, len);
    
    //sprintf(b,"convData: %x, %x, %x\r\n", convData.u16_data, g_Wbuff[6], g_Wbuff[7]);
    //write_command(b); 
}

void CMD_get_version(void)
{   //CMD: aa5510003c2c
    uint8_t len = 0;
    g_Wbuff[len++] = REQUEST_SYNC_MSB;
    g_Wbuff[len++] = REQUEST_SYNC_LSB;
    g_Wbuff[len++] = CMD_GET_VERSION;
    g_Wbuff[len++] = 0x00;
    convData.u16_data = Calculate_CRC16(g_Wbuff, len);
    g_Wbuff[len++] = convData.u8_Data[0];
    g_Wbuff[len++] = convData.u8_Data[1];
    
    usart_writeBytes(g_Wbuff, len);
    //sprintf(b,"convData: %x, %x, %x\r\n", convData.u16_data, g_Wbuff[4], g_Wbuff[5]);
    //write_command(b); 
}

void CMD_get_serialNumber(void)
{   //CMD: aa5512003d4c
    uint8_t len = 0;
    g_Wbuff[len++] = REQUEST_SYNC_MSB;
    g_Wbuff[len++] = REQUEST_SYNC_LSB;
    g_Wbuff[len++] = CMD_GET_SERIAL_NUM;
    g_Wbuff[len++] = 0x00;
    convData.u16_data = Calculate_CRC16(g_Wbuff, len);
    g_Wbuff[len++] = convData.u8_Data[0];
    g_Wbuff[len++] = convData.u8_Data[1];
    
    usart_writeBytes(g_Wbuff, len);
    //sprintf(b,"convData: %x, %x, %x\r\n", convData.u16_data, g_Wbuff[4], g_Wbuff[5]);
    //write_command(b); 
}

uint16_t Calculate_CRC16 (uint8_t *crc, int crc_length)
{
    uint16_t ret = 0xffff, polynomial = 0xA001;
    int shift = 0x0;
    for (int i= crc_length -1; i >= 0; i--) 
    {
        uint16_t code = (uint16_t)(crc[crc_length -1 -i] & 0xff);
        ret = ret ^ code ;
        shift = 0x0;
        while (shift <= 7)      
        {
            if (ret & 0x1) {    
                ret = ret >> 1;
                ret = ret ^ polynomial;
            } else 
                ret = ret >> 1;
            shift++;
        }
    }
    return ret;
}

bool CRC_check(void)
{
    uint8_t crcLen; 
    uint8_t crc[MAX_rDATA_LEN +4];  // DATA + SYNC[2]+CMD+LEN  
    crcLen = g_rDataFormat.len +4;
    
    crc[0] = g_rDataFormat.sync[0];
    crc[1] = g_rDataFormat.sync[1];
    crc[2] = g_rDataFormat.cmd;
    crc[3] = g_rDataFormat.len;
    
    if (crcLen > 4) {
        for (uint8_t i = 0; i < g_rDataFormat.len; i++)
            crc[4+i] = g_rDataFormat.data[i];
    }
    
    convData.u16_data = Calculate_CRC16(crc, crcLen);
    
    if (g_rDataFormat.crc[0] == convData.u8_Data[0] && g_rDataFormat.crc[1] == convData.u8_Data[1])
        return true;
    return false;
}

void tes0902_polling_mode(void)
{
    CMD_get_ppm();
}

void tes0902_ABC_OFF(uint8_t status)
{
    CMD_set_ABC_on(status);
}