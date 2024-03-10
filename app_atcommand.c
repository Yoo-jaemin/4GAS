
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "usb_config.h"
#include "usb/usb_device_cdc.h"

#include "board.h"

#include "util/buffer.h"
#include "util/_string.h"
#include "util/_ftoa.h"
#include "util/_atof.h"
#include "util/_itoa.h"
#include "util/_atoi.h"

#include "bsp/TES0902.h"
#include "bsp/flash.h"

#include "system.h"
#include "nvm.h"
#include "filter.h"
#include "sensor.h"
#include "app_led_task.h"
#include "app_sens_measure.h"
#include "app_atcommand.h"
#include "app_tes0902_task.h"


typedef enum {
	PARSE_READY,
	PARSE_START,
	PARSE_PARAM,
	PARSE_COMPL
} e_Parsing_Seq;

typedef enum {
    AT_PARSE_NONE,
    AT_PARSE_OK,
    AT_PARSE_ERROR
} e_ParseState;

typedef struct {
	e_AtCommand atcommand;
	char szcommand[11];
    e_CommandType type;
} s_Command_type;

const s_Command_type atcommand_type[] = {
	{ ATCZ, "ATCZ", AT_READONLY },
	{ ATCD, "ATCD", AT_READONLY }, 
	{ ATCQ, "ATCQ", AT_READONLY }, 
    { ATCH, "ATCH", AT_READONLY }, 
    { ATCUV, "ATCUV", AT_READONLY }, 
    { ATCVER, "ATCVER", AT_READONLY },
    { ATCRST, "ATCRST", AT_APPLYONLY }, // h/w reset
#ifdef SUPPORT_BOOLTLOADER
    { ATCBLRST, "ATCBLRST", AT_APPLYONLY }, // bootloader reset
#endif

	{ ATCMODEL, "ATCMODEL", AT_READONLY },
	{ ATCSMODEL, "ATCSMODEL", AT_APPLYONLY },
    { ATCF, "ATCF", AT_APPLYONLY },
	{ ATCC, "ATCC", AT_APPLYONLY },
    
    { ATCCAL, "ATCCAL%", AT_APPLYONLY }, // zero uV, span uV, cal ppm
    { ATCZCAL, "ATCZCAL", AT_APPLYONLY},
    { ATCCZRA, "ATCCZRA", AT_APPLYONLY },
    { ATCCZR, "ATCCZR%", AT_APPLYONLY },
    { ATCCSP, "ATCCSP%", AT_READAPPLY }, 
    
    { ATCOFF, "ATCOFF%", AT_READAPPLY },
    { ATCMODE, "ATCMODE", AT_READAPPLY }, //BL filter on/off
    { ATCTCMP, "ATCTCMP", AT_READAPPLY}, 
    { ATCWIN, "ATCWIN%", AT_READAPPLY },
    { ATCKMFEM, "ATCKMFEM", AT_READAPPLY },
    { ATCCO2ABC, "ATCCO2ABC", AT_READAPPLY },
    
    //execute datetime
	{ ATCEXTM, "ATCEXTM", AT_READAPPLY },
    { ATCUSED, "ATCUSED", AT_READAPPLY },
    { ATCBLDTM, "ATCBLDTM", AT_READONLY }
};

/* ******************* Global Variable ******************* */
extern s_Sens_Measure_value Last_value;
extern s_Tes0902_Measure g_tes0902_measure;
extern s_Tes0902_frame g_rDataFormat; 

extern s_Queue g_usb_queue;
extern s_Nvm g_nvm;
extern s_AtCommand g_atcommand;
extern s_Led_Status g_led_status;
extern s_Sens_Measure g_sens_measure;
extern uint8_t g_usb_state;

extern uint8_t g_Wbuff[MAX_WR_LEN];
extern u_2byte_Conv convData;

#ifdef USE_KALMAN_FILTER
    extern s_KalmanFilter_t kalmanFilter[4]; 
#endif

uint8_t writeBuffer[CDC_DATA_IN_EP_SIZE];


/* ****************** Funtion Prototype ****************** */
static e_ParseState parse_command(const uint8_t *buff, s_AtCommand_status *status);
static void update_command(s_AtCommand_status* status);
static void build_response(const s_AtCommand_status* status);
static void clear_command();


void atcommand_init(void)
{
    g_atcommand.seq = AT_INIT;
    clear_command();
}

void write_command(const uint8_t *buff)
{
    if(USBUSARTIsTxTrfReady() == true) 
    {
        putUSBUSART((uint8_t*)buff, (uint8_t)_strlen(buff));
    }
}

void atcommand_task(void)
{
	uint8_t ch;
    if (g_usb_state != USB_ATTACHED) return;

	switch (g_atcommand.seq) {
        case AT_INIT:
            g_atcommand.seq = AT_START;
            break;
        case AT_START:
            ch = buffer_de_queue(&g_usb_queue);
            if (ch) {
                if (ch == '\r' || ch == '\n') {
                    if (g_atcommand.readidx > 2) {
                        g_atcommand.read_cmd[g_atcommand.readidx++] = ch;
                        g_atcommand.readidx = 0;
                        g_atcommand.seq = AT_PARSE;
                        if (g_led_status.isBlink == false)
                            g_led_status.seq = LED_COMMAND;
                    } else {
                        g_atcommand.readidx = 0;
                        _strclr(g_atcommand.read_cmd, sizeof(g_atcommand.read_cmd));
                    }
                } else {
                    if (g_atcommand.readidx >= sizeof(g_atcommand.read_cmd)) g_atcommand.readidx = 0;
                        g_atcommand.read_cmd[g_atcommand.readidx++] = ch;
                }
            } break;  
        case AT_PARSE:
            if (parse_command(g_atcommand.read_cmd, &g_atcommand.status) == AT_PARSE_OK)
                g_atcommand.seq = (g_atcommand.status.isApply) ? AT_APPLY : AT_MESSAGE;
            else g_atcommand.seq = AT_END;
            break;
        case AT_APPLY:
            update_command(&g_atcommand.status);
            g_atcommand.seq = AT_MESSAGE;
            break;
        case AT_MESSAGE:
            build_response(&g_atcommand.status);
            g_atcommand.seq = AT_END;
            break;
        case AT_END:
            if (g_led_status.isBlink == false)
                g_led_status.seq = LED_WAIT_OFF;
            clear_command();
            g_atcommand.seq = AT_START;
            break;
	}
}

static bool check_param(char param)
{
    if (param == '-' || param == '.' || param == ',' ||
       (param >= '0' && param <= '9'))
        return true;
    return false;
}

static unsigned param_copy(char *src1, const char *src2)
{
    unsigned int i = 0;
    char ch = 0;
	if(!src1 || !src2) return 0;
	do {
        ch = *src2++;
        if (check_param(ch)) {
            *src1++ = ch;
            i++;
        }
    } while(ch);
    return i;
}

static bool atcommand_compare(const char *src1, const char *src2, uint8_t n, uint8_t *channelno)
{
    if(!src1 || !src2 || n<=0) return false;
    while (n--) 
    {
        char ch = *src1;
        if (ch == '%') 
        {
            uint8_t no = *src2 - '0';
            if (no > 0 && no <= 6) {  //CHANNEL_COUNT
                *channelno = no;
                ch = *src2;
            } else
                return false;
        }
        if (!(ch == *src2 || ch == (*src2 - 0x20))) // lower case check
//        if (ch != *src2)
            return false;
        src1++;
        src2++;
    }
    return true;
}

static int8_t check_atcommand(const char *cmd, uint8_t *chno)
{
    for (uint8_t i = 0 ; i < sizeof(atcommand_type)/sizeof(s_Command_type) ; i++) 
    {
        if (_strlen(cmd) == _strlen(atcommand_type[i].szcommand)) 
        {
            if (atcommand_compare(atcommand_type[i].szcommand, cmd, _strlen(cmd), chno)) 
                return i;
            
        }
    }
    return -1;
}

static e_ParseState parse_command(const uint8_t *buff, s_AtCommand_status *status)
{
	e_Parsing_Seq seq = PARSE_READY;

	char cmd[11] = {0};
	char param[32] = {0};
	uint8_t cmdidx = 0, paramidx = 0;

	e_ParseState retval = AT_PARSE_NONE;

    char ch = 0;
	do
	{
		ch = *buff;
		switch (seq) 
        {
            case PARSE_READY:
                if (ch == 'A' || ch == 'T' || ch == 'a' || ch == 't') {
                    cmd[cmdidx++] = ch;
                    if (cmdidx > 1) {
                        if ((cmd[0] == 'A' || cmd[0] == 'a') &&
                            (cmd[1] == 'T' || cmd[1] == 't')) {
                            seq = PARSE_START;
                        } else retval = AT_PARSE_ERROR;
                    }
                }
                break;
            case PARSE_START:
                if (ch == ' ') {
                    seq = PARSE_PARAM;
                    paramidx = 0;
                } else //ATCSMODEL
                if (cmdidx > 8 &&
                    (cmd[0] == 'A' || cmd[0] == 'a') && (cmd[1] == 'T' || cmd[1] == 't') && (cmd[2] == 'C' || cmd[2] == 'c') &&
                    (cmd[3] == 'S' || cmd[3] == 's') && (cmd[4] == 'M' || cmd[4] == 'm') && (cmd[5] == 'O' || cmd[5] == 'o') &&
                    (cmd[6] == 'D' || cmd[6] == 'd') && (cmd[7] == 'E' || cmd[7] == 'e') && (cmd[8] == 'L' || cmd[8] == 'l')) 
                {
                    seq = PARSE_PARAM;
                    paramidx = 0;
                    param[paramidx++] = ch;
                } else if (ch == '\r' || ch == '\n') {
                    seq = PARSE_COMPL;
                } else {
                    if (cmdidx >= sizeof(cmd)) cmdidx = 0;
                    cmd[cmdidx++] = ch;
                }
                break;
            case PARSE_PARAM:
                if (ch == '\r' || ch == '\n') {
                    seq = PARSE_COMPL;
                } else {
                    if (paramidx >= sizeof(param)) paramidx = 0;
                    param[paramidx++] = ch;
                }
                break; 
            case PARSE_COMPL:
            {
                uint8_t chno = 0;
                int8_t idx = check_atcommand(cmd, &chno);
                if (idx >= 0) {
                    status->channelno = chno;
                    status->atcommand = atcommand_type[idx].atcommand;
                    status->type = atcommand_type[idx].type;
                    _strcpy(status->szcommand, cmd);
                    switch (status->type) {
                        case AT_READONLY: status->isApply = false; break;
                        case AT_APPLYONLY: param_copy(status->apply_params, param); status->isApply = true; break;
                        case AT_READAPPLY: status->isApply = (param_copy(status->apply_params, param) > 0) ? true : false; break;
                        case AT_STRAPPLY: status->isApply = (_strcpy(status->apply_params, param) > 0) ? true : false; break;
                    }
                    return AT_PARSE_OK;
                } else
                    return AT_PARSE_ERROR;
            }
                break;
		}
        if (ch) buff++;
	} while(ch);

	return retval;
}

static uint8_t split_param(char* src, char* params, uint8_t m, uint8_t n)
{
    if(!src || !params) return 0;
    uint8_t len = _strlen(src);
    uint8_t i = 0, cnt = 0;
    while (len--) {
        char ch = *src;
        if (ch == ',') {
            if (cnt >= m) return 0;
            *((params + (n * cnt++)) + i++) = '\0';
            i = 0;
        } else {
            *((params + (n * cnt)) + i++) = ch;
            if (i >= n) return 0;
        }
        src++;
    }
    *((params + (n * cnt)) + i++) = '\0';
    return cnt+1;
}


//Helper function to execute some blocking delay.
static void update_command(s_AtCommand_status* status)
{
    uint8_t apply;
    uint8_t chno = ((status->channelno - 1) < 0) ? 0 : status->channelno - 1;

    switch (status->atcommand) 
    {
        // h/w reset
        case ATCRST:
            ResetDeviceCleanly();
            return;
        // HID mode reset
#ifdef SUPPORT_BOOLTLOADER
        case ATCBLRST:
            ResetBootloaderCleanly();
            return;
#endif
        case ATCSMODEL:
            if (_strlen(status->apply_params) != 8) break;
            if (_isdigit(status->apply_params, 8)) {
                nvm_clear_all();
                for (int i = 0 ; i < 8 ; i++) g_nvm.flash.serialNum[i] = status->apply_params[i];
            } break;        
        case ATCMODE: 
            apply = (uint8_t)_atoi(status->apply_params); 
            if (apply >= RAW_MODE && apply <= FIL_MODE) g_nvm.flash.filter_mode = apply; break;
        case ATCTCMP:
            apply = (uint8_t)_atoi(status->apply_params);
            if (apply >= TEMP_CMP_OFF && apply <= TEMP_CMP_ON) g_nvm.flash.temp_cmp = apply; break;
        case ATCC: g_nvm.flash.temp_corf = CELSIUS; break;
        case ATCF: g_nvm.flash.temp_corf = FAHRENHEIT; break;
        case ATCOFF:
        {
            if (chno == 0)      g_nvm.flash.gasOffset[0] = _atof(status->apply_params); 
            else if (chno == 1) g_nvm.flash.gasOffset[1] = _atof(status->apply_params);    
            else if (chno == 2) g_nvm.flash.gasOffset[2] = _atof(status->apply_params); 
            else if (chno == 3) g_nvm.flash.co2Offset = _atoi(status->apply_params);
            else if (chno == 4) g_nvm.flash.tempOffset = _atof(status->apply_params);
            else if (chno == 5) g_nvm.flash.humiOffset = _atof(status->apply_params);
        } break;     
        case ATCCAL:
            if (chno >= 0 && chno <= 2) {
                if (split_param(status->apply_params, status->manual_params, 3, 32) >= 3) {
                    g_nvm.flash.baseLine_uV[chno] = _atof(status->manual_params[0]);
                    g_nvm.flash.span_uV[chno] = _atof(status->manual_params[1]); 
                    g_nvm.flash.CalibrationGas[chno] = _atof(status->manual_params[2]);
                    g_nvm.flash.calib_temp = Last_value.tempHumi[CMP_TEMP];
                    if (chno == 0 || chno == 2) {
                        if (g_nvm.flash.CalibrationGas[chno] != 0.0f)
                            g_nvm.flash.gas_slope[chno] = (g_nvm.flash.span_uV[chno] - g_nvm.flash.baseLine_uV[chno]) / g_nvm.flash.CalibrationGas[chno];  
                    } else //chNum =1, O2    
                        g_nvm.flash.gas_slope[chno] = (g_nvm.flash.baseLine_uV[chno] - g_nvm.flash.span_uV[chno]) / (20.8f - g_nvm.flash.CalibrationGas[chno]);
                }   
            } break;
        case ATCZCAL: CMD_manual_cal(); break;
        case ATCCSP:
            if (chno >= 0 && chno <= 2) {
                g_nvm.flash.CalibrationGas[chno] = _atof(status->apply_params);
                g_nvm.flash.span_uV[chno] = Last_value.toxic_gas[chno][LV_GAS_VOLT];  
                if (chno == 0 || chno == 2) {
                    if (g_nvm.flash.CalibrationGas[chno] != 0.0f)
                        g_nvm.flash.gas_slope[chno] = (g_nvm.flash.span_uV[chno] - g_nvm.flash.baseLine_uV[chno]) / g_nvm.flash.CalibrationGas[chno];
                } else //chNum = 1, O2    
                    g_nvm.flash.gas_slope[chno] = (g_nvm.flash.baseLine_uV[chno] - g_nvm.flash.span_uV[chno]) / (20.8f - g_nvm.flash.CalibrationGas[chno]);
            } break;
        case ATCCZRA:
        {
            g_nvm.flash.baseLine_uV[0] = Last_value.toxic_gas[0][LV_GAS_VOLT];
            g_nvm.flash.baseLine_uV[1] = Last_value.toxic_gas[1][LV_GAS_VOLT];
            g_nvm.flash.baseLine_uV[2] = Last_value.toxic_gas[2][LV_GAS_VOLT];
            if (g_nvm.flash.CalibrationGas[0] != 0.0f)     
                g_nvm.flash.gas_slope[0] = (g_nvm.flash.span_uV[0] - g_nvm.flash.baseLine_uV[0]) / g_nvm.flash.CalibrationGas[0];  
            if (g_nvm.flash.CalibrationGas[2] != 0.0f)     
                g_nvm.flash.gas_slope[2] = (g_nvm.flash.span_uV[2] - g_nvm.flash.baseLine_uV[2]) / g_nvm.flash.CalibrationGas[2];      
            g_nvm.flash.gas_slope[1] = (g_nvm.flash.baseLine_uV[1] - g_nvm.flash.span_uV[1]) / (20.8f - g_nvm.flash.CalibrationGas[1]);
            g_nvm.flash.calib_temp = Last_value.tempHumi[CMP_TEMP];
            CMD_manual_cal();
        } break; 
        case ATCCZR:
            if (chno >= 0 && chno <= 2) {
                g_nvm.flash.baseLine_uV[chno] = Last_value.toxic_gas[chno][LV_GAS_VOLT];
                g_nvm.flash.calib_temp = Last_value.tempHumi[CMP_TEMP];
                if (chno == 0 || chno == 2) {
                    if (g_nvm.flash.CalibrationGas[chno] != 0.0f)
                        g_nvm.flash.gas_slope[chno] = (g_nvm.flash.span_uV[chno] - g_nvm.flash.baseLine_uV[chno]) / g_nvm.flash.CalibrationGas[chno];
                } else //chno= 1, O2       
                    g_nvm.flash.gas_slope[chno] = (g_nvm.flash.baseLine_uV[chno] - g_nvm.flash.span_uV[chno]) / (20.8f - g_nvm.flash.CalibrationGas[chno]);  
            } break;    
        case ATCWIN: 
            if (chno >= 0 && chno <= 3) {
                apply = (uint8_t)_atoi(status->apply_params); 
                if (apply >= LEVEL_1 && apply <= LEVEL_15) {
                    g_nvm.flash.window_size[chno] = apply;
                    Set_KalmanFilter_Sensitivity(&kalmanFilter[chno], g_nvm.flash.window_size[chno]);
                }
            } break;
        case ATCUSED: g_nvm.flash.usedtime = (uint32_t)_atoi(status->apply_params); break;      
        case ATCEXTM: g_nvm.flash.exectime = (uint32_t)_atoi(status->apply_params); break;
        case ATCCO2ABC: apply = (uint8_t)_atoi(status->apply_params);
                        if (apply >= OFF && apply <= ON) {
                            g_nvm.flash.CO2_ABC = apply; 
                            tes0902_ABC_OFF(g_nvm.flash.CO2_ABC); 
                        } break;
        case ATCKMFEM: 
            g_nvm.flash.KMf_e_measure =  _atof(status->apply_params);
            Kalman_setMeasurementError(&kalmanFilter[1], g_nvm.flash.KMf_e_measure);
            break;                
	} //end switch
    
    if (status->atcommand >= ATCSMODEL && status->atcommand <= ATCEXTM)
        status->last_update = nvm_write_flash_all(&g_nvm.flash);
}

static void build_response(const s_AtCommand_status* status)
{
    uint8_t chno = ((status->channelno - 1) < 0) ? 0 : status->channelno - 1;
    uint8_t len = _strcpy(&writeBuffer[0], status->szcommand);
    writeBuffer[len++] = ' ';
    
    if (status->atcommand >= ATCSMODEL && status->atcommand <= ATCEXTM)
        nvm_read_flash_all(&g_nvm.flash);

	switch (status->atcommand)
    {
        case ATCZ: writeBuffer[len++] = 'O'; writeBuffer[len++] = 'K';  break;
        case ATCVER: len += _strcpy(&writeBuffer[len], BOARD_NAME"_"VERSION); break;
        case ATCUV:
        {
            len += _ftoa(Last_value.toxic_gas[0][LV_GAS_VOLT], &writeBuffer[len]); writeBuffer[len++] = ',';
            len += _ftoa(Last_value.toxic_gas[1][LV_GAS_VOLT], &writeBuffer[len]); writeBuffer[len++] = ',';
            len += _ftoa(Last_value.toxic_gas[2][LV_GAS_VOLT], &writeBuffer[len]); writeBuffer[len++] = ',';
            len += _itoa(Last_value.co2_gas, &writeBuffer[len]);
        } break;
        case ATCQ:
        {
            for (uint8_t chNum = 0; chNum < CHANNEL_COUNT; chNum++) {
                if (g_sens_measure.isGasSensor_Success[chNum] && g_nvm.flash.CalibrationGas[chNum] != 0.0f && g_led_status.isBlink == false) {
                    if (chNum == 0 ||chNum == 2)
                        len += _itoa((uint32_t)Last_value.toxic_gas[chNum][LV_GAS_CONCEN], &writeBuffer[len]);
                    else
                        len += _ftoa(Last_value.toxic_gas[chNum][LV_GAS_CONCEN], &writeBuffer[len]);
                } else    len += _strcpy(&writeBuffer[len], "----"); 
                writeBuffer[len++] = ',';
            }
            if (g_led_status.isBlink == false)  len += _itoa(Last_value.co2_gas, &writeBuffer[len]);
            else    len += _strcpy(&writeBuffer[len], "----");    
        } break;
        case ATCH:
        {
            for (uint8_t chNum = 0; chNum < CHANNEL_COUNT; chNum++) {
                if (g_sens_measure.isGasSensor_Success[chNum] && g_nvm.flash.CalibrationGas[chNum] != 0.0f && g_led_status.isBlink == false) {
                    if (chNum == 0 ||chNum == 2)
                        len += _itoa((uint32_t)Last_value.toxic_gas[chNum][LV_GAS_CONCEN], &writeBuffer[len]);
                    else
                        len += _ftoa(Last_value.toxic_gas[chNum][LV_GAS_CONCEN], &writeBuffer[len]); 
                } else    len += _strcpy(&writeBuffer[len], "----"); 
                writeBuffer[len++] = ',';
            }
            if (g_led_status.isBlink == false)   len += _itoa(Last_value.co2_gas, &writeBuffer[len]);
            else    len += _strcpy(&writeBuffer[len], "----");
            writeBuffer[len++] = ',';
            if (g_sens_measure.isTemp_Success) {
                float temp = Last_value.tempHumi[LV_TEMP];
                if (g_nvm.flash.temp_corf == FAHRENHEIT)
                    sensor_temp_fahrenheit(&temp);
                len += _ftoa(temp, &writeBuffer[len]);  writeBuffer[len++] = ',';   
                len += _ftoa(Last_value.tempHumi[LV_HUMI], &writeBuffer[len]);
            } else  len += _strcpy(&writeBuffer[len], "----,----");
        } break;
        case ATCD:
             if (g_sens_measure.isTemp_Success) {
                float temp = Last_value.tempHumi[LV_TEMP];
                if (g_nvm.flash.temp_corf == FAHRENHEIT)
                    sensor_temp_fahrenheit(&temp);
                len += _ftoa(temp, &writeBuffer[len]);  
            } else len += _strcpy(&writeBuffer[len], "----");
            writeBuffer[len++] = ',';
            len += _ftoa(Last_value.tempHumi[CMP_TEMP], &writeBuffer[len]);
            break;
        case ATCMODE: len += _itoa(g_nvm.flash.filter_mode, &writeBuffer[len]); break;
        case ATCTCMP: len += _itoa(g_nvm.flash.temp_cmp, &writeBuffer[len]); break;
        case ATCC: case ATCF:
            if (status->last_update) { writeBuffer[len++] = 'O'; writeBuffer[len++] = 'K'; }
            else { writeBuffer[len++] = 'F'; writeBuffer[len++] = 'A'; writeBuffer[len++] = 'I'; writeBuffer[len++] = 'L'; }
            break;
        case ATCZCAL: writeBuffer[len++] = 'O'; writeBuffer[len++] = 'K'; break;
        case ATCCAL:
            if (chno >= 0 && chno <= 2) {
                len += _ftoa(g_nvm.flash.baseLine_uV[chno], &writeBuffer[len]);      writeBuffer[len++] = ',';
                len += _ftoa(g_nvm.flash.span_uV[chno], &writeBuffer[len]);          writeBuffer[len++] = ',';
                len += _ftoa(g_nvm.flash.CalibrationGas[chno], &writeBuffer[len]);   writeBuffer[len++] = ',';
            } else  len += _strcpy(&writeBuffer[len], "error"); 
            break;
        case ATCSMODEL: len--;  // remove ' '
        case ATCMODEL: for (int i = 0 ; i < 8 ; i++) writeBuffer[len++] = g_nvm.flash.serialNum[i]; break;
        case ATCCZRA: 
            len += _ftoa(g_nvm.flash.baseLine_uV[0], &writeBuffer[len]); writeBuffer[len++] = ',';
            len += _ftoa(g_nvm.flash.baseLine_uV[1], &writeBuffer[len]); writeBuffer[len++] = ',';
            len += _ftoa(g_nvm.flash.baseLine_uV[2], &writeBuffer[len]); 
            break;
        case ATCCZR:  
            if (chno >= 0 && chno <= 2) len += _ftoa(g_nvm.flash.baseLine_uV[chno], &writeBuffer[len]);    
            else    len += _strcpy(&writeBuffer[len], "error"); break;  
        case ATCCSP:
            if (chno >= 0 && chno <= 2) {
                len += _ftoa(g_nvm.flash.CalibrationGas[chno], &writeBuffer[len]);        writeBuffer[len++] = ',';
                len += sprintf(&writeBuffer[len], "%.2f", g_nvm.flash.gas_slope[chno]);   writeBuffer[len++] = ',';
                len += _ftoa(g_nvm.flash.baseLine_uV[chno], &writeBuffer[len]);           writeBuffer[len++] = ',';
                len += _ftoa(g_nvm.flash.span_uV[chno], &writeBuffer[len]);               writeBuffer[len++] = ',';
                len += _ftoa(g_nvm.flash.calib_temp, &writeBuffer[len]);
            } else  len += _strcpy(&writeBuffer[len], "error"); 
            break;
        case ATCOFF:
        {
            if (chno == 0)      len += _ftoa(g_nvm.flash.gasOffset[0], &writeBuffer[len]); 
            else if (chno == 1) len += _ftoa(g_nvm.flash.gasOffset[1], &writeBuffer[len]);    
            else if (chno == 2) len += _ftoa(g_nvm.flash.gasOffset[2], &writeBuffer[len]);
            else if (chno == 3) len += _itoa(g_nvm.flash.co2Offset, &writeBuffer[len]);
            else if (chno == 4) len += _ftoa(g_nvm.flash.tempOffset, &writeBuffer[len]);
            else if (chno == 5) len += _ftoa(g_nvm.flash.humiOffset, &writeBuffer[len]);   
        } break;
        case ATCWIN: 
            if (chno >= 0 && chno <= 3) len += _itoa(g_nvm.flash.window_size[chno], &writeBuffer[len]); 
            else    len += _strcpy(&writeBuffer[len], "error"); break;
        case ATCEXTM: len += _itoa(g_nvm.flash.exectime, &writeBuffer[len]); break;    
        case ATCUSED: len += _itoa(g_nvm.flash.usedtime, &writeBuffer[len]); break;    
        case ATCBLDTM: len += _itoa(g_nvm.flash.buildtime, &writeBuffer[len]); break;
        case ATCCO2ABC: // len += sprintf(writeBuffer,"convData: %x| %x,%x\r\n", convData.u16_data, g_Wbuff[6],g_Wbuff[7]); 
                        len += _itoa(g_nvm.flash.CO2_ABC, &writeBuffer[len]); break;
        case ATCKMFEM: len += _ftoa(g_nvm.flash.KMf_e_measure, &writeBuffer[len]);  break;                
	} //end switch
    writeBuffer[len++] = '\r'; writeBuffer[len++] = '\n'; writeBuffer[len++] = '\0';
    write_command(writeBuffer);
}

static void clear_command(void)
{
    g_atcommand.readidx = 0;
	g_atcommand.status.channelno = 0;
    g_atcommand.status.last_update = false;
	g_atcommand.status.atcommand = ATNONE;
	g_atcommand.status.isApply = false;
    g_atcommand.status.type = AT_READONLY;

    _strclr(g_atcommand.read_cmd, sizeof(g_atcommand.read_cmd));
    _strclr(g_atcommand.status.szcommand, sizeof(g_atcommand.status.szcommand));
	_strclr(g_atcommand.status.apply_params, sizeof(g_atcommand.status.apply_params));
    _strclr(writeBuffer, CDC_DATA_IN_EP_SIZE);
}
