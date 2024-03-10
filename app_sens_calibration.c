/*
 * File:   app_sens_calibration.c
 * Author: brian
 *
 * Created on 2022 2/22
 */

#include <stdbool.h>
#include <stdint.h>

#include "board.h"
#include "nvm.h"

#include "bsp/leds.h"
#include "bsp/TES0902.h"
#include "sensor.h"
#include "app_sens_measure.h"

#include "app_led_task.h"
#include "app_sens_calibration.h"

/* ******************* Global Variable ******************* */
extern uint8_t g_usb_state;
extern s_Nvm g_nvm;
extern s_Sens_Calib g_sens_calib;
extern s_Led_Status g_led_status;
extern s_Sens_Measure_value Last_value;

volatile uint32_t g_calib_count;


/* ****************** Funtion Prototype ****************** */
static bool sens_ZeroCalibration(void);


void sens_ZeroCalib_init(void)
{
    g_sens_calib.seq = CALIB_INIT;
    g_sens_calib.zero_cal = false;
    g_calib_count = 0;    
}

static bool sens_ZeroCalibration(void)
{
    g_nvm.flash.baseLine_uV[0] = Last_value.toxic_gas[0][LV_GAS_VOLT];
    g_nvm.flash.baseLine_uV[1] = Last_value.toxic_gas[1][LV_GAS_VOLT];
    g_nvm.flash.baseLine_uV[2] = Last_value.toxic_gas[2][LV_GAS_VOLT];         
    g_nvm.flash.gas_slope[0] = (g_nvm.flash.span_uV[0] - g_nvm.flash.baseLine_uV[0]) / g_nvm.flash.CalibrationGas[0];  
    g_nvm.flash.gas_slope[1] = (g_nvm.flash.baseLine_uV[1] - g_nvm.flash.span_uV[1]) / (20.8f - g_nvm.flash.CalibrationGas[1]);     
    g_nvm.flash.gas_slope[2] = (g_nvm.flash.span_uV[2] - g_nvm.flash.baseLine_uV[2]) / g_nvm.flash.CalibrationGas[2];      
    g_nvm.flash.calib_temp = Last_value.tempHumi[CMP_TEMP];
    CMD_manual_cal();
    g_sens_calib.zero_cal = nvm_write_flash_all(&g_nvm.flash);
    return g_sens_calib.zero_cal;
}

void sens_ZeroCalib_task(void)
{
    if ((g_usb_state == USB_DETACHED) && (g_sens_calib.zero_cal == false)) {
        switch (g_sens_calib.seq)
        {
            case CALIB_INIT:
                LED_Off();
                LED_On(LED_GREEN);
                g_calib_count = 0;
                g_sens_calib.seq = CALIB_WAIT;
                break;
            case CALIB_WAIT:
                if (g_calib_count > ZERO_CALIB_TIME) {
                    g_calib_count = 0;
                    g_sens_calib.seq = CALIB_ZERO;
                } break;
            case CALIB_ZERO:
                if (sens_ZeroCalibration() == false) 
                    g_sens_calib.seq = CALIB_ZERO;
                else    g_led_status.seq = LED_ZEROCAL_END;
                break;
        }//end switch
    } else {
        g_calib_count = 0;
        return;
    }        
}//end function
