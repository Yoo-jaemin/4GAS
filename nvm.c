
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "board.h"
#include "bsp/flash.h"

#include "bsp/lmp91000.h"
#include "bsp/ads1114.h"

#include "util/build_time.h"

#include "filter.h"
#include "nvm.h"


extern s_Nvm g_nvm;

static void nvm_clear_flash(void)
{
    g_nvm.flash.filter_mode = FIL_MODE;
    g_nvm.flash.temp_corf = CELSIUS;
    g_nvm.flash.temp_cmp = TEMP_CMP_ON; 
    g_nvm.flash.CO2_ABC = OFF;
    
 // ================== Calib parameter ================            
    g_nvm.flash.span_uV[0] = 7114.0f;  //7114.0f;        
    g_nvm.flash.span_uV[1] = 67169.0f;  //67169.0f;      
    g_nvm.flash.span_uV[2] = 199570.0f;  //199570.0f;      
    
    g_nvm.flash.baseLine_uV[0] = 0.0f;        
    g_nvm.flash.baseLine_uV[1] = 0.0f; 
    g_nvm.flash.baseLine_uV[2] = 0.0f; 
    
    g_nvm.flash.CalibrationGas[0] = 50.0f;    //50.0f;     
    g_nvm.flash.CalibrationGas[1] = 10.0f;   //10.0f;    
    g_nvm.flash.CalibrationGas[2] = 49.8f;   //49.8f;     
    
    g_nvm.flash.gas_slope[0] = 0.0f;
    g_nvm.flash.gas_slope[1] = 0.0f;
    g_nvm.flash.gas_slope[2] = 0.0f;
// ====================================================
    g_nvm.flash.gasOffset[0] = 0.0f;
    g_nvm.flash.gasOffset[1] = 0.0f;
    g_nvm.flash.gasOffset[2] = 0.0f;
    
    g_nvm.flash.window_size[0] = LEVEL_15; 
    g_nvm.flash.window_size[1] = LEVEL_12; 
    g_nvm.flash.window_size[2] = LEVEL_12; 
    g_nvm.flash.window_size[3] = LEVEL_1; 
            
    g_nvm.flash.co2Offset = 0;
    g_nvm.flash.tempOffset = 0.0f;
    g_nvm.flash.humiOffset = 0.0f;
    g_nvm.flash.calib_temp = 0.0f;
    
    g_nvm.flash.KMf_e_measure = 40.0f;

    g_nvm.flash.exectime = 0;
    g_nvm.flash.usedtime = 0;
    g_nvm.flash.buildtime = (uint32_t)build_time();
}

void nvm_clear(void)
{
    nvm_clear_flash();
}

void nvm_clear_all(void)
{
    uint8_t i;
    nvm_clear();
    for (i = 0 ; i < 8 ; i++) g_nvm.flash.serialNum[i] = 0xff;
}

static bool nvm_serialno_check(const uint8_t* sno)
{
    uint8_t i;
    for (i = 0 ; i < 8 ; i++) {
        if (sno[i] == 0xff || sno[i] == 0x00)
            return false;
    }
    return true;
}

void nvm_init(void)
{
    uint8_t i;

    nvm_read_flash_all(&g_nvm.flash);
    g_nvm.status = NVM_INIT_FLASH;

    if (nvm_serialno_check(g_nvm.flash.serialNum)) {
        uint16_t* pdesc = get_product_desc();
        for (i = 0 ; i < 8 ; i++) 
            pdesc[i + 11] = g_nvm.flash.serialNum[i];
    } else {
        nvm_clear();
        if (nvm_write_flash_all(&g_nvm.flash) == false)
            return;
        g_nvm.status = NVM_INIT_FLASH;   
    }
}

bool nvm_write_flash_all(u_Flash_Nvm* flash)
{
    flash_clear(FLASH_BASE_ADDRESS, (FLASH_BASE_ADDRESS + FLASH_ALL_PARAM_SIZE));
    return flash_writeBytes(FLASH_BASE_ADDRESS, flash->byte, FLASH_ALL_PARAM_SIZE);
}

void nvm_read_flash_all(u_Flash_Nvm* flash)
{
    flash_readBytes(FLASH_BASE_ADDRESS, flash->byte, FLASH_ALL_PARAM_SIZE);
}
