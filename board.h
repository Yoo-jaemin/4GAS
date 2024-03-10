/* 
 * File:   board.h
 *
 */

#ifndef BOARD_H
#define	BOARD_H

#include <stdint.h>


#define BOARD_NAME      "UA58-KFG-U"
#define VERSION         "test"//"4v6"
#define CHANNEL_COUNT   3 
#define GAS_RESOLUTION  0.5f     //ppm

/* Calib parameter */
#define ADC_GAIN_CO     FSR_0p256V 
#define LMP_GAIN_CO     R2p75K      

#define ADC_GAIN_O2     FSR_0p256V  
#define LMP_GAIN_O2     R2p75K  

#define ADC_GAIN_H2S    FSR_1p024V  
#define LMP_GAIN_H2S    R2p75K      

#define LMP_RLOAD       R100
#define LMP_INTZ        S20P
   

//========================================================================
#define MEASURE_DELAY_COUNT     100            // 1sec for timer interrupt
#define THREE_MINUTE_WARMUP_DELAY_COUNT 18000            
#define TWO_YEAR_COUNT          17520          

#define ONE_HOUR_COUNT          360000
#define NO_VALUE                9876.0f

// support bootloader app, start address 0x2000
#define SUPPORT_BOOLTLOADER

#define USE_KALMAN_FILTER
#define CERTIFICATION_TEST


typedef union {
    int16_t  i16_data;
    uint16_t u16_data;
    __pack uint8_t u8_Data[2];
} u_2byte_Conv;

enum {
    CELSIUS,
    FAHRENHEIT
};

enum { 
    RAW_MODE, // ATCD Raw
    FIL_MODE  // ATCD
};

enum {
    GAS_CH0,
    GAS_CH1,
    GAS_CH2,
    GAS_CH3
};

enum {
    USB_NONE,
    USB_DETACHED,
    USB_ATTACHED
};

enum {
    TEMP_CMP_OFF,
    TEMP_CMP_ON
};

enum {
    OFF,
    ON
};

/* ********************************
*	PIC18(L)F2xJ50 - USB CHIP
*
*	PORT DESCRIPTION
******************************** */
#define _XTAL_FREQ          48000000

// LED
#define LED_RED_LAT     LATCbits.LATC1
#define LED_RED_TRIS    TRISCbits.TRISC1  
#define LED_GREEN_LAT   LATCbits.LATC0       
#define LED_GREEN_TRIS  TRISCbits.TRISC0      

// I2C1 - 100kHz
#define I2C_SCL_TRIS        TRISBbits.TRISB4
#define I2C_SDA_TRIS        TRISBbits.TRISB5

// USART1
#define UART_TX_TRIS        TRISCbits.TRISC6
#define UART_RX_TRIS        TRISCbits.TRISC7

// LMP91000 Enable pin
#define LMP91000_MENB_LAT_CH0   LATBbits.LATB0
#define LMP91000_MENB_TRIS_CH0  TRISBbits.TRISB0  
#define LMP91000_MENB_LAT_CH1   LATBbits.LATB1
#define LMP91000_MENB_TRIS_CH1  TRISBbits.TRISB1 
#define LMP91000_MENB_LAT_CH2   LATBbits.LATB2
#define LMP91000_MENB_TRIS_CH2  TRISBbits.TRISB2 

#define USE_UART_INTERRUPT


// Internal FLASH
//#define FLASH_BASE_ADDRESS  0x7800 // 30,720
// 0xf000 ~ 0xfe00 [61,440 ~ 65,024]
#define FLASH_BASE_ADDRESS  0xf400 // 62,464 at 61 page for nvm

// bootloader
// Linker Option -> Additional Option -> code offset to 0x2000
#ifdef SUPPORT_BOOLTLOADER
#define APP_SIGNATURE_ADDRESS   0x2006  //0x2006 and 0x2007 contains the "signature" WORD, indicating successful erase/program/verify operation
#define APP_SIGNATURE_VALUE     0x600D  //leet "GOOD", implying that the erase/program was a success and the bootloader intentionally programmed the APP_SIGNATURE_ADDRESS with this value
#endif

//#define LMP91000_I2C_ADDRESS    0x90

#define USE_SHT3X_COMPENSATE    0
//#define SHT3X_I2C_ADDRESS 	0x44 // 7bit address
#define SHT3X_I2C_ADDRESS 	0x88 // 8bit address

#define TEMPUS_CO2_BAUDRATE_9600  9600



#ifdef	__cplusplus
extern "C" {
#endif

    void write_command(const uint8_t* buff); // output through usb cdc
    uint16_t* get_product_desc(); // get pointer of sd002.string[25]

#ifdef	__cplusplus
}
#endif

#endif	/* BOARD_H */
