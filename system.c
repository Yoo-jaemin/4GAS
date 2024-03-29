/*******************************************************************************
Copyright 2016 Microchip Technology Inc. (www.microchip.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

To request to license the code under the MLA license (www.microchip.com/mla_license), 
please contact mla_licensing@microchip.com
*******************************************************************************/
#include <stdio.h>
#include "system.h"
#include "usb/usb.h"
#include "sensor.h"
#include "board.h"

/** CONFIGURATION Bits **********************************************/
// CONFIG1L
#pragma config WDTEN = OFF      // Watchdog Timer (Disabled - Controlled by SWDTEN bit)
#pragma config PLLDIV = 2       // PLL Prescaler Selection bits (Divide by 2 (8 MHz oscillator input))
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset  (Enabled)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// CONFIG1H
#pragma config CPUDIV = OSC1    // CPU System Clock Postscaler (No CPU system clock divide)
#pragma config CP0 = OFF        // Code Protect (Program memory is not code-protected)

// CONFIG2L
#pragma config OSC = INTOSCPLL  // Oscillator (INTOSCPLL)
#pragma config T1DIG = OFF      // T1OSCEN Enforcement (Secondary Oscillator clock source may not be selected)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator (High-power operation)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor (Enabled)
#pragma config IESO = OFF       // Internal External Oscillator Switch Over Mode (Disabled)

// CONFIG2H
#pragma config WDTPS = 1024     // Watchdog Postscaler (1:1024) (2.7 seconds)
//#pragma config WDTPS = 2048     // Watchdog Postscaler (1:2048)
//#pragma config WDTPS = 4096       // 18.1 sec
//#pragma config WDTPS = 8192     // 36.2 sec 
//#pragma config WDTPS = 32768    // 2.25 min 

// CONFIG3L
#pragma config DSWDTOSC = INTOSCREF// DSWDT Clock Select (DSWDT uses INTRC)
#pragma config RTCOSC = T1OSCREF// RTCC Clock Select (RTCC uses T1OSC/T1CKI)
#pragma config DSBOREN = OFF    // Deep Sleep BOR (Disabled)
#pragma config DSWDTEN = ON     // Deep Sleep Watchdog Timer (Enabled)

//#pragma config DSWDTPS = 2048   // Deep Sleep Watchdog Postscaler (1:2,048 (2.1 seconds))
//#pragma config DSWDTPS = 8192   // Deep Sleep Watchdog Postscaler (1:8,192 (8.5 seconds))
#pragma config DSWDTPS = K32    // Deep Sleep Watchdog Postscaler (1:32,768 (34 seconds))
//#pragma config DSWDTPS = K131   // Deep Sleep Watchdog Postscaler (1:131,072 (135 seconds))
//#pragma config DSWDTPS = K524   // Deep Sleep Watchdog Postscaler (1:524,288 (9 minutes))

// CONFIG3H
#pragma config IOL1WAY = ON     // IOLOCK One-Way Set Enable bit (The IOLOCK bit (PPSCON<0>) can be set once)
#pragma config MSSP7B_EN = MSK7 // MSSP address masking (7 Bit address masking mode)

// CONFIG4L
#pragma config WPFP = PAGE_1    // Write/Erase Protect Page Start/End Location (Write Protect Program Flash Page 1)
#pragma config WPEND = PAGE_0   // Write/Erase Protect Region Select (valid when WPDIS = 0) (Page 0 through WPFP<5:0> erase/write protected)
#pragma config WPCFG = OFF      // Write/Erase Protect Configuration Region (Configuration Words page not erase/write-protected)

// CONFIG4H
#pragma config WPDIS = OFF      // Write Protect Disable bit (WPFP<5:0>/WPEND region ignored)


/*********************************************************************
* Function: void SYSTEM_Initialize( SYSTEM_STATE state )
*
* Overview: Initializes the system.
*
* PreCondition: None
*
* Input:  SYSTEM_STATE - the state to initialize the system into
*
* Output: None
*
********************************************************************/
void SYSTEM_Initialize( SYSTEM_STATE state )
{
    switch(state)
    {
        case SYSTEM_STATE_USB_START:
            //In this devices family of USB microcontrollers, the PLL will not power up and be enabled
            //by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
            //This allows the device to power up at a lower initial operating frequency, which can be
            //advantageous when powered from a source which is not gauranteed to be adequate for 48MHz
            //operation.  On these devices, user firmware needs to manually set the OSCTUNE<PLLEN> bit to
            //power up the PLL.
            {
                unsigned int pll_startup_counter = 600;
                OSCTUNEbits.PLLEN = 1;  //Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
                while(pll_startup_counter--);
            }
            //Device switches over automatically to PLL output after PLL is locked and ready.

            //Configure all I/O pins to use digital input buffers.  The PIC18F87J50 Family devices
            //use the ANCONx registers to control this, which is different from other devices which
            //use the ADCON1 register for this purpose.

            break;
            
        case SYSTEM_STATE_USB_SUSPEND: 
            //Should also configure all other I/O pins for lowest power consumption.
            //Typically this is done by driving unused I/O pins as outputs and driving them high or low.
            //In this example, this is not done however, in case the user is expecting the I/O pins
            //to remain tri-state and has hooked something up to them.
            //Leaving the I/O pins floating will waste power and should not be done in a
            //real application.

            //Sleep on sleep, 125kHz selected as microcontroller clock source
            break;
            
        case SYSTEM_STATE_USB_RESUME:
            OSCCON = 0x70;		//Primary clock source selected.
            
            //Adding a software start up delay will ensure
            //that the primary oscillator and PLL are running before executing any other
            //code.  If the PLL isn't being used, (ex: primary osc = 48MHz externally applied EC)
            //then this code adds a small unnecessary delay, but it is harmless to execute anyway.
            {
                unsigned int pll_startup_counter = 800;                      //Long delay at 31kHz, but ~0.8ms at 48MHz
                while(pll_startup_counter--);			//Clock will switch over while executing this delay loop
            }
            break;
    }
}

			
volatile uint8_t g_tick_count;
extern volatile uint32_t g_large_counter;
extern volatile uint32_t g_led_counter;
extern volatile uint32_t g_usedtime_count;
extern volatile uint32_t g_calib_count;

#if defined(__XC8)
__interrupt(high_priority) void SYS_InterruptHigh(void)
{
    #if defined(USB_INTERRUPT)
        USBDeviceTasks();
    #endif

	/* TIMER0 Interrupt Action */
	if(INTCONbits.TMR0IF)     //1.3ms
	{	
        INTCONbits.TMR0IF = 0;
		g_tick_count++;

		if(g_tick_count > 7)  //10ms
		{
			g_large_counter++;
            g_led_counter++;
            g_usedtime_count++;
            g_calib_count++;
			g_tick_count = 0;
		}
	}  
}
#else
        void YourHighPriorityISRCode();
        void YourLowPriorityISRCode();
        
	//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
	//the reset, high priority interrupt, and low priority interrupt
	//vectors.  However, the current Microchip USB bootloader
	//examples are intended to occupy addresses 0x00-0x7FF or
	//0x00-0xFFF depending on which bootloader is used.  Therefore,
	//the bootloader code remaps these vectors to new locations
	//as indicated below.  This remapping is only necessary if you
	//wish to program the hex file generated from this project with
	//the USB bootloader.  If no bootloader is used, edit the
	//usb_config.h file and comment out the following defines:
	//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER
	//#define PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER

	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x1000
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
	#elif defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x800
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x808
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x818
	#else
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x00
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x08
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x18
	#endif

	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	extern void _startup (void);        // See c018i.c in your C18 compiler dir
	#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
	void _reset (void)
	{
	    _asm goto _startup _endasm
	}
	#endif

        #pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
	void Remapped_High_ISR (void)
	{
	     _asm goto YourHighPriorityISRCode _endasm
	}
	#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
	void Remapped_Low_ISR (void)
	{
	     _asm goto YourLowPriorityISRCode _endasm
	}

	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	//Note: If this project is built while one of the bootloaders has
	//been defined, but then the output hex file is not programmed with
	//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
	//As a result, if an actual interrupt was enabled and occured, the PC would jump
	//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
	//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
	//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
	//would effective reset the application.

	//To fix this situation, we should always deliberately place a
	//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
	//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
	//hex file of this project is programmed with the bootloader, these sections do not
	//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
	//programmed using the bootloader, then the below goto instructions do get programmed,
	//and the hex file still works like normal.  The below section is only required to fix this
	//scenario.
	#pragma code HIGH_INTERRUPT_VECTOR = 0x08
	void High_ISR (void)
	{
	     _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#pragma code LOW_INTERRUPT_VECTOR = 0x18
	void Low_ISR (void)
	{
	     _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#endif	//end of "#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER)"

	#pragma code


	//These are your actual interrupt handling routines.
	#pragma interrupt YourHighPriorityISRCode
	void YourHighPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
        #if defined(USB_INTERRUPT)
	        USBDeviceTasks();
        #endif

	}	//This return will be a "retfie fast", since this is in a #pragma interrupt section
	#pragma interruptlow YourLowPriorityISRCode
	void YourLowPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.

	}	//This return will be a "retfie", since this is in a #pragma interruptlow section
#endif     

    // nakholee append
// enable/disable software controlled watchdog timer
void WDT_initialize(bool enabled)
{
    WDTCONbits.SWDTEN = (enabled == true)?1:0;      /* enable/disable software controlled watchdog timer*/
}

// enter deep sleep mode
void Enter_DeepSleep(void)
{
    WDTCONbits.SWDTEN = 0;

    WDTCONbits.REGSLP = 1;			// On-chip regulator enters low-power operation when device enters Sleep mode   
    OSCCONbits.IDLEN = 0; 			// Device enters Sleep mode on SLEEP instruction
       
    DSCONHbits.DSEN = 1;			// Deep Sleep mode is entered on a SLEEP command
    Sleep();                        // These two instructions must be executed back-to-back          
    
    /* ***********************************************************************************************
     * for deep sleep, execution will restart at reset vector (use WDTCONbits.DS to detect) 
     * Once a wake-up event occurs, the device will perform a Power-on Reset sequence. 
     * Code execution resumes at the device's Reset vector.
    *********************************************************************************************** */ 
}

//Helper function to execute some blocking delay.
void DelayRoutine(unsigned int DelayAmount)
{
    while(DelayAmount)
    {
        ClrWdt();
        DelayAmount--;
    }
}

//Disable the USB module and then wait a long time (long enough for the host
//to correctly detect the USB detach event, process the plug and play event,
//and get itself back into a state ready for a new USB attach event.
static void USBDisableWithLongDelay(void)
{
    USBIsDeviceSuspended() = 0;    //Make sure not in suspend mode
    UCON = 0x00;            //Disable USB module
    DelayRoutine(0xFFFF);   //Wait long time for host to recognize detach event
    USBGetDeviceState() = DETACHED_STATE;
}
//Before resetting the microcontroller, we should shut down the USB module 
//gracefully, to make sure the host correctly recognizes that we detached
//from the bus.  Some USB hosts malfunction/fail to re-enumerate the device
//correctly if the USB device does not stay detached for a minimum amount of
//time before re-attaching to the USB bus.  For reliable operation, the USB
//device should stay detached for as long as a human would require to unplug and
//reattach a USB device (ex: 100ms+), to ensure the USB host software has a 
//chance to process the detach event and configure itself for a state ready for 
//a new attachment event.
void ResetDeviceCleanly(void)
{
    USBDisableWithLongDelay();
    Reset();    
    Nop();
    Nop();
}
// goto bootloader
void ResetBootloaderCleanly(void)
{
    USBDisableWithLongDelay();
    asm("goto 0x001c");
    Nop();
    Nop();
}
