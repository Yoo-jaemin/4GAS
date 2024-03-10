
# 1 "app_sens_calibration.c"

# 15 "C:\Program Files\Microchip\xc8\v2.32\pic\include\c90\stdbool.h"
typedef unsigned char bool;

# 13 "C:\Program Files\Microchip\xc8\v2.32\pic\include\c90\stdint.h"
typedef signed char int8_t;

# 20
typedef signed int int16_t;

# 28
typedef __int24 int24_t;

# 36
typedef signed long int int32_t;

# 52
typedef unsigned char uint8_t;

# 58
typedef unsigned int uint16_t;

# 65
typedef __uint24 uint24_t;

# 72
typedef unsigned long int uint32_t;

# 88
typedef signed char int_least8_t;

# 96
typedef signed int int_least16_t;

# 109
typedef __int24 int_least24_t;

# 118
typedef signed long int int_least32_t;

# 136
typedef unsigned char uint_least8_t;

# 143
typedef unsigned int uint_least16_t;

# 154
typedef __uint24 uint_least24_t;

# 162
typedef unsigned long int uint_least32_t;

# 181
typedef signed char int_fast8_t;

# 188
typedef signed int int_fast16_t;

# 200
typedef __int24 int_fast24_t;

# 208
typedef signed long int int_fast32_t;

# 224
typedef unsigned char uint_fast8_t;

# 230
typedef unsigned int uint_fast16_t;

# 240
typedef __uint24 uint_fast24_t;

# 247
typedef unsigned long int uint_fast32_t;

# 268
typedef int32_t intmax_t;

# 282
typedef uint32_t uintmax_t;

# 289
typedef int16_t intptr_t;




typedef uint16_t uintptr_t;

# 46 "board.h"
typedef union {
int16_t i16_data;
uint16_t u16_data;
__pack uint8_t u8_Data[2];
} u_2byte_Conv;

enum {
CELSIUS,
FAHRENHEIT
};

enum {
RAW_MODE,
FIL_MODE
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

# 143
void write_command(const uint8_t* buff);
uint16_t* get_product_desc();

# 7 "nvm.h"
typedef union {
__pack uint8_t byte[108];
__pack struct {
uint8_t filter_mode;
uint8_t temp_cmp;
uint8_t temp_corf;
uint8_t CO2_ABC;
__pack uint8_t window_size[4];

__pack float baseLine_uV[3];
__pack float span_uV[3];
__pack float CalibrationGas[3];
__pack float gas_slope[3];
__pack float gasOffset[3];

int32_t co2Offset;
float tempOffset;
float humiOffset;
float calib_temp;


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

# 52
void nvm_init(void);
void nvm_clear(void);
void nvm_clear_all(void);

bool nvm_write_flash_all(u_Flash_Nvm* flash);
void nvm_read_flash_all(u_Flash_Nvm* flash);

# 27 "bsp/leds.h"
typedef enum
{
LED_NONE,
LED_RED,
LED_GREEN
} LED;

# 52
void LED_On(LED led);

# 70
void LED_Off(void);

# 88
void LED_Toggle(LED led);

# 106
bool LED_Get(LED led);

# 123
void LED_Enable(LED led);

# 15 "C:\Program Files\Microchip\xc8\v2.32\pic\include\c90\stdbool.h"
typedef unsigned char bool;

# 31 "bsp/TES0902.h"
typedef struct {
__pack uint8_t sync[2];
uint8_t cmd;
uint8_t len;
__pack uint8_t data[8];
__pack uint8_t crc[2];
} s_Tes0902_frame;

# 44
void CMD_get_ppm(void);
void CMD_manual_cal(void);
void CMD_set_ABC_on(uint8_t value);

void CMD_get_version(void);
void CMD_get_serialNumber(void);

uint16_t Calculate_CRC16(uint8_t *crc, int crc_length);
bool CRC_check(void);

void tes0902_polling_mode(void);
void tes0902_ABC_OFF(uint8_t status);

# 13 "sensor.h"
enum e_GasSensValue {
LV_GAS_VOLT,
LV_GAS_CONCEN
};

enum e_SensValue {
LV_TEMP,
LV_HUMI,
CMP_TEMP
};

typedef struct {
bool temp;
bool gas_lmp[3];
} s_Sensor_init;

typedef struct {
__pack float toxic_gas[3][LV_GAS_CONCEN+1];
uint16_t co2_gas;
float tempHumi[CMP_TEMP+1];
} s_Sens_Measure_value;

# 40
void Sensors_initialize(void);
bool sensor_lmp_initialize(uint8_t chNum);
bool sensor_adc_initialize(uint8_t chNum);

bool sensor_ADC_read(uint8_t chNum, int16_t* raw_adc);
float ADCto_uVoltage(int16_t raw_adc, uint8_t gain);
bool gasSensor_read(uint8_t chNum);

bool sensor_read_temp_humi(float* temp, float* humi);
void sensor_temp_fahrenheit(float* temp);
bool tempSensor_read(void);

bool sensor_temp_mode(uint8_t chNum);
bool sensor_gas_mode(uint8_t chNum);
bool CMP_temp_read(uint8_t chNum);
bool gas_init(uint8_t chNum);

# 10 "app_sens_measure.h"
enum e_Measure_Seq {
SENS_INIT,
SENS_READY,
SENS_TEMP_MODE,
SENS_TEMP_MEASURE,
SENS_GAS_MODE,
SENS_GAS_MODE_MEASURE,
SENS_GAS_INIT,
SENS_GAS_MEASURE,
SENS_MEASURE_APPLY,
SENS_MEASURE_CHECK,
SENS_DISPLAY,
SENS_WAIT
};

typedef struct {
uint8_t seq;
uint8_t next_seq;
uint8_t seq_count;

bool isLMP_InitDone[2];
bool isADC_InitDone[2];
bool isTemp_InitDone;

bool isGasSensor_Success[3];
bool isCO2_Success;
bool isTemp_Success;

} s_Sens_Measure;

# 45
void sensMeasure_init();
void sensMeasure_task();

# 5 "app_led_task.h"
typedef enum {
LED_INIT,
LED_READY,
LED_WAIT_OFF,
LED_CALIB,
LED_WARMUP,
LED_COMMAND,
LED_ZEROCAL,
LED_ZEROCAL_END,
LED_FAIL,
LED_ALLOFF,
LED_NOTHING,
} e_Led_Seq;

typedef struct {
uint8_t seq;
uint8_t last_state;

bool isBlink;
bool toggle;
bool twoYear;
} s_Led_Status;

# 33
void led_init(void);
void led_task(void);

# 7 "app_sens_calibration.h"
typedef enum {
CALIB_INIT,
CALIB_WAIT,
CALIB_ZERO
} e_Calib_Seq;

typedef struct {
uint8_t seq;
bool zero_cal;
} s_Sens_Calib;

# 22
void sens_ZeroCalib_init(void);
void sens_ZeroCalib_task(void);

# 23 "app_sens_calibration.c"
extern uint8_t g_usb_state;
extern s_Nvm g_nvm;
extern s_Sens_Calib g_sens_calib;
extern s_Led_Status g_led_status;
extern s_Sens_Measure_value Last_value;

volatile uint32_t g_calib_count;



static bool sens_ZeroCalibration(void);


void sens_ZeroCalib_init(void)
{
g_sens_calib.seq = CALIB_INIT;
g_sens_calib.zero_cal = 0;
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
if ((g_usb_state == USB_DETACHED) && (g_sens_calib.zero_cal == 0)) {
switch (g_sens_calib.seq)
{
case CALIB_INIT:
LED_Off();
LED_On(LED_GREEN);
g_calib_count = 0;
g_sens_calib.seq = CALIB_WAIT;
break;
case CALIB_WAIT:
if (g_calib_count > 30000) {
g_calib_count = 0;
g_sens_calib.seq = CALIB_ZERO;
} break;
case CALIB_ZERO:
if (sens_ZeroCalibration() == 0)
g_sens_calib.seq = CALIB_ZERO;
else g_led_status.seq = LED_ZEROCAL_END;
break;
}
} else {
g_calib_count = 0;
return;
}
}
