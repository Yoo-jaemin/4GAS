
# 1 "app_sens_measure.c"

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

# 4 "C:/Program Files/Microchip/MPLABX/v5.50/packs/Microchip/PIC18F-J_DFP/1.4.41/xc8\pic\include\__size_t.h"
typedef unsigned size_t;

# 7 "C:\Program Files\Microchip\xc8\v2.32\pic\include\c90\stdarg.h"
typedef void * va_list[1];

#pragma intrinsic(__va_start)
extern void * __va_start(void);

#pragma intrinsic(__va_arg)
extern void * __va_arg(void *, ...);

# 43 "C:\Program Files\Microchip\xc8\v2.32\pic\include\c90\stdio.h"
struct __prbuf
{
char * ptr;
void (* func)(char);
};

# 29 "C:\Program Files\Microchip\xc8\v2.32\pic\include\c90\errno.h"
extern int errno;

# 12 "C:\Program Files\Microchip\xc8\v2.32\pic\include\c90\conio.h"
extern void init_uart(void);

extern char getch(void);
extern char getche(void);
extern void putch(char);
extern void ungetch(char);

extern __bit kbhit(void);

# 23
extern char * cgets(char *);
extern void cputs(const char *);

# 88 "C:\Program Files\Microchip\xc8\v2.32\pic\include\c90\stdio.h"
extern int cprintf(char *, ...);
#pragma printf_check(cprintf)



extern int _doprnt(struct __prbuf *, const register char *, register va_list);


# 180
#pragma printf_check(vprintf) const
#pragma printf_check(vsprintf) const

extern char * gets(char *);
extern int puts(const char *);
extern int scanf(const char *, ...) __attribute__((unsupported("scanf() is not supported by this compiler")));
extern int sscanf(const char *, const char *, ...) __attribute__((unsupported("sscanf() is not supported by this compiler")));
extern int vprintf(const char *, va_list) __attribute__((unsupported("vprintf() is not supported by this compiler")));
extern int vsprintf(char *, const char *, va_list) __attribute__((unsupported("vsprintf() is not supported by this compiler")));
extern int vscanf(const char *, va_list ap) __attribute__((unsupported("vscanf() is not supported by this compiler")));
extern int vsscanf(const char *, const char *, va_list) __attribute__((unsupported("vsscanf() is not supported by this compiler")));

#pragma printf_check(printf) const
#pragma printf_check(sprintf) const
extern int sprintf(char *, const char *, ...);
extern int printf(const char *, ...);

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

# 43 "bsp/ads1114.h"
typedef enum {
FSR_6p144V,
FSR_4p096V,
FSR_2p048V,
FSR_1p024V,
FSR_0p512V,
FSR_0p256V
} e_ADC_Gain;


typedef enum {
CONTINUE_CONV,
SINGLE_CONV
} e_ADC_Mode;


typedef enum {
SPS8,
SPS16,
SPS32,
SPS64,
SPS128,
SPS250,
SPS475,
SPS860
} e_ADC_DataRate;



typedef enum {
COMP,
WIN_COMP
} e_Comparator_Mode;


typedef enum {
LO,
HI
} e_Comparator_Pol;


typedef enum {
NONE_LATCH,
LATCH
} e_Comparator_Latch;


typedef enum {
ONE_CONV,
TWO_CONV,
FOUR_CONV,
DISABL
} e_Comparator_Que;

# 101
bool ads1114_read_ready(uint8_t chNum);
bool ads1115_SetUp(uint8_t chNum, uint8_t mode, uint8_t pga, uint8_t dataRate);
bool ads1114_read(uint8_t chNum, int16_t* raw_adc);

bool ads1114_Comparator_SetUp(uint8_t mode, uint8_t polar, uint8_t latch, uint8_t que);

# 25 "bsp/lmp91000.h"
typedef enum
{
EN_CH0,
EN_CH1,
EN_CH2
} LMP_EN;


typedef enum {
EXTERNAL_REGISTOR,
R2p75K,
R3p5K,
R7K,
R14K,
R35K,
R120K,
R350K
} e_TIA_InternalGain;


typedef enum {
R10,
R33,
R50,
R100
} e_TIA_RLoad;


typedef enum {
INTERNAL,
EXTERNAL
} e_Ref_Voltage;


typedef enum {
S20P,
S50P,
S67P,
BYPASS
} e_InZ_sel;


typedef enum {
NAGATIVE,
POSITIVE
} e_Bias_sign;


typedef enum {
B0P,
B1P,
B2P,
B4P,
B6P,
B8P,
B10P,
B12P,
B14P,
B16P,
B18P,
B20P,
B22P,
B24P
} e_Bias_sel;


typedef enum {
DISABLE,
ENABLE
} e_FET;


typedef enum {
SLEEP,
GALVANIC_2LEAD,
STANBY,
AMPEROETRIC_3LEAD,
TEMP_MEASURE_TIAOFF = 6,
TEMP_MEASURE_TIAON = 7
} e_OP_Mode;

# 112
bool lmp91000_lock(uint8_t chNum);
bool lmp91000_unlock(uint8_t chNum);
bool lmp91000_isUnLocked(void);

bool lmp91000_isReady(uint8_t chNum);
bool lmp91000_get_status(uint8_t* preg, uint8_t chNum);


bool lmp91000_get_lock(uint8_t* preg);
bool lmp91000_get_Tiacn(uint8_t* preg);
bool lmp91000_get_Refcn(uint8_t* preg);
bool lmp91000_get_Modecn(uint8_t* preg);


bool lmp91000_set_GainRLoad(uint8_t chNum, uint8_t user_gain, uint8_t RLoad);
bool lmp91000_set_Reference(uint8_t chNum, uint8_t source, uint8_t IntZ, uint8_t sign, uint8_t bias);
bool lmp91000_set_Mode(uint8_t chNum, uint8_t fet, uint8_t mode);

void lmp91000_pin_init(void);
void lmp91000_enable(LMP_EN chNum);
void lmp91000_disable(LMP_EN chNum);

# 79 "bsp/sht3x.h"
bool sht3x_softreset(void);
bool sht3x_measure_start(void);

bool sht3x_art_cmd(void);
bool sht3x_break_cmd(void);

bool sht3x_clear_status_reg(void);
bool sht3x_read_status_reg(uint16_t* status);
bool sht3x_heaterOn(void);
bool sht3x_heaterOff(void);

bool sht3x_write_alert_limits(float humidityHighSet, float temperatureHighSet,
float humidityHighClear, float temperatureHighClear,
float humidityLowClear, float temperatureLowClear,
float humidityLowSet, float temperatureLowSet);

bool sht3x_common_read(int32_t* temp, int32_t* humi);

# 101
bool sht3x_measure_read(float* temp, float* humi);
bool sht3x_measure_nonblock_read(float* temp, float* humi);
bool sht3x_measure_block_read(float* temp, float* humi);

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

# 16 "filter.h"
typedef struct
{
float _err_measure;
float _err_estimate;

float _q_process;

float _current_estimate;
float _last_estimate;
float _kalman_gain;
} s_KalmanFilter_t;


typedef enum {
LEVEL_1 = 1,
LEVEL_2,
LEVEL_3,
LEVEL_4,
LEVEL_5,
LEVEL_6,
LEVEL_7,
LEVEL_8,
LEVEL_9,
LEVEL_10,
LEVEL_11,
LEVEL_12,
LEVEL_13,
LEVEL_14,
LEVEL_15
} e_KalmaFilter_sensitivity;

# 56
float Kalman_updateEstimate(s_KalmanFilter_t *pObj,float mea);
void Kalman_setMeasurementError(s_KalmanFilter_t *pObj,float mea_e);
void Kalman_setEstimateError(s_KalmanFilter_t *pObj,float est_e);
void Kalman_setProcessNoise(s_KalmanFilter_t *pObj,float q);

float Kalman_getKalmanGain(s_KalmanFilter_t *pObj);
float Kalman_getEstimateError(s_KalmanFilter_t *pObj);

void Set_KalmanFilter_Sensitivity(s_KalmanFilter_t *pObj, e_KalmaFilter_sensitivity e_level);

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

# 8 "app_tes0902_task.h"
enum e_Tes0902_seq {
SYNC_MSB_CHECK,
SYNC_LSB_CHECK,
CMD_CHECK,
LEN_CHECK,
DATA_RECV,
DATA_PARSING
};

typedef struct {
uint8_t seq;

} s_Tes0902_Measure;

# 27
void co2Measure_task(void);
void Co2_measure_task(void);

# 21 "app_sens_measure.c"
extern s_Sens_Measure_value sens_value;
extern uint8_t writeBuffer[64];
extern s_Nvm g_nvm;
extern s_Sens_Measure g_sens_measure;
extern s_Tes0902_Measure g_tes0902_measure;
extern float gTemp_volt;



extern s_KalmanFilter_t kalmanFilter[4];


volatile uint32_t g_large_counter;

s_Sens_Measure_value Last_value;
uint8_t gChNum;


void sensMeasure_init(void)
{
g_large_counter = 0;
g_sens_measure.seq = SENS_INIT;
g_tes0902_measure.seq = SYNC_MSB_CHECK;

tes0902_ABC_OFF(g_nvm.flash.CO2_ABC);
g_sens_measure.isLMP_InitDone[0] = 0;
g_sens_measure.isLMP_InitDone[1] = 0;
g_sens_measure.isADC_InitDone[0] = 0;
g_sens_measure.isADC_InitDone[1] = 0;
g_sens_measure.isTemp_InitDone = 0;

g_sens_measure.isGasSensor_Success[0] = 0;
g_sens_measure.isGasSensor_Success[1] = 0;
g_sens_measure.isGasSensor_Success[2] = 0;
g_sens_measure.isCO2_Success = 0;

# 61
for (uint8_t chNum = 0; chNum < 4; chNum++)
{
kalmanFilter[chNum]._current_estimate = 0.0f;
kalmanFilter[chNum]._err_estimate = 0.0f;
kalmanFilter[chNum]._err_measure = 0.0f;
kalmanFilter[chNum]._kalman_gain = 0.0f;
kalmanFilter[chNum]._last_estimate = 0.0f;
kalmanFilter[chNum]._q_process = 0.0f;
Kalman_setMeasurementError(&kalmanFilter[chNum],1);
Kalman_setEstimateError(&kalmanFilter[chNum],1);
Set_KalmanFilter_Sensitivity(&kalmanFilter[chNum], g_nvm.flash.window_size[chNum]);
}

Kalman_setMeasurementError(&kalmanFilter[1], g_nvm.flash.KMf_e_measure);

}

void sensMeasure_task(void)
{
switch (g_sens_measure.seq)
{
case SENS_INIT:
Sensors_initialize();
g_large_counter = 0;
g_sens_measure.seq = SENS_READY;
break;
case SENS_READY:
if (g_large_counter > 20) {
g_large_counter = 0;
g_sens_measure.seq = SENS_TEMP_MODE;
} break;
case SENS_TEMP_MODE:
sensor_temp_mode(GAS_CH2);
g_large_counter = 0;
g_sens_measure.seq = SENS_WAIT;
g_sens_measure.next_seq = SENS_TEMP_MEASURE;
g_sens_measure.seq_count = 20;
break;
case SENS_TEMP_MEASURE:
g_sens_measure.isTemp_Success = tempSensor_read();
CMP_temp_read(GAS_CH2);
g_large_counter = 0;
g_sens_measure.seq = SENS_WAIT;
g_sens_measure.next_seq = SENS_GAS_MODE;
g_sens_measure.seq_count = 10;
break;
case SENS_GAS_MODE:
sensor_gas_mode(GAS_CH2);
g_large_counter = 0;
g_sens_measure.seq = SENS_WAIT;
g_sens_measure.next_seq = SENS_GAS_MODE_MEASURE;
g_sens_measure.seq_count = 20;
break;
case SENS_GAS_MODE_MEASURE:
g_sens_measure.isGasSensor_Success[GAS_CH2] = gasSensor_read(GAS_CH2);
g_sens_measure.seq = SENS_GAS_INIT;
break;
case SENS_GAS_INIT:
gas_init(gChNum);
g_large_counter = 0;
g_sens_measure.seq = SENS_WAIT;
g_sens_measure.next_seq = SENS_GAS_MEASURE;
g_sens_measure.seq_count = 20;
break;
case SENS_GAS_MEASURE:
g_sens_measure.isGasSensor_Success[gChNum] = gasSensor_read(gChNum);
gChNum++;
g_large_counter = 0;
g_sens_measure.seq = SENS_WAIT;
g_sens_measure.next_seq = SENS_GAS_INIT;
g_sens_measure.seq_count = 10;
break;
case SENS_MEASURE_CHECK:
if (g_sens_measure.isGasSensor_Success[0] && g_sens_measure.isGasSensor_Success[1] && g_sens_measure.isGasSensor_Success[2])
g_sens_measure.seq = SENS_MEASURE_APPLY;
else g_sens_measure.seq = SENS_INIT;
break;
case SENS_MEASURE_APPLY:
Last_value.toxic_gas[0][LV_GAS_VOLT] = sens_value.toxic_gas[0][LV_GAS_VOLT];
Last_value.toxic_gas[1][LV_GAS_VOLT] = sens_value.toxic_gas[1][LV_GAS_VOLT];
Last_value.toxic_gas[2][LV_GAS_VOLT] = sens_value.toxic_gas[2][LV_GAS_VOLT];
Last_value.toxic_gas[0][LV_GAS_CONCEN] = sens_value.toxic_gas[0][LV_GAS_CONCEN];
Last_value.toxic_gas[1][LV_GAS_CONCEN] = sens_value.toxic_gas[1][LV_GAS_CONCEN];
Last_value.toxic_gas[2][LV_GAS_CONCEN] = sens_value.toxic_gas[2][LV_GAS_CONCEN];
if (g_sens_measure.isCO2_Success)
Last_value.co2_gas = sens_value.co2_gas;
Last_value.tempHumi[LV_TEMP] = sens_value.tempHumi[LV_TEMP];
Last_value.tempHumi[LV_HUMI] = sens_value.tempHumi[LV_HUMI];
Last_value.tempHumi[CMP_TEMP] = sens_value.tempHumi[CMP_TEMP];
g_sens_measure.seq = SENS_DISPLAY;
break;
case SENS_DISPLAY:



g_large_counter = 0;
g_sens_measure.seq = SENS_READY;
break;
case SENS_WAIT:
if (gChNum == 2) {
gChNum = 0;
g_sens_measure.seq = SENS_MEASURE_CHECK;
} else {
if (g_large_counter > g_sens_measure.seq_count) {
g_large_counter = 0;
g_sens_measure.seq = g_sens_measure.next_seq;
}
} break;
}
}
