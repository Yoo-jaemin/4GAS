
# 1 "app_tes0902_task.c"

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

# 15 "C:\Program Files\Microchip\xc8\v2.32\pic\include\c90\stdbool.h"
typedef unsigned char bool;

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

# 14 "C:\Program Files\Microchip\xc8\v2.32\pic\include\c90\string.h"
extern void * memcpy(void *, const void *, size_t);
extern void * memmove(void *, const void *, size_t);
extern void * memset(void *, int, size_t);

# 36
extern char * strcat(char *, const char *);
extern char * strcpy(char *, const char *);
extern char * strncat(char *, const char *, size_t);
extern char * strncpy(char *, const char *, size_t);
extern char * strdup(const char *);
extern char * strtok(char *, const char *);


extern int memcmp(const void *, const void *, size_t);
extern int strcmp(const char *, const char *);
extern int stricmp(const char *, const char *);
extern int strncmp(const char *, const char *, size_t);
extern int strnicmp(const char *, const char *, size_t);
extern void * memchr(const void *, int, size_t);
extern size_t strcspn(const char *, const char *);
extern char * strpbrk(const char *, const char *);
extern size_t strspn(const char *, const char *);
extern char * strstr(const char *, const char *);
extern char * stristr(const char *, const char *);
extern char * strerror(int);
extern size_t strlen(const char *);
extern char * strchr(const char *, int);
extern char * strichr(const char *, int);
extern char * strrchr(const char *, int);
extern char * strrichr(const char *, int);

# 6 "util/buffer.h"
typedef struct buff_queue
{
uint8_t size;
int8_t read_pos;
int8_t write_pos;
uint8_t buffer[64];
} s_Queue;

# 18
void buffer_init(s_Queue* queue);
void buffer_en_queue(s_Queue* queue, uint8_t ch);
uint8_t buffer_de_queue(s_Queue* queue);
bool buffer_de_queue_bool(s_Queue* queue, uint8_t* ch);

# 10 "util/_string.h"
unsigned _strcpy(char* dest, const char* source);
void _strncpy(char *_to, const char *_from, int _n);
unsigned _strlen(const char* source);
void _strcat(char* dest, const char* source);

unsigned char _strncmp(const char* src1, const char* src2, int n, int i);
void _strclr(char* dest, int n);

unsigned char _isdigit(const char* src, int n);

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

# 29 "app_tes0902_task.c"
extern s_KalmanFilter_t kalmanFilter[4];


extern s_Nvm g_nvm;
extern s_Queue g_usart_queue;

extern s_Tes0902_frame g_rDataFormat;
extern s_Tes0902_Measure g_tes0902_measure;
extern s_Sens_Measure g_sens_measure;
extern s_Sens_Measure_value sens_value;



static bool tes0902_data_receive(void);
static bool tes0902_data_parsing(uint8_t cmd);



void Co2_measure_task(void)
{
uint8_t rev_data;
switch (g_tes0902_measure.seq)
{
case SYNC_MSB_CHECK:
if (buffer_de_queue_bool(&g_usart_queue, &rev_data) == 0)
break;
if (rev_data == 0xBB) {

g_rDataFormat.sync[0] = 0xBB;
g_tes0902_measure.seq = SYNC_LSB_CHECK;
} else g_tes0902_measure.seq = SYNC_MSB_CHECK;
break;
case SYNC_LSB_CHECK:
if (buffer_de_queue_bool(&g_usart_queue, &rev_data) == 0)
break;
if (rev_data == 0x66) {

g_rDataFormat.sync[1] = 0x66;
g_tes0902_measure.seq = CMD_CHECK;
} else g_tes0902_measure.seq = SYNC_MSB_CHECK;
break;
case CMD_CHECK:
if (buffer_de_queue_bool(&g_usart_queue, &rev_data) == 0)
break;
if (rev_data == 0x15 || rev_data == 0x4D || rev_data == 0x23) {

g_rDataFormat.cmd = rev_data;
g_tes0902_measure.seq = LEN_CHECK;
} else g_tes0902_measure.seq = SYNC_MSB_CHECK;
break;
case LEN_CHECK:
if (buffer_de_queue_bool(&g_usart_queue, &rev_data) == 0)
break;
g_rDataFormat.len = rev_data;
g_tes0902_measure.seq = DATA_RECV;
break;
case DATA_RECV:
if (tes0902_data_receive())
g_tes0902_measure.seq = DATA_PARSING;
else g_tes0902_measure.seq = SYNC_MSB_CHECK;
break;
case DATA_PARSING:
g_sens_measure.isCO2_Success = tes0902_data_parsing(g_rDataFormat.cmd);
g_tes0902_measure.seq = SYNC_MSB_CHECK;
break;
}
}

static bool tes0902_data_receive(void)
{
uint8_t data_len = 0;
uint8_t crc_len = 0;
uint8_t rev_data[8];
uint8_t rev_crc[2];

while (data_len != g_rDataFormat.len)
{
if (buffer_de_queue_bool(&g_usart_queue, &rev_data[data_len])) {
g_rDataFormat.data[data_len] = rev_data[data_len];
data_len++;
}
}

while (crc_len != 2)
{
if (buffer_de_queue_bool(&g_usart_queue, &rev_crc[crc_len])) {
g_rDataFormat.crc[crc_len] = rev_crc[crc_len];
crc_len++;
}
}
return 1;
}

static bool tes0902_data_parsing(uint8_t cmd)
{
if (CRC_check() == 0) return 0;

switch (cmd)
{
case 0x15:
sens_value.co2_gas = g_rDataFormat.data[1] *256 + g_rDataFormat.data[0];

# 136
sens_value.co2_gas = Kalman_updateEstimate(&kalmanFilter[3], sens_value.co2_gas);

sens_value.co2_gas += g_nvm.flash.co2Offset;
break;

case 0x4D:
case 0x23:
break;

default: break;
}
return 1;
}

