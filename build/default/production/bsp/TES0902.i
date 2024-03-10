
# 1 "bsp/TES0902.c"

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

# 6 ".\util/buffer.h"
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

# 10 ".\util/_string.h"
unsigned _strcpy(char* dest, const char* source);
void _strncpy(char *_to, const char *_from, int _n);
unsigned _strlen(const char* source);
void _strcat(char* dest, const char* source);

unsigned char _strncmp(const char* src1, const char* src2, int n, int i);
void _strclr(char* dest, int n);

unsigned char _isdigit(const char* src, int n);

# 32 "bsp/usart.h"
void usart_init(uint32_t baudrate);
bool usart_tx_empty(void);
void usart_putc(uint8_t);
bool usart_rx_ready(void);
uint8_t usart_getc(void);

void usart_writeBytes(const uint8_t* wrptr, const uint8_t len);
bool usart_readByte(uint8_t* byte);
bool usart_readChar(char* ch);


void uart_getBuffer(void);

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

# 46 ".\board.h"
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

# 20 "bsp/TES0902.c"
s_Tes0902_frame g_rDataFormat;
uint8_t g_Wbuff[8];
u_2byte_Conv convData;

uint8_t b[64];


void CMD_get_ppm(void)
{
uint8_t len = 0;
g_Wbuff[len++] = 0xAA;
g_Wbuff[len++] = 0x55;
g_Wbuff[len++] = 0x14;
g_Wbuff[len++] = 0x00;
convData.u16_data = Calculate_CRC16(g_Wbuff, len);
g_Wbuff[len++] = convData.u8_Data[0];
g_Wbuff[len++] = convData.u8_Data[1];

usart_writeBytes(g_Wbuff, len);


}

void CMD_manual_cal(void)
{
uint8_t len = 0;
g_Wbuff[len++] = 0xAA;
g_Wbuff[len++] = 0x55;
g_Wbuff[len++] = 0x4C;
g_Wbuff[len++] = 0x00;
convData.u16_data = Calculate_CRC16(g_Wbuff, len);
g_Wbuff[len++] = convData.u8_Data[0];
g_Wbuff[len++] = convData.u8_Data[1];

usart_writeBytes(g_Wbuff, len);


}

void CMD_set_ABC_on(uint8_t value)
{
uint8_t len = 0;
g_Wbuff[len++] = 0xAA;
g_Wbuff[len++] = 0x55;
g_Wbuff[len++] = 0x22;
g_Wbuff[len++] = 0x02;
g_Wbuff[len++] = 0x00;
g_Wbuff[len++] = value;
convData.u16_data = Calculate_CRC16(g_Wbuff, len);
g_Wbuff[len++] = convData.u8_Data[0];
g_Wbuff[len++] = convData.u8_Data[1];
usart_writeBytes(g_Wbuff, len);



}

void CMD_get_version(void)
{
uint8_t len = 0;
g_Wbuff[len++] = 0xAA;
g_Wbuff[len++] = 0x55;
g_Wbuff[len++] = 0x10;
g_Wbuff[len++] = 0x00;
convData.u16_data = Calculate_CRC16(g_Wbuff, len);
g_Wbuff[len++] = convData.u8_Data[0];
g_Wbuff[len++] = convData.u8_Data[1];

usart_writeBytes(g_Wbuff, len);


}

void CMD_get_serialNumber(void)
{
uint8_t len = 0;
g_Wbuff[len++] = 0xAA;
g_Wbuff[len++] = 0x55;
g_Wbuff[len++] = 0x12;
g_Wbuff[len++] = 0x00;
convData.u16_data = Calculate_CRC16(g_Wbuff, len);
g_Wbuff[len++] = convData.u8_Data[0];
g_Wbuff[len++] = convData.u8_Data[1];

usart_writeBytes(g_Wbuff, len);


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
uint8_t crc[8 +4];
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
return 1;
return 0;
}

void tes0902_polling_mode(void)
{
CMD_get_ppm();
}

void tes0902_ABC_OFF(uint8_t status)
{
CMD_set_ABC_on(status);
}
