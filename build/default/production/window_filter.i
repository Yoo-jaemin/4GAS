
# 1 "window_filter.c"

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

# 12 "window_filter.h"
float window_moving_average_recursion(float fdata, uint8_t window_size);
float Average_filter(float f_data);

# 13 "window_filter.c"
float g_moving_buffer[12] = {0.0f, };
uint16_t g_Num_of_sample;
float prev_average;

float window_moving_average_recursion(float fdata, uint8_t window_size)
{
uint8_t idx;
static float avg = 0.0f;

uint8_t size = window_size;

avg = avg + (fdata - g_moving_buffer[0]) / size;

for (idx = 0; idx < size -1; idx++)
g_moving_buffer[idx] = g_moving_buffer[idx +1];

g_moving_buffer[idx] = fdata;

return avg;
}

float Average_filter(float f_data)
{
float average, alpha;
g_Num_of_sample += 1;
alpha = (float)((g_Num_of_sample -1) / g_Num_of_sample);
average = alpha * prev_average + (1- alpha)*f_data;
prev_average = average;
return average;
}
