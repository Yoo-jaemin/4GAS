/*
 * File:   window_filter.c
 * Author: brian
 *
 * Created on 2022 1/4
 */

#include <stdbool.h>
#include <stdint.h>

#include "window_filter.h"

float g_moving_buffer[WINDOW_MOVING_BUFFER_MAX_SIZE] = {0.0f, };
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
	float average, alpha; // alpha: constant
	g_Num_of_sample += 1;
	alpha = (float)((g_Num_of_sample -1) / g_Num_of_sample);
	average = alpha * prev_average + (1- alpha)*f_data;
	prev_average = average;
	return average;
}