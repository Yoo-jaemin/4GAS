#ifndef WINDOW_FILTER_H
#define	WINDOW_FILTER_H

#define WINDOW_MOVING_BUFFER_MAX_SIZE  12



#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

  float window_moving_average_recursion(float fdata, uint8_t window_size);
  float Average_filter(float f_data); 
  
#ifdef	__cplusplus
}
#endif 

#endif	

