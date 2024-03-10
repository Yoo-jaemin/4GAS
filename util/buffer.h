#ifndef _BUFFER_H
#define _BUFFER_H

#define MAX_QUEUE_SIZE	64

typedef struct buff_queue
{
	uint8_t size;
	int8_t read_pos;
	int8_t write_pos;
    uint8_t buffer[MAX_QUEUE_SIZE];
} s_Queue;

#ifdef __cplusplus
extern "C" {
#endif

    void buffer_init(s_Queue* queue);
    void buffer_en_queue(s_Queue* queue, uint8_t ch);
    uint8_t buffer_de_queue(s_Queue* queue); 
    bool buffer_de_queue_bool(s_Queue* queue, uint8_t* ch); 
         
    
#ifdef __cplusplus
}
#endif

#endif //_BUFFER_H
