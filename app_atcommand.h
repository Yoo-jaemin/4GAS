
#ifndef _APP_AT_COMMAND_H_
#define _APP_AT_COMMAND_H_

typedef enum {
    ATNONE,
	ATCZ,
    ATCVER,
    ATCRST, // h/w reset
#ifdef SUPPORT_BOOLTLOADER
    ATCBLRST, // bootloader reset
#endif

    ATCD, 
    ATCQ, 
    ATCH,  
    ATCUV, 
    ATCMODEL,
    
    ATCSMODEL, 
    ATCCO2ABC,
    ATCC,
    ATCF,
    ATCCAL,     //zero uV, span uV, cal ppm
    ATCZCAL,
    ATCCZRA,    //set All baseline!
    ATCCZR,     //set baseline!
    ATCCSP,     
    ATCOFF,
    ATCMODE,
    ATCTCMP,
    
    // kalman filter parameter set        
    ATCWIN,
    ATCKMFEM,        
            

#ifdef USE_DEEPSLEEP
    ATCDSEN,
#endif
    ATCUSED,
    ATCEXTM,
            
            
    ATCBLDTM
} e_AtCommand;

typedef enum {
    AT_READONLY,
    AT_APPLYONLY,
    AT_READAPPLY,
    AT_STRAPPLY
} e_CommandType;

typedef enum {
    AT_INIT,
    AT_START,
    AT_PARSE,
    AT_APPLY,
    AT_MESSAGE,
    AT_END
} e_CommandSeq;

typedef struct {
    uint8_t channelno; 
    bool last_update;      //write result flag
    e_AtCommand atcommand; //atcommand enumeration value
    char szcommand[11]; 

    bool isApply;          //write flag
    char apply_params[32]; //command parameter buffer
    char manual_params[4][32]; //command multi parameter buffer
    
    e_CommandType type; 
} s_AtCommand_status;


typedef struct {
    e_CommandSeq seq;   //atcommand state
    char read_cmd[64];  //read buffer
    uint8_t readidx;    
    s_AtCommand_status status; //atcommand result
    
} s_AtCommand;


#ifdef __cplusplus
extern "C" {
#endif

    void atcommand_init(void);
    void atcommand_task(void);
        
#ifdef __cplusplus
}
#endif

#endif // _APP_AT_COMMAND_H_
