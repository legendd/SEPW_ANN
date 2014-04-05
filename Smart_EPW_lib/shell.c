/*=============================================================================
 *
 * @file     : shell.c
 * @author        : JackABK
 * @data       : 2014/2/2
 * @brief   :  trasfer the message from other device and could be achieve interactive effect.
 *
 *============================================================================*/

#include "FreeRTOS.h"
#include "stm32f4xx_usart.h"
#include "uart.h"
#include "shell.h"
#include "EPW_command.h"


extern rpm_left_motor,rpm_right_motor;

/*============================================================================*
 ** Prototype    : receive_task
 ** Description  : receive the command from the other device, and send to relative info.
 ** Input          : void *p  
 ** Output       : None
 ** Return Value : 
 *============================================================================*/
void receive_task(void *p)
{
	    int i , j;
		struct  receive_cmd_list * receive_cmd_type;
        
		while (1) {
				if(Receive_String_Ready ){
						//GPIO_ToggleBits(GPIOD,GPIO_Pin_14);

                        /*load the accept command string to the command list structure*/
                        receive_cmd_type = received_string;

                        /*identifier the command's format, if yes, analyze the command list and perform it. */
                        if(receive_cmd_type->Identifier[0] =='c' && receive_cmd_type->Identifier[1] =='m' && receive_cmd_type->Identifier[2] =='d'){
                            PerformCommand(receive_cmd_type->group,receive_cmd_type->control_id, receive_cmd_type->value);
                            
                        }

                        
						/*clear the received string and the flag*/
						Receive_String_Ready = 0;
						for( i = 0 ; i< MAX_STRLEN ; i++){
								received_string[i]= 0;
						}
				} 

		}
}

void send_out_task(void *p){
    cmd_group EPW_cmd_group = SEND_EPW_INFO;
    EPW_Info_issue send_out_EPW_info_id = EPW_ULTRASONIC_0; /*first member of info.*/
    int i;



    /**
     * send command list format to Android used 
     * Note, because transfer data's unit is byte, so I convert to char,
     * but the java accept byte have a sign bit problem,
     * on the java, byte is represent  -128~127
     */

    printf("cmd%c%c%cend",EPW_cmd_group,send_out_EPW_info_id++,(unsigned char)Get_CH1Distance() );
    printf("cmd%c%c%cend",EPW_cmd_group,send_out_EPW_info_id++,(unsigned char)Get_CH2Distance() );
    printf("cmd%c%c%cend",EPW_cmd_group,send_out_EPW_info_id++,(unsigned char)Get_CH3Distance() );
    printf("cmd%c%c%cend",EPW_cmd_group,send_out_EPW_info_id++,(unsigned char)Get_CH4Distance() );
    //printf("cmd%c%c%cend",EPW_cmd_group,send_out_EPW_info_id++, ); /*the actuatorA of limit switch state*/
    //printf("cmd%c%c%cend",EPW_cmd_group,send_out_EPW_info_id++, ); /*the actuatorB of limit switch state*/
    printf("cmd%c%c%cend",EPW_cmd_group,send_out_EPW_info_id++, (unsigned char)rpm_left_motor);
    printf("cmd%c%c%cend",EPW_cmd_group,send_out_EPW_info_id, (unsigned char)rpm_right_motor);
        

    
#ifdef OUTPUT_EPW_INFO          
                printf("-------------------EPW Info----------------------\r\n");
                printf("SpeedValue PWM-->   Left = %d   Right = %d\r\n" , SpeedValue_left,SpeedValue_right  );
                printf("RPM-->   Left = %d   Right = %d\r\n" , round(rpm_left_motor) , round(rpm_right_motor));
                printf("Encoder-->   Left = %d   Right =  %d\r\n" , encoder_left_counter , encoder_right_counter);      
                /*because freertos of printf cannot be used %f , so I scale 10 factor to convert int.*/
                printf("Kp = %d Ki = %d Kd = %d\r\n" ,(int)(PID_Motor_R.Kp*10.0f) , (int)(PID_Motor_R.Ki*10.0f), (int)(PID_Motor_R.Kd*10.0f));
#endif     
}

