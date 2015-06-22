#include "FreeRTOS.h"
#include "timers.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_syscfg.h"
#include "EPW_behavior.h"
#include "ultrasound.h"
#include "timers.h"
#include "uart.h"
#include "clib.h"
#include "EPW_command.h"
#include "PID.h"

#define B_phase 1
#define NEURAL_IDENTIFIER 1

int encoder_left_counter_1;
int encoder_right_counter_1;
int differential_counter = 0;
int first_control = 0;
int base_pwm_l = 120;
int base_pwm_r = 120;

float move_distance = 15.0;

neural_state_t n_r;
neural_state_t n_l;
int err_sum = 0;

// input command
float rin = 0.0f;

float referenceModel(float rin, float rf_out_1);
float referenceModel2(float rin, float rf_out_1, float rf_out_2);

void array_1d_Init(int size, float value, float *array){
	int i;
	float tmp = value / (float)size;
    for (i = 0; i < size; ++i){ 
    	array[i] = -value + tmp * (i * 2);
    }
}

void array_1d_Init_2(int size, float value, float *array){
	int i;
    for (i = 0; i < size; ++i) array[i] = value;
}

// array[x][y]  array[i][j]
void array_2d_Init(int size_x, int size_y, float value, float array[][size_y]){
    int i,j;
    for ( i = 0; i < size_x; ++i){
    	float tmp = value / (float)size_x;
        for ( j = 0; j < size_y; ++j){
            array[i][j] = tmp*i;
        }
    }
}

// array2 = array1
void array_1d_Copy(int size, float *array1, float *array2){
    int i;
    for ( i = 0; i < size; ++i)
        array2[i] = array1[i];
}

// array2 = array1
void array_2d_Copy(int size_x, int size_y, float array1[][size_y], float array2[][size_y]){
    int i, j;
    for ( i = 0; i < size_x; ++i)
        for ( j = 0; j < size_y; ++j)
            array2[i][j] = array1[i][j];
}

float referenceModel2(float rin, float rf_out_1, float rf_out_2){
    return ((25 * rin - rf_out_2 - 7 * rf_out_1)/25);
}

typedef enum{
		CAR_STATE_IDLE,
		CAR_STATE_REST,                               
		CAR_STATE_MOVE_FORWARD,
		CAR_STATE_MOVE_BACK,
		CAR_STATE_MOVE_LEFT,
		CAR_STATE_MOVE_RIGHT
}car_state_t;

static car_state_t car_state;

int encoder_right_counter;
int encoder_left_counter;

int count_l = 0;
int count_r = 0;

/*pwm regulate of two motor */
static int pwm_value_left = 120; /*default pwm value*/
static int pwm_value_right = 120;
static int motor_speed_value = 1; /*global speed value, range is 0~10.
static char flag;

/*pid alg premeter.*/
static float Kp,Ki,Kd;

/*Timer handle declare*/
xTimerHandle carTimers;
xTimerHandle PID_Timers;
xTimerHandle Show_data_Timers;
xTimerHandle Get_Motor_Timers;

/*============================================================================*/
/*============================================================================*
 ** function : init_motor
 ** brief :  initialize the motor control pin set, the AF is  setting by pwm mode based on timer.
 ** param :  None
 ** retval : None
 **============================================================================*/
/*============================================================================*/
void init_motor(void){
		GPIO_InitTypeDef GPIO_InitStruct;
		/* Enable GPIO A clock. */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		// Setup Blue & Green LED on STM32-Discovery Board to use PWM.
		GPIO_InitStruct.GPIO_Pin =  MOTOR_LEFT_PWM_PIN|MOTOR_RIGHT_PWM_PIN;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init( MOTOR_PWM_PORT, &GPIO_InitStruct );   

		/*====================TIM Setting=============================*/
		GPIO_PinAFConfig(MOTOR_PWM_PORT, GPIO_PinSource13, GPIO_AF_TIM4);
		GPIO_PinAFConfig(MOTOR_PWM_PORT, GPIO_PinSource15, GPIO_AF_TIM4);

		/* TIM4 clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

		/**
		 * Compute the prescaler value
		 * old version , 84MHz / 2000 / 42  = 1KHz
		 * --> u32 PrescalerValue = 42 - 1; 
		 * --> u32 TimPeriod = 2000 - 1;
		 *
		 * new version is setting for pwm 8 bit resolution , 84MHz / 256 / 250  ~= 1312.5 Hz
		 */   
		u32 PrescalerValue = 250 - 1; /*YinChen added*/
		u32 TimPeriod = 256 - 1;

		/* Time base configuration */
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_TimeBaseStructure.TIM_Period = TimPeriod;     
		TIM_TimeBaseStructure.TIM_Prescaler =PrescalerValue ; 
		TIM_TimeBaseStructure.TIM_ClockDivision = 0 ;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

		/*====================PWM Setting=============================*/
		TIM_OCInitTypeDef TIM_OCInitStructure;
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 0; /*max pwm value is TIM's period, in our case, it's  255*/
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		/* PWM1 Mode configuration: Channel2   (MOTOR_LEFT_PWM_PIN)*/
		TIM_OC2Init(TIM4, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
		/* PWM1 Mode configuration: Channel4   (MOTOR_RIGHT_PWM_PIN)*/
		TIM_OC4Init(TIM4, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

		TIM_Cmd(TIM4, ENABLE);
}


void init_motor_CWCCW(void){
		GPIO_InitTypeDef GPIO_InitStruct;
		/* Enable GPIO D clock. */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
        #ifdef L298N_MODE
        GPIO_InitStruct.GPIO_Pin =  MOTOR_LEFT_IN1_PIN| MOTOR_LEFT_IN2_PIN | MOTOR_RIGHT_IN3_PIN | MOTOR_RIGHT_IN4_PIN ;
        #else /*Smart EPW*/
        GPIO_InitStruct.GPIO_Pin =  MOTOR_LEFT_CWCCW_PIN | MOTOR_RIGHT_CWCCW_PIN;
        #endif
	
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;            // Alt Function - Push Pull
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init( MOTOR_CWCCW_PORT, &GPIO_InitStruct ); 
		//GPIO_WriteBit(MOTOR_CWCCW_PORT,MOTOR_LEFT_CWCCW_PIN,Bit_RESET);
		//GPIO_WriteBit(MOTOR_CWCCW_PORT,MOTOR_RIGHT_CWCCW_PIN,Bit_RESET);	
}

void init_Neural(neural_state_t *n_s, float KP, float KI, float KD){
	// array initialization
    array_1d_Init_2(neuralNumber, 0.0, n_s->x);
    array_2d_Init(centerNumber, neuralNumber, move_distance, n_s->c);
    array_1d_Init_2(neuralNumber, move_distance/6, n_s->b);
    array_1d_Init(neuralNumber, 0.5, n_s->w);

    // record the temporal array value    array_1d_Copy(neuralNumber, b, b_1);
    array_1d_Copy(neuralNumber, n_s->w, n_s->w_1);
    array_1d_Copy(neuralNumber, n_s->b, n_s->b_1);
    array_2d_Copy(centerNumber, neuralNumber, n_s->c, n_s->c_1);
	
	// learning speed
	n_s->eta = 0.15;

	// PID parameter
	n_s->kp = KP;
	n_s->ki = KI;
	n_s->kd = KD;

	// PID parameter of last cycle
	n_s->kp_1 = KP;
	n_s->ki_1 = KI;
	n_s->kd_1 = KD;

	// dy/du = jacobian
	n_s->yu = 0;
	n_s->dyu = 0;

	// output of reference model
	n_s->rf_out = 0;
	n_s->rf_out_1 = 0;
	n_s->rf_out_2 = 0;

	// output of neural identifier
	n_s->ynout = 0;
	n_s->ynout_sum = 0;

	// PID controller output and last 2 cycles
	n_s->du = 0;
	n_s->u = 0;
	n_s->u_1 = 0;
	n_s->u_2 = 0;

	// error of identifier and PID controller
	n_s->e_1 = 0; 
	n_s->e_2 = 0;
	n_s->erbf = 0;
	n_s->e = 0;
	n_s->erbf_correct_times = 0;

	n_s->erbf_avg = 0;
}

void neural_reset(neural_state_t *n_s){
	n_s->xc[0] = 0;//e - e_1;
	n_s->xc[1] = 0;
	n_s->xc[2] = 0;//e - (2 * e_1) + e_2;
	n_s->e = 0;
	n_s->e_1 = 0;
	n_s->e_2 = 0;
//	n_s->erbf = 0;
	n_s->erbf_correct_times = 0;
//	n_s->ynout = 0;
	n_s->ynout_sum = 0;
//	n_s->u_1 = 0;
//	n_s->u = 0;
//	n_s->du = 0;
}

/*============================================================================*/
/*============================================================================*
 ** function : init_encoder
 ** brief : initialization the encoder , GPIO is setting to input.
 ** param : None
 ** retval : None
 **============================================================================*/
/*============================================================================*/
void init_encoder(void)
{
		GPIO_InitTypeDef GPIO_InitStruct;
		/* Enable GPIO A clock. */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

		GPIO_InitStruct.GPIO_Pin =       ENCODER_LEFT_PHASE_A_PIN \
										 |   ENCODER_RIGHT_PHASE_A_PIN  \ 
										 |   ENCODER_LEFT_PHASE_B_PIN \
										 |  ENCODER_RIGHT_PHASE_B_PIN ; 

		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init( ENCODER_PORT, &GPIO_InitStruct ); 
}

/*============================================================================*/
/*============================================================================*
 ** function : init_External_Interrupt
 ** brief : connect the two encoder phase A to interrupt
 ** param : None
 ** retval : None
 **============================================================================*/
/*============================================================================*/
void init_External_Interrupt(void){
		GPIO_InitTypeDef GPIO_InitStruct;
		EXTI_InitTypeDef EXTI_InitStruct;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

#if 1
		/* Connect EXTI Line0 to PA0 pin */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);
		EXTI_InitStruct.EXTI_Line = EXTI_Line0;
		EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStruct);
		EXTI_ClearITPendingBit(EXTI_Line0);
		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);



		/* Connect EXTI Line1 to PA1 pin */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource1);
		EXTI_InitStruct.EXTI_Line = EXTI_Line1;
		EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStruct);
		EXTI_ClearITPendingBit(EXTI_Line1);
		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
#endif
		/* Connect EXTI Line2 to PA2 pin */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource2);
		EXTI_InitStruct.EXTI_Line = EXTI_Line2;
		EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStruct);
		EXTI_ClearITPendingBit(EXTI_Line2);
		NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		/* Connect EXTI Line3 to PA3 pin */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource3);
		EXTI_InitStruct.EXTI_Line = EXTI_Line3;
		EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStruct);
		EXTI_ClearITPendingBit(EXTI_Line3);
		NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
}


void init_car(){
        
		init_motor();
        init_motor_CWCCW();
        init_encoder();
		init_External_Interrupt();
		init_Neural(&n_r, 1, 50, 1);
		init_Neural(&n_l, 1.5, 50, 1);

		//printf("Neuron Number = %d", neuralNumber);
		//printf("distance = %d\n", move_distance);
		//rintf("Neural task delay period = %d\n", NEURAL_PERIOD);
		printf("initial Kp = %d, Ki = %d, Kd = %d",  (int)(n_r.kp*100), (int)(n_r.ki*100), (int)(n_r.kd*100));
		//printf("Move period = %d\n", MOVE_PERIOD);
		printf("learning speed = %d\n", (int)(n_r.eta*100));

		carTimers = xTimerCreate("Car_State_Polling",	 ( CAR_POLLING_PERIOD), pdTRUE, ( void * ) 1,  Car_State_Polling );
		xTimerStart( carTimers, 0 );

//		Show_data_Timers = xTimerCreate("Show_data_Polling",( SHOW_DATA_PERIOD), pdTRUE, ( void * ) 1,  Show_data_Polling );
//		xTimerStart( Show_data_Timers, 0 );
}



/*============================================================================*
 ** Prototype    : Car_State_Polling
 ** Description  : record the car's state of all behavior and keep to polling,
                   this is used for auto-avoidance polling mechanism
 ** Input        : None
 ** Output       : None
 ** Return Value : 
 *============================================================================*/
#define THRESHOLD_DISTANCE 50 /*unit : cm*/
/**
 *  the unit is  (CAR_POLLING_PERIOD ms) , so depend on the CAR_POLLING_PERIOD.
 *  for example the CAR_POLLING_PERIOD=20, CAR_MOVING_PERIOD=10,
 *  then the car moving will be keep 200(10*20) ms time.
 */
#define CAR_MOVING_PERIOD 250 
#define CAR_REST_PERIOD  5
void Car_State_Polling(){
		getMotorData();
		static int count=0;
        unsigned int distance[4];
        
		if(car_state==CAR_STATE_IDLE){
			proc_cmd("stop" , base_pwm_l, base_pwm_r);
				count=0;
		}
		else if(car_state==CAR_STATE_REST){
				proc_cmd("stop" , base_pwm_l, base_pwm_r);

				if(count>=CAR_REST_PERIOD){
						count=0;
						car_state=CAR_STATE_IDLE;
				}
		}
		else if(car_state==CAR_STATE_MOVE_FORWARD){
                            
            if (count_r >= MOVE_PERIOD || count_l >= MOVE_PERIOD)  //2000 == 4 cycle
            {
				printf("y_r=%d yn_r=%d \n", (int) count_r, (int)n_r.ynout_sum);
				printf("y_l=%d yn_l=%d \n", (int) count_l, (int)n_l.ynout_sum);
				printf("erbf_avg_r %d \n", (int)n_r.erbf_avg);
				printf("erbf_avg_l %d \n", (int)n_l.erbf_avg);
				printf("erbf_correct_times %d \n", (int)n_r.erbf_correct_times);
				printf("erbf_correct_times %d \n", (int)n_l.erbf_correct_times);
				//printf("%d %d %d\n", xc[0], xc[1], xc[2]);
			    printf("PID R %d %d %d \n", (int)(n_r.kp*100), (int)(n_r.ki*100), (int)(n_r.kd*100));
			    printf("PID L %d %d %d \n", (int)(n_l.kp*100), (int)(n_l.ki*100), (int)(n_l.kd*100));
			    printf("err_sum = %d \n", err_sum);
            	count_l = 0;
            	count_r = 0;
				car_state=CAR_STATE_REST;
				neural_reset(&n_r);
				neural_reset(&n_l);
        		encoder_right_counter_1 = 0;
        		encoder_left_counter_1 = 0;
        		err_sum = 0;
            }
				
		}	
		else if(car_state==CAR_STATE_MOVE_BACK){
                
				if(count>=CAR_MOVING_PERIOD){
						count=0;
						car_state=CAR_STATE_REST;
				}
		}
        else if(car_state==CAR_STATE_MOVE_LEFT){
                if (count_r >= MOVE_LEFT_RIGHT_PERIOD || count_l >= MOVE_LEFT_RIGHT_PERIOD)  //2000 == 4 cycle
                {
                	count_l = 0;
                	count_r = 0;
					car_state=CAR_STATE_REST;
					neural_reset(&n_r);
					neural_reset(&n_l);
	        		encoder_right_counter_1 = 0;
	        		encoder_left_counter_1 = 0;
                }
		}   
        else if(car_state==CAR_STATE_MOVE_RIGHT){
                if(count_r >= MOVE_LEFT_RIGHT_PERIOD || count_l >= MOVE_LEFT_RIGHT_PERIOD){
					count_l = 0;
                	count_r = 0;
					car_state=CAR_STATE_REST;
					neural_reset(&n_r);
					neural_reset(&n_l);
	        		encoder_right_counter_1 = 0;
	        		encoder_left_counter_1 = 0;
    		    }
		}
}

/*============================================================================*/
/*============================================================================*
 ** function : parse_EPW_motor_dir
 ** brief : parse the EPW of motor direction from the uart siganl,
            note, the motor_pwm_value  max is 255, even if 256, it's duty cycle is equal to 255.
 ** param : DIR_cmd
 ** retval :  None
 **============================================================================*/
/*============================================================================*/
void parse_EPW_motor_dir(unsigned char DIR_cmd)
{
		if(DIR_cmd == 'f'){
				car_state = CAR_STATE_MOVE_FORWARD;
				encoder_left_counter=0;
				encoder_right_counter=0;
		}
		else if(DIR_cmd == 's'){
				car_state = CAR_STATE_IDLE;
				encoder_left_counter=0;
				encoder_right_counter=0;
				/*even stop function is  always zero , but for security, I also set speedvalue to zero.*/
				pwm_value_left = base_pwm_l;
				pwm_value_right = base_pwm_r;
				proc_cmd("stop" , pwm_value_left , pwm_value_right);

		}
		else if(DIR_cmd == 'b'){
                car_state = CAR_STATE_MOVE_BACK;
				encoder_left_counter=0;
				encoder_right_counter=0;
				pwm_value_left = motor_speed_convert_to_pwm(MOTOR_CCW, motor_speed_value); 
				pwm_value_right=pwm_value_left;
				proc_cmd("backward" , pwm_value_left , pwm_value_right);
		}
        else if(DIR_cmd == 'l'){
                car_state = CAR_STATE_MOVE_LEFT;
				encoder_left_counter=0;
				encoder_right_counter=0;
				pwm_value_left = motor_speed_convert_to_pwm(MOTOR_CCW, motor_speed_value); 
				pwm_value_right= motor_speed_convert_to_pwm(MOTOR_CW, motor_speed_value); 
				proc_cmd("left" , pwm_value_left , pwm_value_right);
		}
        else if(DIR_cmd == 'r'){
                car_state = CAR_STATE_MOVE_RIGHT;
				encoder_left_counter=0;
				encoder_right_counter=0;
				pwm_value_left = motor_speed_convert_to_pwm(MOTOR_CW, motor_speed_value); 
				pwm_value_right=motor_speed_convert_to_pwm(MOTOR_CCW, motor_speed_value);
				proc_cmd("right" , pwm_value_left , pwm_value_right);
		}
		else{
				/*do not anything*/
		}
}


/*============================================================================*/
/*============================================================================*
 ** function : motor_speed_convert_to_pwm
 ** brief : convert the motor speed value to pwm value based on motor driver.
 **         motor speed value range is 1~10,
 **         convert the pwm value by 
 **           CW : 146~222 
 **           CCW : 32~108 
 **         1 speed value convert to 8 pwm scale.
 ** param : motor_dir, speed_value
 ** retval : pwm value
 **============================================================================*/
/*============================================================================*/
int motor_speed_convert_to_pwm(int motor_dir, int speed_value){
    static int pwm_value;
    switch(motor_dir){
        case MOTOR_CW: /*pwm range: 146~222*/
            pwm_value = 146 + speed_value*8;
            break;
        case MOTOR_CCW: /*pwm range: 32~108*/
            pwm_value = 108 - speed_value*8;
            break;
    }
    /*pwm value accept range is 0~255*/
    if(pwm_value <=0) pwm_value = 0;
    else if (pwm_value >=255) pwm_value = 255;
    return pwm_value;
}


void PerformCommand(unsigned char group,unsigned char control_id, unsigned char value)
{
   if(group == OUT_EPW_CMD){ /*0*/
		switch ( control_id )
		{
		    case EPW_MOTOR_DIR:
		        parse_EPW_motor_dir(value);
		        break;
		    case EPW_MOTOR_PWM:
		        motor_speed_value = value; /*0~10 scale*/
                
		        break;
		    case EPW_ACTUATOR_A :
		        set_linearActuator_A_cmd(value , 255); /*the actuator of pwm_value is fixed, value is dir flag.*/
		        break;
		    case EPW_ACTUATOR_B :                
		        set_linearActuator_B_cmd(value , 255); /*the actuator of pwm_value is fixed. value is dir flag.*/
		        break;
		    default:
		        ;
		}
   }
}

float pow2(float number, int times){
	int i = 0;
	float tmp = 1.0f;
	for (i = 0; i < times; ++i)
	{
		tmp = number * tmp;
	}
	return tmp;
}

float abs2(float number){
	float result = number;
	if (result < 0)
	{
		result = (-1) * result;
	}
	return result;
}

float exponential(float ld){
	float result = 1.0;
	float term = ld;
	int diaminator = 2;
	int count = 0;
	int test = 0;

	while(count < 20){
		result = result + term;
		term = term * ld;
		term = term / (float)diaminator;
		(int)diaminator ++;
		count ++;
		test = (int)(abs2(term)*1000);
		if(test < 10){
			break;
		}
	}

	return result;
}

void neural_update(neural_state_t *n_s, float rin, int encoder_counter, int wheel_count){
if (car_state != CAR_STATE_IDLE)
{

#if NEURAL_IDENTIFIER == 1
	    int i = 0, j = 0;

        // 1. RBFNN Output
        n_s->ynout = 0;
        for ( i = 0; i < neuralNumber; ++i)
        {
        	n_s->norm_c_2[i] = pow2((n_s->x[0] - n_s->c[0][i]), 2) + pow2((n_s->x[1] - n_s->c[1][i]), 2) + pow2((n_s->x[2] - n_s->c[2][i]), 2);
            	            
            float tmp = (((-0.5) * n_s->norm_c_2[i]));
            tmp = tmp/ pow2(n_s->b[i], 2);
            n_s->h[i] = exponential(tmp);
            n_s->ynout = n_s->ynout + (n_s->h[i])*(n_s->w[i]);
        }
        n_s->ynout_sum += n_s->ynout;
        
        // 2 Get the motor speed of last clock cycle, and calculate the error of rbf
        n_s->erbf = encoder_counter - n_s->ynout;
        n_s->erbf_record[4] = n_s->erbf_record[3];
		n_s->erbf_record[3] = n_s->erbf_record[2];
		n_s->erbf_record[2] = n_s->erbf_record[1];
		n_s->erbf_record[1] = n_s->erbf_record[0];
		n_s->erbf_record[0] = abs2(n_s->erbf);
		n_s->erbf_avg = (n_s->erbf_record[0] + n_s->erbf_record[1] + n_s->erbf_record[2] + n_s->erbf_record[3] + n_s->erbf_record[4])/5.0;

        // 3. Update w of RBFNN
        for ( i = 0; i < neuralNumber; ++i)
        {
        	float tmp = n_s->erbf * n_s->h[i];
            n_s->dw[i] = n_s->eta * tmp;
            n_s->w_1[i] = n_s->w[i];
            n_s->w[i] = n_s->w_1[i] + n_s->dw[i];
        }

        // 4. Update bj
        for ( i = 0; i < neuralNumber; ++i)
        {
        	float tmp = n_s->eta * n_s->erbf;
        	tmp = tmp * n_s->w[i];
        	tmp = tmp * n_s->h[i];
        	tmp = tmp * n_s->norm_c_2[i];
        	tmp = tmp / pow2(n_s->b[i], 3);
            n_s->db[i] = tmp;
            n_s->b_1[i] = n_s->b[i];
            n_s->b[i] = n_s->b_1[i] + n_s->db[i];
        }

        // 5. Update Cj
        for ( i = 0; i < neuralNumber; ++i)
        {
            for ( j = 0; j < centerNumber; ++j)
            {
            	float tmp = n_s->eta * n_s->erbf;
            	tmp = tmp * n_s->w[i];
            	tmp = tmp * n_s->h[i];
            	tmp = tmp * (n_s->x[j] - n_s->c[j][i]);
            	tmp = tmp /  pow2(n_s->b[i], 2);
            	n_s->dc[j][i] = tmp;
                n_s->c_1[j][i] = n_s->c[j][i];
                n_s->c[j][i] = n_s->c_1[j][i] + n_s->dc[j][i];	                
            }
        }

        // 6. Calculate Jacobian
        n_s->yu = 0;
        for ( i = 0; i < neuralNumber; ++i)
        {
        	float tmp = n_s->w[i] * n_s->h[i];
        	float tmp2 = (-1) * n_s->x[0];
        	tmp = tmp *  (tmp2 + n_s->c[0][i]) ;
        	tmp = tmp / pow2(n_s->b[i], 2);
            n_s->yu = n_s->yu + tmp;
        }
        n_s->dyu = n_s->yu;
#endif
        // 7. Update error
        n_s->rf_out = referenceModel2(rin, n_s->rf_out_1, n_s->rf_out_2);
        n_s->e = n_s->rf_out - (float)encoder_counter;
        n_s->rf_out_2 = n_s->rf_out_1;
        n_s->rf_out_1 = n_s->rf_out;

        // 8. Incremental PID
        n_s->xc[0] = n_s->e - n_s->e_1;
        n_s->xc[1] = n_s->e;
        n_s->xc[2] = n_s->e - (2 * n_s->e_1) + n_s->e_2; 
#if NEURAL_IDENTIFIER == 1
        //int tmp_erbf = (int)(abs2(erbf)*100);
        int tmp_erbf = (wheel_count - n_s -> ynout_sum)/wheel_count;
        tmp_erbf = tmp_erbf * 100; 
        if (tmp_erbf < 10)
        {
        	n_s -> erbf_correct_times ++;
        }
        if ((tmp_erbf < 10) && (encoder_counter > 1) && (n_s->erbf_correct_times >30)){
        	n_s->erbf_correct_times ++;
	        float kp_add = n_s->eta * n_s->e;
	        kp_add = kp_add * n_s->dyu;
	        kp_add = kp_add * n_s->xc[0];
	        
	        float ki_add = n_s->eta * n_s->e;
	        ki_add = ki_add * n_s->dyu;
	        ki_add = ki_add * n_s->xc[1];
	        
	        float kd_add = n_s->eta * n_s->e;
	        kd_add = kd_add * n_s->dyu;
	        kd_add = kd_add * n_s->xc[2];
	        
	        // 10. update kp(k-1) ki(k-1) kd(k-1)
	        n_s->kp_1 = n_s->kp;
	        n_s->ki_1 = n_s->ki;
	        n_s->kd_1 = n_s->kd;

	        // 9. Update the parameter of PID controller    
	        n_s->kp = n_s->kp_1 + kp_add;
	        n_s->ki = n_s->ki_1 + ki_add;
	        n_s->kd = n_s->kd_1 + kd_add;
	        if (n_s->kp < 0)
	        {
	        	n_s->kp = n_s->kp_1;
	        }
	        if (n_s->ki < 0)
	        {
	        	n_s->ki = n_s->ki_1;
	        }
	        if (n_s->kd < 0)
	        {
	        	n_s->kd = n_s->ki_1;
	        }		       
	    }
#endif
        // 11. Calculate the output of PID controller
        n_s->du = n_s->kp * n_s->xc[0] + n_s->ki * n_s->xc[1] + n_s->kd * n_s->xc[2];
        n_s->u = n_s->u_1 + n_s->du;
        if (n_s->u < 0)
        {
        	n_s->u = 0;
        }else if(n_s->u > 110) {
        	n_s->u = 110;
        }
#if NEURAL_IDENTIFIER == 1
        // 12. update yout(k-1) u(k-1)
        n_s->yout_1 = n_s->x[1];
        n_s->u_2 = n_s->u_1;
        n_s->u_1 = n_s->u;
#endif
        // 13. update e(k-1) e(k-2)
        n_s->e_2 = n_s->e_1;
        n_s->e_1 = n_s->e;
#if NEURAL_IDENTIFIER == 1
        // 14. update input of RBFNN
        n_s->x[0] = n_s->du;
        n_s->x[1] = (float)encoder_counter;
        n_s->x[2] = n_s->yout_1;
#endif
    }
}

void neural_task(void *p)
{
	while(1){
		detachInterrupt(EXTI_Line0); /*close external interrupt 0*/ 
		detachInterrupt(EXTI_Line1); /*close external interrupt 1*/ 
		detachInterrupt(EXTI_Line2); /*close external interrupt 2*/ 
		detachInterrupt(EXTI_Line3); /*close external interrupt 3*/ 

	    if(car_state == CAR_STATE_MOVE_FORWARD){
	    	getMotorData();
	    	float err = 0;
    		rin = move_distance;
    		err = ((float)(encoder_left_counter_1 - encoder_right_counter_1));
	    	err_sum += err;

			neural_update(&n_r, rin, encoder_right_counter_1, count_r);
		    neural_update(&n_l, rin + err, encoder_left_counter_1, count_l);

		    float input_l =  n_l.u_1;
		    float input_r =  n_r.u_1;

	        proc_cmd("forward", base_pwm_l+(int)input_l, base_pwm_r+(int)input_r);

	    }
	    else if (car_state == CAR_STATE_MOVE_LEFT){
	    	rin = 3;
	    	getMotorData();

	    	neural_update(&n_r, rin, encoder_right_counter_1, count_l);
		    neural_update(&n_l, rin, encoder_left_counter_1, count_r);

		    float input_l =  n_l.u_1;
		    float input_r =  n_r.u_1;

	        proc_cmd("forward", base_pwm_l-(int)input_l, base_pwm_r+(int)input_r);
	    }
	    else if (car_state == CAR_STATE_MOVE_RIGHT){
	    	rin = 3;
	    	getMotorData();

	    	neural_update(&n_r, rin, encoder_right_counter_1, count_l);
		    neural_update(&n_l, rin, encoder_left_counter_1, count_r);

		    float input_l =  n_l.u_1;
		    float input_r =  n_r.u_1;

	        proc_cmd("forward", base_pwm_l+(int)input_l, base_pwm_r-(int)input_r);
	    }

	    else{
	    	rin = 0.0;
	    	count_l = 0;
	    	count_r = 0;
	    	getMotorData();

		    //neural_update(&n_r, rin, encoder_right_counter_1, count_r);
		    //neural_update(&n_l, rin, encoder_left_counter_1, count_l);

		    //float input_l =  n_l.u_1;
		    //float input_r =  n_r.u_1;

	        proc_cmd("forward", base_pwm_l, base_pwm_r);
	    }

	    attachInterrupt(EXTI_Line0); 
		attachInterrupt(EXTI_Line1);
		attachInterrupt(EXTI_Line2); 
		attachInterrupt(EXTI_Line3);
	    vTaskDelay(NEURAL_PERIOD);
	}
}
void Show_data_Polling(void)
{
//	printf("r %d \n", (int) encoder_right_counter_1);
//	printf("l %d \n", (int) encoder_left_counter_1);
//	printf("l : %d r : %d\n", count_l, count_r);
	//int tmp_erbf = (erbf*100); 
	//tmp_erbf = tmp_erbf / encoder_right_counter_1;
	//tmp_erbf = tmp_erbf*100;
	//printf("fff\n");
//	printf("erbf = %d ", (int)erbf);
//	printf("yn = %d ", (int)ynout);
	printf("r %d %d %d \n", (int) encoder_right_counter_1, (int)n_r.ynout, (int)n_r.erbf);
	printf("l %d %d %d\n", (int) encoder_left_counter_1, (int)n_l.ynout, (int)n_l.erbf);
//	printf("erbf %d \n", (int)n_r.erbf_avg);
//	printf("erbf %d \n", (int)n_l.erbf_avg);
	//printf("%d %d %d\n", xc[0], xc[1], xc[2]);
    printf("R %d %d %d \n", (int)(n_r.kp*100), (int)(n_r.ki*100), (int)(n_r.kd*100));
    printf("L %d %d %d \n", (int)(n_l.kp*100), (int)(n_l.ki*100), (int)(n_l.kd*100));
//    printf("c * 100 = %d \n", (int)(c[0][0]*100));
 //   printf("db * 100 = %d \n", (int)(db[0]*100));
   // printf("dw * 100 = %d \n", (int)(dw[0]*100));
 //   printf("du * 100 = %d \n", (int)(du*100));	        
    //printf("100 * u_1 = %d\n", (int)(u_1 * 100));
//	printf("erbf=%d\n", (int)(erbf*100));
	      
	//printf(" left encoder : %d \n", count_l);
	//printf(" right encoder : %d \n", count_r);
}

void Get_Motor_Polling (void)
{
	getMotorData();
}

void getMotorData(void)
{
    /*for SmartEPW is used by 500 pulse/rev */
    //rpm_left_motor=(float)encoder_left_counter;// * 3.0f;//60.0f / 1000.0f / 0.02f;
    //rpm_right_motor=(float)encoder_right_counter;// * 3.0f;// 60.0f  / 1000.0f / 0.02f;
    encoder_right_counter_1 = encoder_right_counter;
    encoder_left_counter_1 = encoder_left_counter;

    encoder_left_counter = 0;
    encoder_right_counter = 0;          
}

void EXTI0_IRQHandler(){

		if(EXTI_GetITStatus(EXTI_Line0) != RESET)
		{
				encoder_left_counter++ ;
				count_l ++;
				EXTI_ClearITPendingBit(EXTI_Line0);
		}
}

void EXTI1_IRQHandler(){
		if(EXTI_GetITStatus(EXTI_Line1) != RESET)
		{
				encoder_right_counter++ ;
				count_r ++;
				EXTI_ClearITPendingBit(EXTI_Line1);
		}
}
void EXTI2_IRQHandler(){

		if(EXTI_GetITStatus(EXTI_Line2) != RESET)
		{
				encoder_left_counter++ ;
				count_l ++;
				EXTI_ClearITPendingBit(EXTI_Line2);
		}
}

void EXTI3_IRQHandler(){
		if(EXTI_GetITStatus(EXTI_Line3) != RESET)
		{
				encoder_right_counter++ ;
				count_r ++;
				EXTI_ClearITPendingBit(EXTI_Line3);
		}
}
void detachInterrupt(uint32_t EXTI_LineX){
		EXTI_InitTypeDef EXTI_InitStruct;
		EXTI_InitStruct.EXTI_Line = EXTI_LineX;
		EXTI_InitStruct.EXTI_LineCmd = DISABLE;
}

void attachInterrupt(uint32_t EXTI_LineX){
		EXTI_InitTypeDef EXTI_InitStruct;
		EXTI_InitStruct.EXTI_Line = EXTI_LineX;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
}
