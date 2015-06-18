#include "FreeRTOS.h"
#include "timers.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_syscfg.h"
#include "EPW_behavior.h"
#include "ultrasound.h"
#include "timers.h"
#include "uart.h"
//#include "clib.h"
#include "EPW_command.h"
#include "PID.h"
#include "math.h"

#define CAR_POLLING_PERIOD  20//unit : ms
#define PID_POLLING_PERIOD  20//unit : ms
#define SHOW_DATA_PERIOD  2000
#define GET_MOTOR_PERIOD   50
#define MOVE_PERIOD 3000

#define MOTOR_CW 0
#define MOTOR_CCW 1

#define PI 3.14
#define ts 0.001
#define neuralNumber  6
#define centerNumber  3
#define RF 1

int encoder_left_counter_1;
int encoder_right_counter_1;

float move_distance = 10.0;

typedef struct _neural_State{
float eta = 0.15;
float kp = 3;
float ki = 25;
float kd = 3;

float kp_1 = 1;
float ki_1 = 3;
float kd_1 = 1;

float yout = 0;
float yout_1 = 0;
float yu = 0;
float dyu = 0;

float rf_out = 0;
float rf_out_1 = 0;
float rf_out_2 = 0;

float h[neuralNumber];
float ynout = 0;

float du = 0;
float u = 0;
float u_1 = 0;
float u_2 = 0;

float e_1 = 0, e_2 = 0;
float erbf = 0;
float e = 0;

float x[centerNumber];

float c[centerNumber][neuralNumber];
float dc[centerNumber][neuralNumber];

float b[neuralNumber];
float db[neuralNumber];

float w[neuralNumber];
float dw[neuralNumber];

float c_1[centerNumber][neuralNumber];
float b_1[neuralNumber];
float w_1[neuralNumber];

float xc[3];

float norm_c_2[neuralNumber];  // (norm_c)^2
} neural_state;

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
    for ( i = 0; i < size_x; ++i)
        for ( j = 0; j < size_y; ++j)
            array[i][j] = (float)value;
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
static neural_state_t n_r;

int encoder_right_counter;
int encoder_left_counter;

int count_l = 0;
int count_r = 0;

/*pwm regulate of two motor */
static int pwm_value_left = 125; /*default pwm value*/
static int pwm_value_right = 125;
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
}


void init_car(){
        
		init_motor();
        init_motor_CWCCW();
        init_encoder();
		init_External_Interrupt();

		// array initialization
	    array_1d_Init_2(neuralNumber, 0.0, x);
	    array_2d_Init(centerNumber, neuralNumber, move_distance/2, c);
	    array_1d_Init_2(neuralNumber, move_distance*10, b);
	    array_1d_Init(neuralNumber, 0.5, w);

	    // record the temporal array value    array_1d_Copy(neuralNumber, b, b_1);
	    array_1d_Copy(neuralNumber, w, w_1);
	    array_1d_Copy(neuralNumber, b, b_1);
	    array_2d_Copy(centerNumber, neuralNumber, c, c_1);
        

		carTimers = xTimerCreate("Car_State_Polling",	 ( CAR_POLLING_PERIOD), pdTRUE, ( void * ) 1,  Car_State_Polling );
		xTimerStart( carTimers, 0 );

		Show_data_Timers = xTimerCreate("Show_data_Polling",( SHOW_DATA_PERIOD), pdTRUE, ( void * ) 1,  Show_data_Polling );
		xTimerStart( Show_data_Timers, 0 );

//		Get_Motor_Timers = xTimerCreate("Get_Motor_Polling", (GET_MOTOR_PERIOD), pdTRUE, ( void * ) 2,  Get_Motor_Polling );
//		xTimerStart( Get_Motor_Timers, 0 );
	
        /*Initialization the right motor of pid paremeter.*/
        Kp=2.0f; Ki=0.1f; Kd=4.0f;
        InitPID(&PID_Motor_R , Kp ,Ki,Kd);
        InitPID(&PID_Motor_L , Kp ,Ki,Kd);
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
			proc_cmd("stop" , 125 , 125);
				count=0;
		}
		else if(car_state==CAR_STATE_REST){
				proc_cmd("stop" , 125 , 125);

				count++;
				if(count>=CAR_REST_PERIOD){
						count=0;
						car_state=CAR_STATE_IDLE;
				}
		}
		else if(car_state==CAR_STATE_MOVE_FORWARD){
                count++;

				//int pwm_value_right = 150;
				//int 																																																																																																																																																																																																																																																																																																								``	pwm_value_left = (int)PID_Inc_Calc(&PID_Motor_L , rpm_right_motor, rpm_left_motor);
                                
                if (count_r >= MOVE_PERIOD)  //2000 == 4 cycle
                {
                	count_l = 0;
                	count_r = 0;
					car_state=CAR_STATE_REST;
					xc[0] = 0;//e - e_1;
	        		xc[1] = 0;
	        		xc[2] = 0;//e - (2 * e_1) + e_2;
	        		e = 0;
	        		e_1 = 0;
	        		e_2 = 0;
	        		erbf = 0;
	        		ynout = 0;
	        		u_1 = 0;
	        		u = 0;
	        		du = 0;
	        		encoder_right_counter_1 = 0;
	        		encoder_left_counter_1 = 0;
					InitPID(&PID_Motor_R, Kp, Ki, Kd);
					InitPID(&PID_Motor_L, Kp, Ki, Kd);
//					proc_cmd("stop" , 125 , 125);
                }
				/*if(count>=CAR_MOVING_PERIOD){
						count=0;
						car_state=CAR_STATE_REST;
				}*/
		}	
		else if(car_state==CAR_STATE_MOVE_BACK){
                
				count++;
				if(count>=CAR_MOVING_PERIOD){
						count=0;
						car_state=CAR_STATE_REST;
				}
		}
        else if(car_state==CAR_STATE_MOVE_LEFT){
                count++;
				if(count>=CAR_MOVING_PERIOD){
						count=0;
						car_state=CAR_STATE_REST;
				}
		}   
        else if(car_state==CAR_STATE_MOVE_RIGHT){
                count++;
				if(count>=CAR_MOVING_PERIOD){
						count=0;
						car_state=CAR_STATE_REST;
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
				pwm_value_left = 125;
				pwm_value_right = 125;
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
    return pwm_value; ;
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
		    case EPW_PID_ALG_KP :
                 Kp = (float)value;
		         InitPID(&PID_Motor_R , Kp/10.0f ,Ki,Kd);
		        break;
		    case EPW_PID_ALG_KI :
                 Ki = (float)value;
		         InitPID(&PID_Motor_R , Kp, Ki/10.0f, Kd);
		        break;
            case EPW_PID_ALG_KD :
                 Kd = (float)value;
		         InitPID(&PID_Motor_R , Kp, Ki, Kd/10.0f);
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

void neural_task(void *p)
{
	float erbf_record[5] = {0};
	float erbf_avg = 0;
	while(1){
		detachInterrupt(EXTI_Line0); /*close external interrupt 0*/ 
		detachInterrupt(EXTI_Line1); /*close external interrupt 1*/ 

	    if(car_state == CAR_STATE_MOVE_FORWARD){
	    	rin = move_distance;
	    }else{
	    	rin = 0.0;
	    }

        int i = 0, j = 0;

        // 2. RBFNN Output
        ynout = 0;
        for ( i = 0; i < neuralNumber; ++i)
        {
        	norm_c_2[i] = pow2((x[0] - c[0][i]), 2) + pow2((x[1] - c[1][i]), 2) + pow2((x[2] - c[2][i]), 2);
            	            
            float tmp = (((-0.5) * norm_c_2[i]));
            tmp = tmp/ pow2(b[i], 2);
            h[i] = exponential(tmp);
            ynout = ynout + h[i]*w[i];
        }
        
        // 2.5 Get the motor speed of last clock cycle, and calculate the error of rbf
        getMotorData();
        erbf = (float)encoder_right_counter_1 - ynout;
        erbf_record[4] = erbf_record[3];
		erbf_record[3] = erbf_record[2];
		erbf_record[2] = erbf_record[1];
		erbf_record[1] = erbf_record[0];
		erbf_record[0] = abs2(erbf);
		erbf_avg = (erbf_record[0] + erbf_record[1] + erbf_record[2] + erbf_record[3] + erbf_record[4])/5.0;

        // 3. Update w of RBFNN
        for ( i = 0; i < neuralNumber; ++i)
        {
        	float tmp = erbf * h[i];
            dw[i] = eta * tmp;
            w_1[i] = w[i];
            w[i] = w_1[i] + dw[i];
        }

        // 4. Update bj
        for ( i = 0; i < neuralNumber; ++i)
        {
        	float tmp = eta * erbf;
        	tmp = tmp * w[i];
        	tmp = tmp * h[i];
        	tmp = tmp * norm_c_2[i];
        	tmp = tmp / pow2(b[i], 3);
            db[i] = tmp;
            b_1[i] = b[i];
            b[i] = b_1[i] + db[i];
        }

        // 5. Update Cj
        for ( i = 0; i < neuralNumber; ++i)
        {
            for ( j = 0; j < centerNumber; ++j)
            {
            	float tmp = eta * erbf;
            	tmp = tmp * w[i];
            	tmp = tmp * h[i];
            	tmp = tmp * (x[j] - c[j][i]);
            	tmp = tmp /  pow2(b[i], 2);
            	dc[j][i] = tmp;
                c_1[j][i] = c[j][i];
                c[j][i] = c_1[j][i] + dc[j][i];	                
            }
        }

        // 6. Calculate Jacobian
        yu = 0;
        for ( i = 0; i < neuralNumber; ++i)
        {
        	float tmp = w[i] * h[i];
        	float tmp2 = (-1) * x[0];
        	tmp = tmp *  (tmp2 + c[0][i]) ;
        	tmp = tmp / pow2(b[i], 2);
            yu = yu + tmp;
            //yu = yu + w[i] * h[i] * (-1 * x[0] + c[0][i]) / pow2(b[i], 2);
        }
        dyu = yu;

        // 7. Update error
#if RF == 1 
        rf_out = referenceModel2(rin, rf_out_1, rf_out_2);
        e = rf_out - (float)encoder_right_counter_1;
        rf_out_2 = rf_out_1;
        rf_out_1 = rf_out;
#else
        e = rin - (float)encoder_right_counter_1;
#endif
        // 8. Incremental PID
        xc[0] = e - e_1;
        xc[1] = e;
        xc[2] = e - (2 * e_1) + e_2; 

        //int tmp_erbf = (int)(abs2(erbf)*100);
        int tmp_erbf = (int)(erbf_avg * 100);
        if ((tmp_erbf < 200) && (encoder_right_counter_1 > 2)){
	        float kp_add = eta * e;
	        kp_add = kp_add * dyu;
	        kp_add = kp_add * xc[0];
	        
	        float ki_add = eta * e;
	        ki_add = ki_add * dyu;
	        ki_add = ki_add * xc[1];
	        
	        float kd_add = eta * e;
	        kd_add = kd_add * dyu;
	        kd_add = kd_add * xc[2];
	        
	        // 10. update kp(k-1) ki(k-1) kd(k-1)
	        kp_1 = kp;
	        ki_1 = ki;
	        kd_1 = kd;

	        // 9. Update the parameter of PID controller    
	        kp = kp_1 + kp_add;
	        ki = ki_1 + ki_add;
	        kd = kd_1 + kd_add;
	        if (kp < 0)
	        {
	        	kp = kp_1;
	        }
	        if (ki < 0)
	        {
	        	ki = ki_1;
	        }
	        if (kd < 0)
	        {
	        	kd = ki_1;
	        }		       
	    }

        // 11. Calculate the output of PID controller
        du = kp * xc[0] + ki * xc[1] + kd * xc[2];
        u = u_1 + du;
        if (u < 0)
        {
        	u = 0;
        }else if(u > 100) {
        	u = 100;
        }

        // 12. update yout(k-1) u(k-1)
        yout_1 = x[1];
        u_2 = u_1;
        u_1 = u;

        // 13. update e(k-1) e(k-2)
        e_2 = e_1;
        e_1 = e;

        // 14. update input of RBFNN
        x[0] = du;
        x[1] = (float)encoder_right_counter_1;
        x[2] = yout_1;

        proc_cmd("forward", 125, (125 + (int)u_1));

	    attachInterrupt(EXTI_Line0); 
		attachInterrupt(EXTI_Line1);
	    vTaskDelay(20);
	}
}
void Show_data_Polling(void)
{
	//int tmp_erbf = (erbf*100); 
	//tmp_erbf = tmp_erbf / encoder_right_counter_1;
	//tmp_erbf = tmp_erbf*100;
	//printf("fff\n");
//	printf("erbf = %d ", (int)erbf);
//	printf("yn = %d ", (int)ynout);
	printf("%d %d %d\n", (int) encoder_right_counter_1, (int)ynout, (int)erbf);
	//printf("%d %d %d\n", xc[0], xc[1], xc[2]);
    printf("p=%d i=%d d=%d \n", (int)(kp*100), (int)(ki*100), (int)(kd*100));
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
