
/******************************************************************************
 *
 * Smartcar application: go on the white road with solidary or dotted black boundary
 * (using 6 infrared sensors)
 * (with an imperfect obstacle avoiding function using an ultrasonic sensor)
 * (operate on FreeRTOS base)
 * 2020/8/31
 * 
 ******************************************************************************/

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Freedom metal includes. */
#include <metal/machine.h>
#include <metal/machine/platform.h>
#include <metal/plic_driver.h>
#include <metal/encoding.h>
#include <metal/stdatomic.h>
#include <metal/pwm1.h>
#include <metal/gpio1.h>
#include <metal/Sifive.h>

/* Car includes. */
#include "pwm.h"
#include "init1.h"

#define left_monitor p10
#define right_monitor p0
#define left_back_monitor p8
#define right_back_monitor p7
#define left_mid_monitor p1
#define right_mid_monitor p2

#define car_motor_right_forward p15
#define car_motor_right_back p16
#define car_motor_left_forward p14
#define car_motor_left_back p13

#define PLIC_NUM_INTERRUPTS 52
#define input_pullup 3

// Structures for registering different interrupt handlers
// for different parts of the application.
typedef void (*function_ptr_t)(void);
function_ptr_t g_ext_interrupt_handlers[PLIC_NUM_INTERRUPTS];
// Instance data for the PLIC.
plic_instance_t g_plic;
// Queue handle for FreeRTOS
QueueHandle_t Message_Queue;

uint32_t GPIO_SET(uint32_t pin_num, uint32_t pin_val, uint32_t pin_model);
void smartcar_init(void);
//void approach_obstacle(int angle, int *state, int *global_state, int *o_angle, int *finish, int *step, float *dist);
void tracking_car_control(int speed, int *state, int *global_state, int *o_angle);
uint32_t read_pin_val(uint32_t pin_num);
void PWM_run(int speed);
void PWM_left(int speed);
void PWM_right(int speed);
void PWM_back(int speed);
volatile void wait_ms(uint64_t ms);
void motor_stop();
void motor_back();
void motor_forward();
void motor_left();
void motor_right();
void basic_control();
int delay(uint32_t time_x);
int delay_us(uint32_t time_x);
void turn_left(int angle);
void turn_right(int angle);
void forward_cm(int cm);
uint32_t key_scan(uint32_t pin_num);
float ultrasonic();
void tune(uint32_t left_b, uint32_t right_b, int *global_state);

/*smart car part*/
//=====================================================================================
//=====================================================================================
volatile void wait_ms(uint64_t ms) //busy wait for the specified time
{
	static const uint64_t ms_tick = RTC_FREQ / 1000;
	volatile uint64_t *mtime = (uint64_t *)(METAL_RISCV_CLINT0_2000000_BASE_ADDRESS + METAL_RISCV_CLINT0_MTIME);
	uint64_t then = (ms_tick * ms) + *mtime;
	while (*mtime < then)
		;
}

int delay(uint32_t time_x) //delay (time_x * 1/25) us
{
	volatile uint32_t num;

	num = time_x;
	while (num >= 1)
	{
		num--;
	}
	return 0;
}

int delay_us(uint32_t time_x) //delay (time_x) us
{
	volatile uint32_t num;

	num = 25 * time_x;
	while (num > 0)
	{
		num--;
	}
	return 0;
}

uint32_t GPIO_SET(uint32_t pin_num, uint32_t pin_val, uint32_t pin_model)// which pin, value, input or output
{// enable the specified gpio pin, set it as input or output state, and enter the output value
	uint32_t input_val;
	if (pin_model == output)
	{
		GPIO_REG(METAL_SIFIVE_GPIO0_OUTPUT_EN) |= (0x01 << pin_num);
		if (pin_val == 1)
		{
			GPIO_REG(METAL_SIFIVE_GPIO0_OUTPUT_VAL) |= (0x01 << pin_num);
		}
		if (pin_val == 0)
		{
			GPIO_REG(METAL_SIFIVE_GPIO0_OUTPUT_VAL) &= ~(0x01 << pin_num);
		}
	}
	if (pin_model == input)
	{
		GPIO_REG(METAL_SIFIVE_GPIO0_INPUT_EN) |= (0x01 << pin_num);

		input_val = GPIO_REG(METAL_SIFIVE_GPIO0_INPUT_VAL) & (0x01 << pin_num);
		if (input_val != 0)
		{
			input_val = 1;
		}
		else
		{
			input_val = 0;
		}
		return input_val;
	}
	return 0;
}

void motor_stop()
{// stop the car

	GPIO_SET(car_motor_left_back, 0, output);	 //left back stops
	GPIO_SET(car_motor_left_forward, 0, output); //left forward stops

	GPIO_SET(car_motor_right_back, 0, output);	  //right back stops
	GPIO_SET(car_motor_right_forward, 0, output); //right forward stops
}

void motor_forward()
{// car goes forward

	GPIO_SET(car_motor_left_back, 0, output);	 //left back stops
	GPIO_SET(car_motor_left_forward, 1, output); //left forward runs

	GPIO_SET(car_motor_right_back, 0, output);	  //right back stops
	GPIO_SET(car_motor_right_forward, 1, output); //right forward runs
}

void motor_back()
{// car goes back

	GPIO_SET(car_motor_left_back, 1, output);	 //left back runs
	GPIO_SET(car_motor_left_forward, 0, output); //left forward stops

	GPIO_SET(car_motor_right_back, 1, output);	  //right back runs
	GPIO_SET(car_motor_right_forward, 0, output); //right forward stops
}

void motor_left()
{// car turns left

	GPIO_SET(car_motor_left_back, 1, output);	 //left back stops
	GPIO_SET(car_motor_left_forward, 0, output); //left forward stops

	GPIO_SET(car_motor_right_back, 0, output);	  //right back stops
	GPIO_SET(car_motor_right_forward, 1, output); //right forward stops
}

void motor_right()
{// car turns right

	GPIO_SET(car_motor_left_back, 0, output);	 //left back stops
	GPIO_SET(car_motor_left_forward, 1, output); //left forward stops

	GPIO_SET(car_motor_right_back, 1, output);	  //right back stops
	GPIO_SET(car_motor_right_forward, 0, output); //right forward stops
}

void PWM_run(int speed)
{// car goes forward at a specified speed
	for (int i = 1; i <= 150; i++)
	{
		if (i > speed)
		{ //Forward pins high
			GPIO_SET(car_motor_left_back, 0, output);
			GPIO_SET(car_motor_left_forward, 1, output);
			GPIO_SET(car_motor_right_back, 0, output);
			GPIO_SET(car_motor_right_forward, 1, output); 
		}
		// else if (i>0.7*speed)
		// { //Forward pins high
		// 	GPIO_SET(car_motor_left_back, 0, output);
		// 	GPIO_SET(car_motor_left_forward, 0, output);
		// 	GPIO_SET(car_motor_right_back, 0, output);
		// 	GPIO_SET(car_motor_right_forward, 1, output); // forward pins high
		// }
		else
		{
			//All pins low
			GPIO_SET(car_motor_left_back, 0, output);
			GPIO_SET(car_motor_left_forward, 0, output);

			GPIO_SET(car_motor_right_back, 0, output);
			GPIO_SET(car_motor_right_forward, 0, output); // all pins low
			delay(100);
		}
	}
}
void PWM_right(int speed)
{// the right wheel moves at a specified speed
	for (int i = 1; i <= 150; i++)
	{
		if (i > speed)
		{
			GPIO_SET(car_motor_right_back, 0, output);
			GPIO_SET(car_motor_right_forward, 0, output); 
			GPIO_SET(car_motor_left_back, 1, output);
			GPIO_SET(car_motor_left_forward, 0, output);
		}
		else
		{
			GPIO_SET(car_motor_right_back, 0, output);
			GPIO_SET(car_motor_right_forward, 0, output); // all pins low
			GPIO_SET(car_motor_left_back, 0, output);
			GPIO_SET(car_motor_left_forward, 0, output);
			delay(100);
		}
	}
}
void PWM_left(int speed)
{// the left wheel moves at a specified speed
	for (int i = 1; i <= 150; i++)
	{
		if (i > speed)
		{
			GPIO_SET(car_motor_left_back, 0, output);
			GPIO_SET(car_motor_left_forward, 0, output); 
			GPIO_SET(car_motor_right_back, 1, output);
			GPIO_SET(car_motor_right_forward, 0, output);
		}
		else
		{
			GPIO_SET(car_motor_left_back, 0, output);
			GPIO_SET(car_motor_left_forward, 0, output); // left pins low
			GPIO_SET(car_motor_right_back, 0, output);
			GPIO_SET(car_motor_right_forward, 0, output);
			delay(100);
		}
	}
}
void PWM_back(int speed)
{// car goes back at a specified speed
	for (int i = 1; i <= 150; i++)
	{
		if (i > speed)
		{
			GPIO_SET(car_motor_left_back, 1, output);
			GPIO_SET(car_motor_left_forward, 0, output);

			GPIO_SET(car_motor_right_back, 1, output);
			GPIO_SET(car_motor_right_forward, 0, output); // back pins high
		}
		// else if (i > 0.9 * speed)
		// { 
		// 	GPIO_SET(car_motor_left_back, 1, output);
		// 	GPIO_SET(car_motor_left_forward, 0, output);
		// 	GPIO_SET(car_motor_right_back, 0, output);
		// 	GPIO_SET(car_motor_right_forward, 0, output);
		// }
		else
		{
			GPIO_SET(car_motor_left_back, 0, output);
			GPIO_SET(car_motor_left_forward, 0, output);

			GPIO_SET(car_motor_right_back, 0, output);
			GPIO_SET(car_motor_right_forward, 0, output); // all pins low
			delay(100);
		}
	}
}
void turn_left(int angle)
{// car turns left with a specified angle
	motor_left();
	wait_ms(angle * 100 / 450 * 8);
	motor_stop();
	wait_ms(100);
}

void turn_right(int angle)
{// car turns right with a specified angle
	motor_right();
	wait_ms(angle * 100 / 450 * 8);
	motor_stop();
	wait_ms(100);
}

void forward_cm(int cm)
{//forward cm. 1000ms=60cm
	motor_forward();
	wait_ms(1000 / 60 * cm);
	motor_stop();
	wait_ms(1000);
}

void smartcar_init()
{ // enable the infrared sensor pins

	GPIO_SET(left_monitor, 0, input);
	GPIO_SET(right_monitor, 0, input);
	GPIO_SET(left_back_monitor, 0, input);
	GPIO_SET(right_back_monitor, 0, input);
	GPIO_SET(left_mid_monitor, 0, input);
	GPIO_SET(right_mid_monitor, 0, input);
}

uint32_t read_pin_val(uint32_t pin_num)
{// read the input value of a specified pin
	if ((GPIO_REG(METAL_SIFIVE_GPIO0_INPUT_VAL)) & (0x01 << pin_num))
	{
		return 1;
	}
	else
		return 0;
}

void tune(uint32_t left_b, uint32_t right_b, int *global_state)
{
// There is a usual situation:
//If the left back sensor touches the boundary, the car is at the left side of the road.
//That is also ture for the right back sensor.
//This function switches global_state according to this fact.

	if ((left_b == 0) && (right_b == 1))
	{
		*global_state = 0;
	}
	else if ((left_b == 1) && (right_b == 0))
	{
		*global_state = 1;
	}
}


//******The following is an imperfect function for obstacle avoiding********
/******************************************************************************/
// void approach_obstacle(int angle, int *state, int *global_state, int *o_angle, int *finish, int *step, float *dist)
// {
// 	int pre_state;

// 	if (*state==0)
// 	{
// 		turn_right(60);
// 		*state=1;
// 		*global_state=1;
// 	}
// 	else 
// 	{
// 		if (*o_angle >= angle)
// 		{
// 			if (*global_state == 0)
// 			{
// 				turn_left(5);
// 				pre_state = 4;
// 			}
// 			else if (*global_state == 1)
// 			{
// 				turn_right(5);
// 				pre_state = 5;
// 			}
// 		}
// 		else
// 		{
// 			if (*global_state == 1) //rightward
// 			{
// 				turn_left(5);
// 				pre_state = 4;
// 			}
// 			else if ((*global_state == 0) || (*global_state == 2)) //leftward
// 			{
// 				turn_right(5);
// 				pre_state = 5;
// 			}
// 		}
// 	}

// 	if ((*o_angle % angle == 0) || (*finish == 1))// avoid moving forward at the beginning
// 	{							// deal with the situation in which car meet obstacles after the switch of last 15 degrees
// 		*step+=1;
// 	}

// 	if ((*dist>0.05)&&(*step>0))
// 	{
// 		forward_cm(2);
// 		*step-=1;
// 	}

// 	*o_angle += 10;
// 	if (*o_angle >= 2*angle)
// 	{
// 		*o_angle -= 2*angle;
// 	}

// 	if (*state == 1)
// 	{
// 		*state = pre_state;
// 	}
// 	else if (*state == 2)
// 	{
// 		*state = 7;
// 	}
// 	else if (*state == 3)
// 	{
// 		*state = 8;
// 	}
// 	// else if (*state == 0)
// 	// {
// 	// 	*global_state == 2;
// 	// 	*state = 6;
// 	// }
// 	*finish=0;
// }

void tracking_car_control(int speed, int *state, int *global_state, int *o_angle)
{
	uint32_t msg1;
	uint32_t msg2;
	uint32_t msg3;
	uint32_t msg4;
	uint32_t msg5;
	uint32_t msg6;
	float dist = 1000.0;
	
	//state 
	//0:initial	1:forward	2:turn left	for lines	3:turn right for lines	
	//4:switch left for obstacle when moving forward	5:switch right for obstacle	when moving forward
	//6: initial with obstacle	7:switch left for obstacle near boundary	8:switch right for obstacle	near boundary

	// Receive the ultrasonic message from the queue
	// if (xQueueReceive(Message_Queue, &dist, portMAX_DELAY))
	// {
	// 	//write(STDOUT_FILENO, "Receiving successes!\n", strlen("Receiving successes!\n"));
	// }
	// else
	// 	dist = 1000.0;

	if (Message_Queue != NULL) // Create queue successfully
	{
		msg1 = read_pin_val(left_monitor);
		msg2 = read_pin_val(right_monitor);
		msg3 = read_pin_val(left_back_monitor);
		msg4 = read_pin_val(right_back_monitor);
		msg5 = read_pin_val(left_mid_monitor);
		msg6 = read_pin_val(right_mid_monitor);

		if ((msg1 == 1) && (msg2 == 1) && (msg3 == 1) && (msg4 == 1) && (msg5 == 1) && (msg6 == 1)) // not touch the boundary
		{
			if (dist >= 0.4)
			{
				if ((*state == 2)||(*state == 7))
				{
					if (*o_angle < 30)
					{
						turn_left(30 - *o_angle);
					}
					*global_state = 1 - *global_state;
				}
				if ((*state == 3)||(*state == 8))
				{
					if (*o_angle < 30)
					{
						turn_right(30 - *o_angle);
					}
					*global_state = 1 - *global_state;
				}
				if (*state == 1)
				{
					for (int i = 1; i <= 3; i++)
					{
						PWM_run(speed);
						motor_stop();
					}
				}
				if (*state==0)
				{
					turn_left(20);
				}
				
				*state = 1;
			}
		/////////////////////////////////////////////
			else //meet the obstacle
			{
				// if ((*state==2)||(*state==3)||(*state==7)||(*state==8))
				// {
				// 	approach_obstacle(90, state, global_state, o_angle, finish, step, &dist);
				// }
				// else
				// {
				// 	approach_obstacle(130, state, global_state, o_angle, finish, step, &dist);
				// }
				
			}
		}
		/////////////////////////////////////////////
		else //touch the boundary
		{
			motor_stop();
			wait_ms(500);
			// *finish=1;

			if ((*o_angle!=0)&&(msg2==1)&&(msg4==1)&&(msg6==1))
			{
				turn_right(10);
				*state = 3;
				*global_state=0;
			}
			else if ((*o_angle!=0)&&(msg1==1)&&(msg3==1)&&(msg5==1))
			{
				turn_left(10);
				*state = 2;
				*global_state=1;
			}
				
			else 
			{
				tune(msg3, msg4, global_state);
				if (*global_state == 2)
				{
					*global_state = 1;
				}
				if (*state == 1)
				{
					motor_back();
					wait_ms(30);
				}
				if (*global_state == 0) //touch the left boundary
				{
					turn_right(10);
					*state = 3;
				}
				else
				{

					turn_left(10);
					*state = 2;
				}
			}
		}
	}

	else
	{
		write(STDOUT_FILENO, "No queue!\n", strlen("No queue!\n"));
	}
}

//The following 2 functions are used to get the distance

// float ultrasonic()
// {// The original one Sunny wrote. There are some problems in it.
// 	uint64_t t1=0,t2=0;
// 	float distance=0;

// 	GPIO_SET(p9,0,output);
// 	//printf("init\n");
// 	delay_us(20);
// 	//printf("output\n");
// 	GPIO_SET(p9,1,output);
// 	delay_us(200);
// 	GPIO_SET(p9,0,output);
// 	delay_us(20);
// 	//printf("stop output\n");

// 	GPIO_REG(METAL_SIFIVE_GPIO0_OUTPUT_EN) &=~(0x01<<p12);
// 	GPIO_REG(METAL_SIFIVE_GPIO0_INPUT_EN) |= (0x01<<p12);
// 	distance = GPIO_SET(p9,0,input_pullup);
// 	//printf("nothing\n");
// 	for ( int a=0;a<2000;a++ )
// 	{
// 		if (read_pin_val(p9) == 1)	break;
// 		write(STDOUT_FILENO, "no\n", strlen("no\n"));
// 	}
// 	t1 = get_timer_value_1();
// 	while (read_pin_val(p9) == 1)
// 	{
// 		t2 = get_timer_value_1();
// 		write(STDOUT_FILENO, "yes\n", strlen("yes\n"));
// 	}

// 	if (t2!=0)
// 	{
// 		distance =( (float)(t2-t1) )/RTC_FREQ*340/2;

// 		printf("%.2f   \n \n",distance*100);
// 		return distance;
// 	}
// 	else
// 	{
// 		return 1000.0;
// 	}

// }
float ultrasonic()
{// THe one wrote by Teacher Chen.
	uint64_t t1 = 0, t2 = 0;
	float distance = 0;
	__suseconds_t ultra_t;
	//    struct timeval t1 ,t2;
	//	uint64_t t1, t2;
	char ultra_r;
	long cpu_hz, ut1, ut2;
	uint32_t distance_d;

	ultra_r = 0;

#if 0
//	metal_gettimeofday(&t1,NULL);
//	metal_gettimeofday(&t2,NULL);

//	GPIO_REG(METAL_SIFIVE_GPIO0_OUTPUT_EN) |= (0x01<<p9);
	ut1 = get_timer_value_1();
	ut2 = get_timer_value_1();

//    ut1 = metal_cpu_get_mtime(cpu);
//    ut2 = metal_cpu_get_mtime(cpu);

	ultra_t =ut2-ut1;
	GPIO_SET(p9,1,_OUTPUT);

//	GPIO_REG(METAL_SIFIVE_GPIO0_OUTPUT_VAL) |=(0x01<<p9);
	while(ultra_t<32768)
	{
		ut2 = get_timer_value_1();
		ultra_t =ut2-ut1;
	}
//	GPIO_REG(METAL_SIFIVE_GPIO0_OUTPUT_VAL) &=~(0x01<<p9);
	GPIO_SET(p9,0,_OUTPUT);
    ultra_r = 1;
	return(2);
#endif

#if 0
	metal_gettimeofday(&t1,NULL);
	metal_gettimeofday(&t2,NULL);

//	GPIO_REG(METAL_SIFIVE_GPIO0_OUTPUT_EN) |= (0x01<<p9);
//	ut1 = get_timer_value_1();
//	ut2 = get_timer_value_1();

	ultra_t =t2.tv_usec-t1.tv_usec;
	GPIO_SET(p9,1,_OUTPUT);

//	GPIO_REG(METAL_SIFIVE_GPIO0_OUTPUT_VAL) |=(0x01<<p9);
	while(ultra_t<500)
	{
		metal_gettimeofday(&t2,NULL);
		ultra_t =t2.tv_usec-t1.tv_usec;
	}
//	GPIO_REG(METAL_SIFIVE_GPIO0_OUTPUT_VAL) &=~(0x01<<p9);
	GPIO_SET(p9,0,_OUTPUT);
	ultra_r = 1;
	return(3);
#endif

	// output a specified pulse to activate the sensor
	GPIO_SET(p9, 0, output);
	delay_us(20);
	GPIO_SET(p9, 1, output);
	delay_us(200);
	GPIO_SET(p9, 0, output);

	// change the gpio to the input state (p12 have been connected to p9)
	GPIO_REG(METAL_SIFIVE_GPIO0_OUTPUT_EN) &= ~(0X01 << p9);
	GPIO_REG(METAL_SIFIVE_GPIO0_INPUT_EN) |= (0x01 << p9);

	for (int a = 0; a < 2000; a++)
	{
		if (read_pin_val(p9) == 1)
		{
			ultra_r = 1;
			//metal_gettimeofday(&t1,NULL);
			t1 = get_timer_value_1();
			//printf("No\n");
			break;
		}
	}

	while (read_pin_val(p9) == 1)
	{
		//	printf("Yes\n");
	}
	t2 = get_timer_value_1();

	// compute the distance and return it
	if (ultra_r)
	{
		ultra_r = 0;
		//		metal_gettimeofday(&t2,NULL);
		//		ultra_t =t2.tv_usec-t1.tv_usec;
		ultra_t = t2 - t1;
		//		distance =(float)(ultra_t*340/RTC_FREQ/2 );
		distance = ((float)(t2 - t1) * 340) / RTC_FREQ / 2;

		distance_d = distance;

		//		printf("%.2f   \n \n",distance*100);
		//printf("%d\n",ultra_t);
		//printf("%d\n",distance_d);

		//motor_stop();
		return distance;
	}
	else
	{
		return 1000.0;
	}
}
void basic_control()
{// the car goes forward for more than 80cm
	for (int i = 1; i <= 310; i++)
	{
		PWM_run(50);
	}
	motor_stop();
	wait_ms(1000);

// turn left for 90 degrees
	for (int i = 1; i <= 79; i++)
	{
		PWM_right(50);
	}

	motor_stop();
	wait_ms(1000);

// the car goes forward for more than 80cm
	for (int i = 1; i <= 310; i++)
	{
		PWM_run(50);
	}
	motor_stop();
	wait_ms(1000);

// turn right for 90 degrees
	for (int i = 1; i <= 97; i++)
	{
		PWM_left(50);
	}
	motor_stop();
	wait_ms(1000);

// the car goes forward for more than 80cm
	for (int i = 1; i <= 310; i++)
	{
		PWM_run(50);
	}
	motor_stop();
	wait_ms(1000);

// the car goes back for more than 80cm
	for (int i = 1; i <= 310; i++)
	{
		PWM_back(50);
	}
	motor_stop();
	wait_ms(1000);
}

uint32_t key_scan(uint32_t pin_num)
{// check whether the key is pressed
	if ((GPIO_REG(0UL)) & (0x01 << pin_num))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
//FreeRTOS part
//============================================================================//
//============================================================================//

TaskHandle_t StartTask_Handler;
TaskHandle_t CarTask_Handler;
TaskHandle_t UltraTask_Handler;
TaskHandle_t PrintTask_Handler;

static void Print_task()
{
	const char *const message1 = "FreeRTOS Version 10.3.1\n";
	const char *const message2 = "Hi, Sifive!\n";
	write(STDOUT_FILENO, message1, strlen(message1));
	write(STDOUT_FILENO, message2, strlen(message2));

	vTaskDelete(NULL);
}

static void Car_task()
{
	int state = 0; 
	int global_state = 0; // record the direction the car faces
	int o_angle = 0; // record the angle the car has turned (original angle)
	//int finish = 0; //finish==1: the car hasn't avoided the obstacle; finish==0: the car has avoided the obstacle
	//int step=0; // the num of steps for which the car need to go forward when looking for the road between obstacles
	while (1)
	{
		tracking_car_control(50, &state, &global_state, &o_angle); // when the battery is full, speed should be set to 63
		vTaskDelay(1);
	}
}

static void Ultra_task()
{
	float result;
	BaseType_t err;
	while (1)
	{
		if (Message_Queue != NULL) // create queue successfully
		{
			result = ultrasonic(); // get the result of sensors
			err = xQueueSend(Message_Queue, &result, 0);
			if (err == errQUEUE_FULL) // if queue is full
			{
				write(STDOUT_FILENO, "Queue is full. Transmitting fails!\n", strlen("Queue is full. Transmitting fails!\n"));
			}
			else
			{
				//write(STDOUT_FILENO, "Sending successes!\n", strlen("Sending successes!\n"));
			}
		}
		else
		{
			write(STDOUT_FILENO, "Creating Queue task fails!\n", strlen("Creating Queue task fails!\n"));
		}
		//write(STDOUT_FILENO, "ultra successes!\n", strlen("ultra successes!\n"));
		vTaskDelay(1);
	}
}

static void Start_Task(void)
{
	taskENTER_CRITICAL();
	Message_Queue = xQueueCreate(4, sizeof(float));

	xTaskCreate(Print_task, "Print_Task", 200, NULL, 5, (TaskHandle_t *)&PrintTask_Handler);
	//xTaskCreate(Ultra_task, "Ultra_Task", 300, NULL, 3, (TaskHandle_t *)&UltraTask_Handler);
	xTaskCreate(Car_task, "Car_Task", 512, NULL, 4, (TaskHandle_t *)&CarTask_Handler);

	vTaskDelete(NULL);
	taskEXIT_CRITICAL();
}

int main()
{
	PLIC_init(&g_plic,
			  METAL_RISCV_PLIC0_C000000_BASE_ADDRESS,
			  PLIC_NUM_INTERRUPTS,
			  METAL_RISCV_PLIC0_C000000_RISCV_MAX_PRIORITY);
	//basic_control();
	smartcar_init();
	xTaskCreate(Start_Task, "Start_Task", 300, NULL, 1, (TaskHandle_t *)&StartTask_Handler);
	vTaskStartScheduler();
	return 0;
}
