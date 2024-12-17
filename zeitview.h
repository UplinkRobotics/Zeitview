// marten_pro.h
#ifndef MARTEN_LIBRARY
#define MARTEN_LIBRARY

/***************************************************************************************************/
// Marten specific information for logging purposes
#define CRAWLER_VERSION "Zeitview_1.0"
#define CRAWLER_NUM "1"
#define MANUFACTURE_DATE "12/16/24"

/***************************************************************************************************/
#define CURRENTHIGH 3000 // current reading which going above signifies overcurrent
#define OVERTIME 100 // time between overcurrent samples 

/***************************************************************************************************/

// define the rx and tx pins on the ESP
#define RXD2 16
#define TXD2 17

#define LOOP_TIME 2 // amount of minimum milliseconds in a loop, 1 = 1kHz, 2 = 500Hz

#define CURRENTHIGH 3000 // current reading which going above signifies overcurrent
#define OVERTIME 100 // time between overcurrent samples 

#define THR_CH 2
#define STR_CH 3
#define SER_CH 1
#define EXT_CH 0
#define LOW_VAL 172
#define HIGH_VAL 1811
#define GIMBAL_DEFAULT 991

#define DEFAULT (HIGH_VAL + LOW_VAL)/2 // get the middle value

// GPIO Pin Assignments
#define CAM_CTRL_IO 19 // camera select
#define GIMBAL_SERVO_IO 18 // gimbal servo
#define GIMBAL_SERVO2_IO 5 // gimbal servo 2

#define LED1_IO 4   // led channel 1 - Lower right, tied to button
#define LED2_IO 0   // led channel 2

#define LEFT_MOT_DIR_IO 14  // left motor direction
#define LEFT_MOT_PWM_IO 12  // left motor PWM
#define RIGHT_MOT_DIR_IO 32 // right motor direction
#define RIGHT_MOT_PWM_IO 33 // right motor PWM

#define MOTORS_NSLEEP_IO 15 // motor sleep pin

#define CURRENT_LM1_IO 36   // current sense left motor 1 (Sensor_VP)
#define CURRENT_LM2_IO 39   // current sense left motor 2 (Sensor_VN)
#define CURRENT_RM1_IO 34   // current sense right motor 1
#define CURRENT_RM2_IO 35   // current sense right motor 2

#define FAULT_IO 25         // fault sense - active low - low value indiates fault from one of the motor drivers

#define BATTERY_VOLT_IO 27     // analog pin to the battery level voltage

#define LEDARRAY_CLK_IO  13    // pin to the clock input of the shift register that controls the 8 leds, rising edge trigger
#define LEDARRAY_DATA_IO 2     // pin to the data input of the shift register that controls the 8 leds

#endif