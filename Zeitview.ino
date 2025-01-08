// Uplinkrobotics PCB Zeitview Code V1
//
//
// Copyright © 2024 UplinkRobotics LLC

// Include libraries
//#include <IBusBM.h>
#include "SBUS.h"
#include "crsf.h"
#include <ESP32Servo.h>
#include "driver/ledc.h"
#include "motor_library.h"
#include "zeitview.h"

/***************************************************************************************************/

// define the rx and tx pins on the ESP
#define RXD2 16
#define TXD2 17

SBUS rxsr(Serial2); //SBUS object
uint16_t channels[16];
bool failSafe;
bool lostFrame;

Servo ext1;  // create servo objects for external peripherals
Servo ext2;
Servo ext3;
Servo ext4;
Servo ext5;
Servo ext6;

Motors mot; // initialize motors

// Radio channel integers - raw values from the radio
int ch1 = DEFAULT; // set to default value so crawler doesnt move on startup
int ch2 = DEFAULT; 
int ch3 = DEFAULT;
int ch4 = DEFAULT;
int ch5 = LOW_VAL;
int ch6 = LOW_VAL;
int ch7 = LOW_VAL;
int ch8 = LOW_VAL;
int ch9 = LOW_VAL;
int ch10 = LOW_VAL;
int ch11 = LOW_VAL;
int ch12 = LOW_VAL;

// Smoothed values to reduce current draw and make operation smoother
float thr_smoothed = 0;
float str_smoothed = 0;

// Smoothing alpha values
float thr_alpha = 0.98;
float str_alpha = 0.96;

// Radio channel values mapped into useful numbers
float thr, str, ext1float, ext2float, ext3float, ext4float, ext5float, ext6float;

//LED array values
// led_array[8] = {orange, red, yellow, green, blue, red(rgb), green(rgb), blue(rgb)}
int led_array[8] = {0,0,0,0,0,0,0,0};

// Region around neutral where the sticks don't give an output (no motion)
const int deadzone_thr = 6;
const int deadzone_str = 6;

unsigned long loop_timer = 0; // timer for main loop execution

// Setup function that runs once as the ESP starts up
// ===================================================================================================
void setup() {
  // Setup the LED headlight/spotlight control, also in the motors library
  mot.ledc_init(LEDC_CHANNEL_1, LED2_IO, 12, 15625); // important ledc configuration for motors/LEDs

  Serial.begin(115200);     // debug info

  rxsr.begin(RXD2, TXD2, true, 100000); // optional parameters for ESP32: RX pin, TX pin, inverse mode, baudrate

  ext1.attach(GIMBAL_SERVO_IO);   // attaches servo pin
  ext2.attach(GIMBAL_SERVO2_IO);  // sevo2
  ext3.attach(CAM_CTRL_IO);       // cam control
  ext4.attach(VTX_CTRL_IO);       // vtx control
  ext5.attach(DB_SDA_IO);         // i2c sda
  ext6.attach(DB_SCL_IO);         // i2c scl
  ext1.writeMicroseconds(1000);   // off positon
  ext2.writeMicroseconds(1000);
  ext3.writeMicroseconds(1000);
  ext4.writeMicroseconds(1000); 
  ext5.writeMicroseconds(1000);
  ext6.writeMicroseconds(1000);

  pinMode(LEDARRAY_CLK_IO, OUTPUT); // on board indicator LEDs
  pinMode(LEDARRAY_DATA_IO, OUTPUT);

  // Setup the motors
  mot.setup(); 

  ledarray_set(led_array); // clear leds potentially still on from last run
}

// Loop that repeats forever within the ESP after setup runs once
// ===================================================================================================
void loop() {
  // READ VALUES / GATHER INFORMATION 
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Read all the raw values from the reciever and store into a variable
  read_receiver(&ch1, &ch2, &ch3, &ch4, &ch5, &ch6, &ch7, &ch8, &ch9, &ch10, &ch11, &ch12); // read values from the Receiver
  
  // Read sensor values
  mot.sample_values();
  mot.overcurrent_right(); // perform overcurrent testing
  mot.overcurrent_left();

  // RE-MAP VALUES / MAKE VALUES CLEAN AND USEFUL
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Re-map all the rx into their useful ranges

  thr = constrain(map(ch1, LOW_VAL, HIGH_VAL, -105, 105), -100, 100); // throttle
  str = constrain(map(ch2, LOW_VAL, HIGH_VAL, -105, 105), -100, 100); // steering 
  // headlight toggle
  ext1float = constrain(map(ch5, LOW_VAL, HIGH_VAL, 950, 2050), 1000, 2000); // external control 1
  ext2float = constrain(map(ch6, LOW_VAL, HIGH_VAL, 950, 2050), 1000, 2000); // external control 2
  ext3float = constrain(map(ch7, LOW_VAL, HIGH_VAL, 950, 2050), 1000, 2000); // external control 3
  ext4float = constrain(map(ch8, LOW_VAL, HIGH_VAL, 950, 2050), 1000, 2000); // external control 4
  ext5float = constrain(map(ch9, LOW_VAL, HIGH_VAL, 950, 2050), 1000, 2000); // external control 5
  ext6float = constrain(map(ch10, LOW_VAL, HIGH_VAL, 950, 2050), 1000, 2000); // external control 6

  // smooth out throttle, steering, and servo
  thr_smoothed = (thr_smoothed * thr_alpha) + (thr * (1 - thr_alpha));
  str_smoothed = (str_smoothed * str_alpha) + (str * (1 - str_alpha));

  // ACTION TAKEN BASED VALUES / WRITE TO OUTPUTS
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // set the RCPWM for each ext channel
  ext1.writeMicroseconds(ext1float);
  ext2.writeMicroseconds(ext2float);
  ext3.writeMicroseconds(ext3float);
  ext4.writeMicroseconds(ext4float);
  ext5.writeMicroseconds(ext5float);
  ext6.writeMicroseconds(ext6float);

  // if fault from motor drivers turn on yellow led
  if(mot.fault == 0) led_array[0] = 1;
  else led_array[0] = 0;
 
  // handle motor controls
  if ((thr_smoothed > -deadzone_thr && thr_smoothed < deadzone_thr) &&
      (str_smoothed > -deadzone_thr && str_smoothed < deadzone_thr)){
    mot.left_motors(0);
    mot.right_motors(0);
  }
  else{
    mot.left_motors(thr_smoothed + str_smoothed);
    mot.right_motors(thr_smoothed - str_smoothed);
  }

  ledarray_set(led_array);
 
  // check loop timer, make sure the loop isnt taking way too long
  if(millis() < LOOP_TIME + loop_timer) while(millis() < LOOP_TIME + loop_timer);
  loop_timer = millis(); // reset loop timer
}

// Function to set the values in the led array
// leds[8] = {orange, red, yellow, green, blue, red(rgb), green(rgb), blue(rgb)}
// 1 = led on, 0 = led off
void ledarray_set(int leds[]){
  for (int i = 0; i < 8; i++){
    digitalWrite(LEDARRAY_CLK_IO, LOW);
    digitalWrite(LEDARRAY_DATA_IO, leds[i]);
    digitalWrite(LEDARRAY_CLK_IO, HIGH);
  }
}

// Function to read values from the receiver 
void read_receiver(int *ch1, int *ch2, int *ch3, int *ch4, int *ch5, int *ch6, int *ch7, int *ch8, int *ch9, int *ch10, int *ch11, int *ch12){
  // look for a good SBUS packet from the receiver
  if(rxsr.read(&channels[0], &failSafe, &lostFrame)){
    *ch1 = channels[THR_CH];
    *ch2 = channels[STR_CH];
    *ch3 = channels[SER_CH];
    *ch4 = channels[EXT_CH];
    *ch5 = channels[4];
    *ch6 = channels[5];
    *ch7 = channels[6];
    *ch8 = channels[7];
    *ch9 = channels[8];
    *ch10 = channels[9];
    *ch11 = channels[10];
    *ch12 = channels[11];

    // sanitize values, usually only matters on controller turn off
    if(failSafe || lostFrame){
      *ch1 = DEFAULT;
      *ch2 = DEFAULT;
      *ch3 = DEFAULT;
      *ch4 = DEFAULT;
      *ch5 = LOW_VAL;
      *ch6 = LOW_VAL; 
      *ch7 = LOW_VAL; 
      *ch8 = LOW_VAL; 
      *ch9 = LOW_VAL; 
      *ch10 = LOW_VAL;
      *ch11 = LOW_VAL;
    }
  }
}
