// Uplinkrobotics PCB Rev A testing code
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
#include "ota_updates.h"

/***************************************************************************************************/

// define the rx and tx pins on the ESP
#define RXD2 16
#define TXD2 17

SBUS rxsr(Serial2); //SBUS object
uint16_t channels[16];
bool failSafe;
bool lostFrame;

Servo ext1;  // create servo object to external peripherals
Servo ext2;

Motors mot; // initialize motors
Wifi wifi; // initialize wifi code

// Radio channel integers - raw values from the radio
int ch1 = DEFAULT; // set to default value so crawler doesnt move on startup
int ch2 = DEFAULT; 
int ch3 = LOW_VAL;
int ch4 = LOW_VAL;
int ch5, ch6, ch7, ch8, ch9, ch10, ch11, ch12;

// Smoothed values to reduce current draw and make operation smoother
float thr_smoothed = 0;
float str_smoothed = 0;
float bat_smoothed = 11; // start at a nominal battery voltage

// Smoothing alpha values
float thr_alpha = 0.98;
float str_alpha = 0.96;
const float battery_alpha = 0.99;

// Radio channel values mapped into useful numbers
float thr, str, ext1float, ext2float;
int led2;
 
// Sensor values
int voltage_read = 0;
double battery_voltage = 0.0;

//LED array values
// led_array[8] = {orange, red, yellow, green, blue, red(rgb), green(rgb), blue(rgb)}
int led_array[8] = {0,0,0,0,0,0,0,0};

// Headlight boolean
bool headlights = 0;

// Region around neutral where the sticks don't give an output (no motion)
const int deadzone_thr = 6;
const int deadzone_str = 6;

int loop_timer = 0; // timer for main loop execution

// wifi update variables
int update_counter = 0;
int update_timer = millis();
int last_op_mode = 0; // integer for enabling wifi updates
int operating_mode = 0; // used to switch to Wifi mode

// Setup function that runs once as the ESP starts up
// ===================================================================================================
void setup() {
  // Setup the LED headlight/spotlight control, also in the motors library
  mot.ledc_init(LEDC_CHANNEL_1, LED2_IO, 12, 15625);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0); // headlights are off
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1); // apply the duty cycle

  Serial.begin(115200);     // debug info

  rxsr.begin(RXD2, TXD2, true, 100000); // optional parameters for ESP32: RX pin, TX pin, inverse mode, baudrate

  ext1.attach(GIMBAL_SERVO_IO); // attaches servo pin
  ext1.writeMicroseconds(0); // default position
  ext2.attach(GIMBAL_SERVO2_IO); // attaches servo pin
  ext2.writeMicroseconds(0); // default position

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
  voltage_read = analogRead(BATTERY_VOLT_IO);
  mot.sample_values();
  mot.overcurrent_right(); // perform overcurrent testing
  mot.overcurrent_left();

  // RE-MAP VALUES / MAKE VALUES CLEAN AND USEFUL
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Re-map all the rx into their useful ranges

  thr = constrain(map(ch1, LOW_VAL, HIGH_VAL, -105, 105), -100, 100); // throttle
  str = constrain(map(ch2, LOW_VAL, HIGH_VAL, -105, 105), -100, 100); // steering 
  ext1float = constrain(map(ch3, LOW_VAL, HIGH_VAL, 500, 2500), 500, 2500); // external control 1
  ext2float = constrain(map(ch4, LOW_VAL, HIGH_VAL, 500, 2500), 500, 2500); // external control 2
  // headlight toggle
  headlights = constrain(map(ch5, LOW_VAL, HIGH_VAL, -1, 3), 0, 1); // headlight toggle

  // operating mode switch for the WiFi mode
  operating_mode = constrain(map(ch7, LOW_VAL, HIGH_VAL, -1, 3), 0, 2);   // switch at least three times quickly to go into wifi mode

  // smooth out throttle, steering, and servo
  thr_smoothed = (thr_smoothed * thr_alpha) + (thr * (1 - thr_alpha));
  str_smoothed = (str_smoothed * str_alpha) + (str * (1 - str_alpha));
 
  // Calculate the voltage based on the analog value
  battery_voltage = (voltage_read / 319.5) + 1.6;
  bat_smoothed = (bat_smoothed * battery_alpha) + (battery_voltage * (1 - battery_alpha));

// WIFI CODE
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Safeguards to make wifi mode harder to stumble into
  if(update_counter > 0){ // check if the counter needs cleared
    if(update_timer + 1000 < millis()){
      update_counter = 0;
    }
  }
  // check for a full switch flip
  if(last_op_mode == 0 && operating_mode == 2 || last_op_mode == 2 && operating_mode == 0){
    update_counter++;
    update_timer = millis(); // timer till the counter resets
    last_op_mode = operating_mode;
  }
  // Over-The-Air Update code
  if(update_counter == 3){   // WiFi and update mode
    update_counter = 0;
    last_op_mode = operating_mode;
    mot.left_motors(0); // set channels to default to ensure the crawler stops in place
    mot.right_motors(0); 
    thr_smoothed = 0;   // zero these values so it doesn't start rolling after leaving the loop
    str_smoothed = 0;

    led_array[4] = 1;   // Turn on blue LED
    ledarray_set(led_array); 

    wifi.beginwifi();   // Turn on Wifi
    Serial.println("Waiting for connection with Mobile device...");
    update_timer = millis(); 
    while (last_op_mode == operating_mode){
      // if the mode changes, end the loop
      read_receiver(&ch1, &ch2, &ch3, &ch4, &ch5, &ch6, &ch7, &ch8, &ch9, &ch10, &ch11, &ch12);
      operating_mode = constrain(map(ch7, 172, 1811, -1, 3), 0, 2);    // 0 = normal operation, 2 = WiFi and update mode
      if(((update_timer + 3000) > millis()) && (operating_mode != 1)){ // 3 second timeout before you can leave wifi mode
        last_op_mode = operating_mode; // stay in this loop
      }
      if(operating_mode == 1) operating_mode = last_op_mode; // ignore middle switch state
      wifi.start_transmission(); // check for connection for transmission
      delay(200); // slow loop down
    }
    Serial.println("Shutting down Wifi");
    wifi.endwifi(); // turn off wifi
    led_array[4] = 0; // turn off blue LED
    ledarray_set(led_array); 
  }

  // ACTION TAKEN BASED VALUES / WRITE TO OUTPUTS
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // set the gimbal location
  ext1.writeMicroseconds(ext1float);
  ext2.writeMicroseconds(ext2float);

  //logarithmic lighting
  led2 = log_lighting(ch8);

  // control the LEDs. Had to inverse the values for some reason.
  if(headlights) ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, led2); // set the duty cycle for led channel 2
  else ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0); // Else headlights are off
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1); // apply the duty cycle

  // If battery is close to dead turn on red led
  if(battery_voltage < 9.9) led_array[1] = 1;
  else led_array[1] = 0;

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

// Function to apply logarithmic lighting to the LEDs
int log_lighting(int ch){
  if(ch < 173) return 0; // cut off anything below this point, don't waste current when LEDs aren't on
  float temp = ch * 0.00295; //lot of magic values specifically for SBUS
  temp = (exp(temp) + 25 + 0.154 * ch) * 8;
  return constrain(temp, 0, 4096);
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
    if(failSafe){
      *ch1 = DEFAULT;
      *ch2 = DEFAULT;
      *ch3 = LOW_VAL;
      *ch4 = LOW_VAL;
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
