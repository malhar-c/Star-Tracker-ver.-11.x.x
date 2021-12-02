/*
 * Star Tracker (Arduino + L298N Stepper Driver)
 * by Malhar Chakraborty (OuttaSyllabus)
 * date 07/03/2021 - started (version 0.1 full step)
 * version details:-
 * 0.5 micro stepping implementation
 * 2.0 1/4 microstepped
 *     cons- less torque
 * 2.1 increasing voltage to increase torque
 *     cons- less torque in intermediate microsteps
 * 3.0 coding the actual thing
 *     pros- 1. precise time display in serial monitor
 *           2. angle measurement of the final base with each microstep
 *           3. extra controls of motorised allignment and adjustment (using inturrupt)*
 * 3.1* scraping the idea of inturrupt implementation...
 * 4.0 millis() implementation for non bloking code
 * 5.0 remoteXY config
 *
 * 6.0 scrubbing idea of portability, switching to barn door type
 *    - Barn Door concept
 *    - 1 RPM drive
 *    - Quarter stepping (192 Steps per revolution)
 *    - 0.3125 s gap between steps
 *    - 312 milliseconds + 500 microseconds
 *
 * 6.1 plank size reduced
 *    - 0.5 RPM Drive
 *    - quarter stepping
 *    - intermediate gear (white gear with 60 teeth)
 *    - 0.01041666 s gap betweeen steps
 *    - 10 ms + 416 us
 *
 * 7.0 Scrubbed intermediate gear
 *    - scrubbed using of white gear because of wobble (the gear itself is wobbly)
 *    - back to direct motor drive (motor shaft to drive bolt)
 *    - quarter stepping
 *    - 625 ms gap between each micro steps (for 300mm focal length)
 *
 * 8.0 Switch to Half Stepping
 *    - not enough torque at 192 microsteps
 *    - switching to half steps instead (96 microsteps)
 *    - 1250 ms gap between steps
 *    - may be just enough for 300 mm focal length
 *
 ----Star Tracker SN2----
  10.0.1 Switch to Atom IDE and new RemoteXY UI
     - New GUI (simplified first page)
     - Provision to automatically stop the drive bolt (when reached limits)
     - one button fast rewind with protection.
     - better manual control.
     - STATUS page shows the status of the system, uptime, tracktime,
     motor stepping, position limit sensors, base angle, etc.

  10.0.2 fixed a bug which don't show the base angle on homepage
  10.0.3 to 10.0.x
  [FIXED] uptime resetting when stopping the tracking (first time)
  [FIXED] main status shows "STOPPED" always
  [FIXED] sensor status shows NULL
  [FIXED] virtual LEDs malfunction
  [FIXED] motor level indicator glitching

  10.1.0 Automatic Calibration
     - Adding a calibration function which will detect and calibrate the stepCount
     and base angle offset with the help of position/limit sensors/switches.
     - the calibration will run at the start everytime the star tracker is turned
     on and connected to wifi.

  10.1.1 [FIXED] Not connecting to wifi whenever calibration function is called
  10.1.2 to 10.1.x
  [FIXED] runtime error for unsigned data type of step count, now step count
  can be in negative initially, just because of calibration.
  [TWEAKED] virtual LEDs
  [TWEAKED] For calibration, no need to connect to wifi, as soon as the
            ESP is ready and AP is configed. it will start calibrating.
  [MAJOR PROBLEM] when wifi disconnects, tracking force reduces... (coil voltage reduces)

  *********************
  FULL TIME TEST 1.0
  *********************
  (with Li-ion (3s) not charged fully (~ 4.15 V))
  (this is a worse case senario test (not worst tho))
  - Total Runtime = ~3hr [still two bars left 25% to 50% ~3.7 V on each cell]
  - Max Track time = ~ 46 min
  - Total Steps = 2326
  - Total Base Angle = ~ 12 degrees
  - Battery level after first 1 hr = 50% - 75% (3 solid bars)
  - Battery level after first 2 hrs = ~ 50%  (3rd bar not stable)
  - Disconnects = few (because of my moron-ness)
  - High Speed Rewind Time = ~15 sec

  **Problem(s) analysis and solutions (to do)
  - Lower limit detection error (contacts not touching)
  - [fix idea] raise the height of the contact (solder pading may be)
  - For Rewind function there is two conditions to stop the motor
    1. Limit Sensor (may fail)
    2. Step count == 0 (can't fail) but aparently this is not working.
  - should do calibration after every rewind.
  - base degree should be added with the base angle offset.
  - as the top plank load increases, driving becomes harder as the drive ball
  sticks with the top pvc layer.
  - if wifi disconnect drive force reduces.
  - when max limit is reached, the motor still gets power (in holding mode)
  - tracking text fluctuates.
  - built in LED glows all the time.

  **** base angle calibration ****
  (all are with the axis of rotation)
  - bottom plank = 3 degrees
  - top plank = 17 degrees max***
  - top plank = -- degrees min***
  [*** to redo]

10.2.x (First Run problem fixes and improvements)**
 - [FIXED] Lower limit switch/sensor solder padded (height increased)
 - [EXPECTATION] now the max tracking time should reduce a lil bit (as the
lower limit height increased)
 - [IMPROVED] (Still to test whether fixed or not) drive bolt top touching
 surface re-enforced with M-Seal and silicone lubed (surface smoothed too)
 now the drive head should not stick as the top plank load increases.

 **** [NEW] base angle calibration ****
 (all are with the axis of rotation)
 - bottom plank = 0 degrees [CALIBRATED]
 - top plank = 15.05 degrees max
 - top plank = 4.6 degrees min

 ver. 11.0.0 (UI changes and new features)
  - UI version SN2.1
  - RemoteXY lib. ver. 3.1.6
  - will turn on the fan when its needed
  - live temperature readings of the ESP and motor driver
  - turn on (or off) the fan automatically
  - manual fan control as well (failsafe)

11.0.x
  [FIXED] Glitch in fan on/off and status display
  [FIXED] Temperature readings changing drastically (jumping all over the place)
  now, the main variable will take an average of 200 samples and process according
  to that.
  [NOTE*] Temperature sensors calibrated wrt 12V adaptor
  [ADDED] PWM control for the fan, the fan now turns on, adjusts speed
  according to the temperature of the wifi mod and motor driver.
  [PROBLEM*] Temperature sensor values changes as the input voltage changes

  *doubtful, may have to revisit once everything is running normally with battery

 */


#define REMOTEXY_MODE__ESP8266_HARDSERIAL_POINT

#include <Arduino.h>
#include <RemoteXY.h>
#include <math.h>

// RemoteXY connection settings
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 115200
#define REMOTEXY_WIFI_SSID "Star_Tracker_SN2-1"
#define REMOTEXY_WIFI_PASSWORD "tintin123"
#define REMOTEXY_SERVER_PORT 6377

// Star Tracker configs
#define A           4                     // the pin connected to the wire A of the coil A (or to the H-bridge pin controlling the same wire)
#define A_bar       3                     // the pin connected to the wire A- of the coil A (or to the H-bridge pin controlling the same wire)
#define B           7                     // the pin connected to the wire B of the coil A (or to the H-bridge pin controlling the same wire)
#define B_bar       8                     // the pin connected to the wire B- of the coil A (or to the H-bridge pin controlling the same wire)

#define CoilA_PWM   5                    // PWM V control for coil A
#define CoilB_PWM   6                    // PWM V control for coil B

#define Fan_PWM     11                   // PWM control pin for Fan
#define Min_temp    45                   // min threshold temperature (in degree C) for turning the fan on
#define Max_temp    52                   // max temperature (in degree C) where fan speed is 100%

#define x           0                  // delay(microseconds) between steps
#define y           1250                    //delay(milliseconds) between steps

#define max_limit   2                  // max limit switch (for position sensing)
#define min_limit   9                 // min limit switch (for position sensing)

#define min_angle   4.8              // minimum plank angle (in lower limit)

#define stepsPerRevolution 96           // number of microsteps per revolution

#define max_P        255                //this is the max power
#define min_P        175                  //min power (coil turned off)
#define int_max_P    160      //160    //intermediate max power (intermediate steps between two half steps)
#define int_min_P    110      //100    //intermediate max power (intermediate steps between two half steps)

long stepCount = 0; // number of steps the motor has taken

//these flags are for detecting whether the panks have reached their limits
bool max_lim_flag = 0;
bool min_lim_flag = 0;

unsigned long man_prev_millis = 0;
unsigned long man_curr_millis = 0;
int ch1 = 1;                          //for full step drive
unsigned short del_b_fullstep = 0;                //interval between the full steps as set by the speed selector
int man_step_c = 0;                   //manual steps counter

unsigned long previousMillis = 0;
float rotation = 0;
float deg = 0;
int ch = 1;
int rot_met = 0;

//variables for on board time keeping (tracking)
unsigned int hrs = 0;
unsigned int minutes = 0;
unsigned int seconds = 0;
unsigned long time_hold = 0;
unsigned long reset_time = 0;

//for uptime keeping (will never reset when the arduino is powered on)
unsigned int up_hrs = 0;
unsigned int up_minutes = 0;
unsigned int up_seconds = 0;
unsigned long up_time_hold = 0;
unsigned long up_reset_time = 0;

//for virtual LEDs
unsigned long previousMillisLED = 0;
unsigned long currentMillisLED = 0;

//for calibration
bool Cal_flag = 0;

//for live temperature measurement
const int esp01_t_in = A0;           //esp01 temp sensor connected to A0
const int L298n_t_in = A1;           //L298N temp sensor connected to A1
const float InternalReferenceVoltage = 1.080;  //measured ref for calibration
const float t0 = 30.8;              // Calibration ambient temperature
// Calibrated forward voltages of the Diodes
const float esp01_vf0 = 0.7445;
const float L298n_vf0 = 0.5685;
// Variables for storing and intermediate calculations
float esp01_dtemp;
float esp01_t;
float esp01_vf = 0;
float L298n_dtemp;
float L298n_t;
float L298n_vf = 0;
// for averaging, displaying and processing
unsigned int esp_temp_avg_sum = 0;
unsigned int L298n_temp_avg_sum = 0;
unsigned short esp_t_avg = 0;
unsigned short L298n_t_avg = 0;
unsigned short sample_count = 0;
// for Fam PWM control and speed display
unsigned short Fan_PWM_value = 0;
unsigned short Fan_speed = 0;
bool fan_flag = 0;


// RemoteXY configurate
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,6,0,177,0,168,3,11,25,5,
  130,1,3,14,19,39,2,25,66,194,
  4,5,55,43,1,180,173,67,5,9,
  49,45,8,1,36,25,11,2,1,19,
  69,25,9,1,36,172,30,8,83,84,
  65,82,84,0,83,84,79,80,0,67,
  5,22,29,20,5,1,181,25,11,131,
  1,1,94,18,5,1,174,31,72,79,
  77,69,0,131,0,20,94,21,5,2,
  174,31,65,68,74,85,83,84,0,131,
  0,42,94,20,5,3,174,31,83,84,
  65,84,85,83,0,129,0,3,11,52,
  3,3,36,84,114,97,99,107,101,114,
  32,99,111,110,102,105,103,46,58,32,
  78,111,114,116,104,101,114,110,32,72,
  101,109,105,115,112,104,101,114,101,0,
  129,0,3,3,34,3,3,36,85,73,
  32,118,101,114,46,58,32,83,116,97,
  114,32,84,114,97,99,107,101,114,32,
  83,78,50,46,49,0,129,0,3,7,
  36,3,3,36,67,111,100,101,32,118,
  101,114,46,58,32,49,49,46,120,46,
  120,32,111,110,119,97,114,100,115,0,
  129,0,3,15,50,3,3,36,77,111,
  116,111,114,32,99,111,110,102,105,103,
  46,58,32,72,97,108,102,32,83,116,
  101,112,112,105,110,103,32,40,57,54,
  47,114,101,118,46,41,0,129,0,3,
  24,27,4,3,180,85,112,116,105,109,
  101,32,32,32,32,32,32,32,32,32,
  32,32,32,58,0,67,4,31,23,21,
  6,3,180,25,11,129,0,3,30,27,
  3,3,194,84,114,97,99,107,32,84,
  105,109,101,32,32,32,32,32,32,32,
  32,32,32,32,32,32,32,58,0,67,
  4,31,30,24,4,3,194,25,11,66,
  129,4,83,25,4,3,180,173,129,0,
  3,34,27,3,3,194,80,111,115,105,
  116,105,111,110,32,83,101,110,115,111,
  114,115,32,32,32,58,0,67,4,31,
  34,24,4,3,194,25,11,129,0,3,
  38,27,3,3,194,83,116,97,116,117,
  115,32,32,32,32,32,32,32,32,32,
  32,32,32,32,32,32,32,32,32,32,
  32,32,58,0,67,4,31,38,24,4,
  3,194,25,11,129,0,3,42,27,3,
  3,194,66,97,115,101,32,97,110,103,
  108,101,32,32,32,32,32,32,32,32,
  32,32,32,32,32,58,0,67,4,31,
  42,19,4,3,194,25,11,68,49,32,
  69,28,20,3,24,194,83,116,101,112,
  115,0,129,0,4,78,25,4,3,194,
  77,111,116,111,114,32,83,116,97,116,
  117,115,58,0,65,7,29,61,4,4,
  1,129,0,3,19,27,3,3,36,84,
  105,109,101,32,71,97,112,32,58,32,
  49,50,53,48,32,109,115,0,3,3,
  10,22,8,22,2,36,24,1,2,33,
  37,19,12,2,36,31,82,101,119,105,
  110,100,0,65,7,40,28,4,4,2,
  129,0,11,46,7,3,2,194,76,79,
  87,0,129,0,10,18,7,3,2,194,
  72,73,71,72,0,129,0,5,22,3,
  5,2,175,83,0,129,0,5,27,3,
  4,2,175,80,0,129,0,5,31,3,
  4,2,175,69,0,129,0,5,35,3,
  4,2,175,69,0,129,0,5,40,3,
  4,2,175,68,0,67,5,3,3,57,
  8,2,36,25,11,66,194,3,66,20,
  16,2,180,173,67,5,3,84,20,5,
  2,180,25,11,129,0,28,61,30,4,
  2,180,77,97,110,117,97,108,32,67,
  111,110,116,114,111,108,115,0,1,0,
  30,70,12,12,2,131,31,79,112,101,
  110,0,1,0,43,70,12,12,2,131,
  31,67,108,111,115,101,0,67,5,32,
  17,20,5,2,180,25,11,129,0,3,
  49,27,3,3,39,69,83,80,48,49,
  32,84,101,109,112,32,40,194,176,67,
  41,32,32,32,58,0,129,0,3,54,
  26,3,3,39,76,50,57,56,78,32,
  84,101,109,112,32,40,194,176,67,41,
  32,32,32,58,0,67,4,31,49,19,
  4,3,39,25,11,67,4,31,54,19,
  4,3,39,25,11,129,0,3,59,26,
  3,3,39,70,97,110,32,83,116,97,
  116,117,115,32,32,32,32,32,32,32,
  32,32,32,32,32,32,58,0,129,0,
  6,66,18,2,3,36,77,97,110,117,
  97,108,32,70,97,110,32,67,111,110,
  116,114,111,108,0,2,1,9,69,13,
  5,3,35,172,31,31,79,78,0,79,
  70,70,0,67,4,31,59,19,4,3,
  39,25,11,67,5,39,51,21,3,3,
  36,25,21 };

// this structure defines all the variables and events of your control interface
struct {

    // input variables
  uint8_t switch_1; // =1 if switch ON and =0 if OFF
  uint8_t select_1; // =0 if select position A, =1 if position B, =2 if position C, ...
  uint8_t button_1; // =1 if button pressed, else =0
  uint8_t open_button; // =1 if button pressed, else =0
  uint8_t close_button; // =1 if button pressed, else =0
  uint8_t fan_switch; // =1 if switch ON and =0 if OFF

    // output variables
  int8_t base_level; // =0..100 level position
  char status[11];  // string UTF8 end zero
  char base_degree[11];  // string UTF8 end zero
  char uptime[11];  // string UTF8 end zero
  char time[11];  // string UTF8 end zero
  int8_t motor_level; // =0..100 level position
  char position_2[11];  // string UTF8 end zero
  char status_2[11];  // string UTF8 end zero
  char base_degree_2[11];  // string UTF8 end zero
  float steps_graph;
  uint8_t status_led_r; // =0..255 LED Red brightness
  uint8_t status_led_g; // =0..255 LED Green brightness
  uint8_t status_led_b; // =0..255 LED Blue brightness
  uint8_t status_led_1_r; // =0..255 LED Red brightness
  uint8_t status_led_1_g; // =0..255 LED Green brightness
  uint8_t status_led_1_b; // =0..255 LED Blue brightness
  char status_1[11];  // string UTF8 end zero
  int8_t base_level_1; // =0..100 level position
  char base_degree_1[11];  // string UTF8 end zero
  char position_1[11];  // string UTF8 end zero
  char esp01_temp[11];  // string UTF8 end zero
  char L298n_temp[11];  // string UTF8 end zero
  char fan_status[11];  // string UTF8 end zero
  char temp_warning[21];  // string UTF8 end zero

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

//all function prototypes
void update_everything();
void update_time();
void update_uptime();
void check_limits();
void Track();
void reset();
void blink_wd_red(unsigned short);
void blink_wd_green(unsigned short);
void Rewind();
void Adjust();
void speed_select();
void CW();
void CCW();
void turn_off_LEDs();
void Calibrate();
void Activate_motors();   //only use for manual and full step drives
void Deactivate_motors();  //only use for manual and full step drives
void update_temps();
void control_fan();

void setup()
{
  RemoteXY_Init ();

  //motor PWM controls
  pinMode(CoilA_PWM, OUTPUT);
  pinMode(CoilB_PWM, OUTPUT);

  pinMode (Fan_PWM, OUTPUT);

  //motor coils
  pinMode(A, OUTPUT);
  pinMode(A_bar, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(B_bar, OUTPUT);

  //limit switches
  pinMode(max_limit, INPUT_PULLUP);
  pinMode(min_limit, INPUT_PULLUP);

  //motor PWM controls
  analogWrite(CoilA_PWM, 0);
  analogWrite(CoilB_PWM, 0);

  //temperature sensors
  pinMode(esp01_t_in, INPUT_PULLUP);
  pinMode(L298n_t_in, INPUT_PULLUP);
  analogReference (INTERNAL);
  analogRead (esp01_t_in);
  analogRead (L298n_t_in);
}

void loop()
{
  RemoteXY_Handler ();

  update_everything();

  //this will run once when the arduino is switched on and wifi configed.
  while(Cal_flag == 0)
  {
    Cal_flag = 1;
    Calibrate();
  }

  //if max limit reached
  if (max_lim_flag == 1)
  {
    //turn_off_LEDs();
    blink_wd_red(1000);
    RemoteXY.switch_1 = 0; //automatically turn off UI switch
    Deactivate_motors();
    sprintf(RemoteXY.status, "CHK limits");
    sprintf(RemoteXY.status_1, "CHK limits");
    sprintf(RemoteXY.status_2, "CHK limits");
    sprintf(RemoteXY.position_1, "MAX LIMIT");
    sprintf(RemoteXY.position_2, "MAX LIMIT");

    Rewind();//rewind button activated
  }

  else
  {
    turn_off_LEDs();
    Deactivate_motors();
    sprintf(RemoteXY.status, "Ready");
    sprintf(RemoteXY.status_1, "Ready");
    sprintf(RemoteXY.status_2, "Ready");
    if (min_lim_flag != 1)
    {
      sprintf(RemoteXY.position_1, "NOMINAL");
      sprintf(RemoteXY.position_2, "NOMINAL");
    }
   //BELOW ELSE is not tested (to update min limit status of position sensor)
    else
    {
      sprintf(RemoteXY.position_1, "MIN LIMIT");
      sprintf(RemoteXY.position_2, "MIN LIMIT");
    }
    blink_wd_green(250);
    if(RemoteXY.switch_1 == 1)
    {
      RemoteXY.status_led_g = 255;
      RemoteXY.status_led_1_g = 255;
      Track();
      update_time();
      digitalWrite(13, HIGH);
      sprintf(RemoteXY.status, "Tracking");
      sprintf(RemoteXY.status_1, "Tracking");
      sprintf(RemoteXY.status_2, "Tracking");
    }

    else
    {
      blink_wd_green(250);
      reset();
      update_everything();
      digitalWrite(13, LOW);
      sprintf(RemoteXY.status, "Ready");
      sprintf(RemoteXY.status_1, "Ready");
      sprintf(RemoteXY.status_2, "Ready");

      Adjust(); //for manual control
    }
  }
}

void Activate_motors()
{
  analogWrite(CoilA_PWM, 200);
  analogWrite(CoilB_PWM, 200);
}

void Deactivate_motors()
{
  analogWrite(CoilA_PWM, 0);
  analogWrite(CoilB_PWM, 0);
}

void update_temps()
{
  esp01_vf = ((float) analogRead(esp01_t_in) + 0.5) / 1024.0 * InternalReferenceVoltage;
  esp01_dtemp = (esp01_vf - esp01_vf0) / 0.00150;
  esp01_t = t0 - esp01_dtemp;

  L298n_vf = ((float) analogRead(L298n_t_in) + 0.5) / 1024.0 * InternalReferenceVoltage;
  L298n_dtemp = (L298n_vf - L298n_vf0) / 0.0022;
  L298n_t = t0 - L298n_dtemp;

  if (sample_count < 200)
  {
    ++sample_count;
    esp_temp_avg_sum += (int)esp01_t;
    L298n_temp_avg_sum += (int)L298n_t;
  }

  if (sample_count == 200)
  {
    sample_count = 0;
    esp_t_avg = esp_temp_avg_sum/200;
    L298n_t_avg = L298n_temp_avg_sum/200;
    sprintf(RemoteXY.esp01_temp , "%d" , esp_t_avg);
    sprintf(RemoteXY.L298n_temp , "%d" , L298n_t_avg);
    esp_temp_avg_sum = 0;
    L298n_temp_avg_sum = 0;
  }

}

void control_fan()
{
  if (RemoteXY.fan_switch == 0)
  {
    if (max(esp_t_avg, L298n_t_avg) >= 48)
    {
      fan_flag = 1;
    }

    else if (max(esp_t_avg, L298n_t_avg) <= Min_temp)
    {
      fan_flag = 0;
    }

    if (fan_flag == 1)
    {
      sprintf(RemoteXY.fan_status, "ON (%d%%)", Fan_speed);
      analogWrite(Fan_PWM, Fan_PWM_value);
    }

    else
    {
      sprintf(RemoteXY.fan_status, "OFF");
      analogWrite(Fan_PWM, 0);
    }

    Fan_PWM_value = map(max(esp_t_avg, L298n_t_avg), Min_temp, Max_temp, 60, 255);
    Fan_speed = map(Fan_PWM_value, 0, 255, 0, 100);

    if (max(esp_t_avg, L298n_t_avg) > Max_temp)
    {
      Fan_PWM_value = 255;
      Fan_speed = 100;
      sprintf(RemoteXY.temp_warning, "WARNING");
    }
    else
    {
      sprintf(RemoteXY.temp_warning, "NOMINAL");
    }

  }

  else
  {
    analogWrite(Fan_PWM, 255);
    sprintf(RemoteXY.fan_status, "(MAN) ON");
  }
}

//this will run everytime at start to iniitalize the step count
void Calibrate()
{
  update_everything();
  sprintf(RemoteXY.status, "--Cal--");
  sprintf(RemoteXY.status_1, "--Cal--");
  sprintf(RemoteXY.status_2, "--Cal--");
  RemoteXY.status_led_b = 255;
  if (max_lim_flag != 1)
  {
    del_b_fullstep = 10;
    Activate_motors();
    for(int i=0; i<250; ++i)
    {
      update_everything();
      CCW();
      if (max_lim_flag == 1)
      {
        Deactivate_motors();
        break;
      }
    }
    update_everything();
  }

  del_b_fullstep = 20;
  do
  {
    update_everything();
    Activate_motors();
    CW();
  } while(min_lim_flag != 1);
  stepCount = 0;
  Deactivate_motors();
  sprintf(RemoteXY.position_1, "MIN LIMIT");
  sprintf(RemoteXY.position_2, "MIN LIMIT");
}

void speed_select()
{
  switch(RemoteXY.select_1)
  {
    case 0 :
    del_b_fullstep = 10;
    break;

    case 1 :
    del_b_fullstep = 50;
    break;

    case 2 :
    del_b_fullstep = 100;
    break;

    default:
    break;
  }
}

void Adjust()
{
  speed_select();
  update_everything();
  turn_off_LEDs();
  while (RemoteXY.open_button == 1)
  {
    update_everything();
    if (max_lim_flag != 1)
    {
      Activate_motors();
      CCW();
    }
    else
    {
      Deactivate_motors();
      sprintf(RemoteXY.position_1, "MAX LIMIT");
      sprintf(RemoteXY.position_2, "MAX LIMIT");
      RemoteXY.status_led_r = 255;
      RemoteXY.status_led_1_r = 255;
    }
  }

  while (RemoteXY.close_button == 1)
  {
    update_everything();
    if (min_lim_flag != 1)
    {
      Activate_motors();
      CW();
    }
    else
    {
      Deactivate_motors();
      sprintf(RemoteXY.position_1, "MIN LIMIT");
      sprintf(RemoteXY.position_2, "MIN LIMIT");
      RemoteXY.status_led_r = 255;
      RemoteXY.status_led_1_r = 255;
    }
  }
}

void Rewind()
{
  speed_select();
  if(RemoteXY.button_1 == 1)
  {
    Activate_motors();
    do
    {
      //rotate the motor clockwise at set speed through speed select
      CW();
    } while(min_lim_flag != 1);
    //until the plank reaches min limit or the step count becomes 0
    sprintf(RemoteXY.position_1, "MIN LIMIT");
    sprintf(RemoteXY.position_2, "MIN LIMIT");
    Deactivate_motors();
    Calibrate();
  }
}

void CW()
{
  //full step drive clock wise (48/Rev)
  switch(ch1)
  {
    man_curr_millis = millis();
    case 1:
    {
      man_curr_millis = millis();
      if (man_curr_millis - man_prev_millis >= del_b_fullstep)
      {
        man_prev_millis = man_curr_millis;
        digitalWrite(A, HIGH);
        digitalWrite(A_bar, LOW);
        digitalWrite(B, LOW);
        digitalWrite(B_bar, HIGH);
        stepCount-=2;
        update_everything();
        ++ch1;
      }
      break;
    }

    case 2:
    {
      man_curr_millis = millis();
      if (man_curr_millis - man_prev_millis >= del_b_fullstep)
      {
        man_prev_millis = man_curr_millis;
        digitalWrite(A, LOW);
        digitalWrite(A_bar, HIGH);
        digitalWrite(B, LOW);
        digitalWrite(B_bar, HIGH);
        stepCount-=2;
        update_everything();
        ++ch1;
      }
      break;
    }

    case 3:
    {
      man_curr_millis = millis();
      if (man_curr_millis - man_prev_millis >= del_b_fullstep)
      {
        man_prev_millis = man_curr_millis;
        digitalWrite(A, LOW);
        digitalWrite(A_bar, HIGH);
        digitalWrite(B, HIGH);
        digitalWrite(B_bar, LOW);
        stepCount-=2;
        update_everything();
        ++ch1;
      }
      break;
    }

    case 4:
    {
      man_curr_millis = millis();
      if (man_curr_millis - man_prev_millis >= del_b_fullstep)
      {
        man_prev_millis = man_curr_millis;
        digitalWrite(A, HIGH);
        digitalWrite(A_bar, LOW);
        digitalWrite(B, HIGH);
        digitalWrite(B_bar, LOW);
        stepCount-=2;
        update_everything();
        ch1 = 1;
      }
      break;
    }
  }
}

void CCW()
{
  //Full Step Drive anticlock wise (48 steps per rev)
  switch(ch1)
  {
    man_curr_millis = millis();

    case 1:
    {
      man_curr_millis = millis();
      if (man_curr_millis - man_prev_millis >= del_b_fullstep)
      {
        man_prev_millis = man_curr_millis;
        digitalWrite(A, HIGH);
        digitalWrite(A_bar, LOW);
        digitalWrite(B, HIGH);
        digitalWrite(B_bar, LOW);
        stepCount+=2;
        update_everything();
        ++ch1;
      }
      break;
    }

    case 2:
    {
      man_curr_millis = millis();
      if (man_curr_millis - man_prev_millis >= del_b_fullstep)
      {
        man_prev_millis = man_curr_millis;
        digitalWrite(A, LOW);
        digitalWrite(A_bar, HIGH);
        digitalWrite(B, HIGH);
        digitalWrite(B_bar, LOW);
        stepCount+=2;
        update_everything();
        ++ch1;
      }
      break;
    }

    case 3:
    {
      man_curr_millis = millis();
      if (man_curr_millis - man_prev_millis >= del_b_fullstep)
      {
        man_prev_millis = man_curr_millis;
        digitalWrite(A, LOW);
        digitalWrite(A_bar, HIGH);
        digitalWrite(B, LOW);
        digitalWrite(B_bar, HIGH);
        stepCount+=2;
        update_everything();
        ++ch1;
      }
      break;
    }

    case 4:
    {
      man_curr_millis = millis();
      if (man_curr_millis - man_prev_millis >= del_b_fullstep)
      {
        man_prev_millis = man_curr_millis;
        digitalWrite(A, HIGH);
        digitalWrite(A_bar, LOW);
        digitalWrite(B, LOW);
        digitalWrite(B_bar, HIGH);
        stepCount+=2;
        update_everything();
        ch1 = 1;
      }
      break;
    }
  }
}

void blink_wd_red(unsigned short delay_milli)
{
  currentMillisLED = millis();
  RemoteXY.status_led_g = 0;
  RemoteXY.status_led_b = 0;
  if (currentMillisLED - previousMillisLED >= delay_milli)
  {
    previousMillisLED = currentMillisLED;

    if (RemoteXY.status_led_r == 0)
    {
      RemoteXY.status_led_r = 255;
    }
    else
    {
      RemoteXY.status_led_r = 0;
    }
  }
}

void blink_wd_green(unsigned short delay_milli)
{
  currentMillisLED = millis();
  RemoteXY.status_led_r = 0;
  RemoteXY.status_led_b = 0;
  if (currentMillisLED - previousMillisLED >= delay_milli)
  {
    previousMillisLED = currentMillisLED;

    if (RemoteXY.status_led_g == 0)
    {
      RemoteXY.status_led_g = 255;
    }
    else
    {
      RemoteXY.status_led_g = 0;
    }
  }
}

void turn_off_LEDs()
{
  RemoteXY.status_led_r = 0;
  RemoteXY.status_led_g = 0;
  RemoteXY.status_led_b = 0;
  RemoteXY.status_led_1_r = 0;
  RemoteXY.status_led_1_g = 0;
  RemoteXY.status_led_1_b = 0;
}

void reset()
{
  reset_time = millis();       //this will reset the clock panel
  ch=1;                      //this will reset the microstep initialization
}

void update_time()
{
  time_hold = millis() - reset_time;
  seconds = time_hold/1000;
  hrs = seconds / 3600;
  minutes = seconds % 3600 / 60;
  sprintf(RemoteXY.time, "%3d:%02d:%02d", hrs, minutes,(seconds % 60));
}

//uptime should never reset when then powered on
void update_uptime()
{
  up_time_hold = millis();
  up_seconds = up_time_hold/1000;
  up_hrs = up_seconds / 3600;
  up_minutes = up_seconds % 3600 / 60;
  sprintf(RemoteXY.uptime, "%3d:%02d:%02d", up_hrs, up_minutes,(up_seconds % 60));
}

void update_everything()
{
  RemoteXY_Handler ();
  deg = min_angle + (atan(stepCount/(114.6*96)))*(180/M_PI);
  RemoteXY.motor_level = (stepCount%96);
  RemoteXY.steps_graph = stepCount;
  update_uptime();
  check_limits(); //to update the limit flags
  update_temps();
  control_fan();

  dtostrf(deg, 2, 5, RemoteXY.base_degree); //updation of base_degree (homepage)
  dtostrf(deg, 2, 5, RemoteXY.base_degree_1); //updation of base_degree (adjust page)
  dtostrf(deg, 2, 5, RemoteXY.base_degree_2); //updation of base_degree (status page)

  //plank base angle indicator updation
  RemoteXY.base_level = deg;
  RemoteXY.base_level_1 = deg;

  //Update the status LED
  RemoteXY.status_led_1_r = RemoteXY.status_led_r;
  RemoteXY.status_led_1_g = RemoteXY.status_led_g;
  RemoteXY.status_led_1_b = RemoteXY.status_led_b;
}

void check_limits()
{
  if (digitalRead(max_limit) == LOW)
  {
    max_lim_flag = 1;
  }
  else
  {
    max_lim_flag = 0;
  }

  if (digitalRead(min_limit) == LOW)
  {
    min_lim_flag = 1;
  }
  else
  {
    min_lim_flag = 0;
  }
}

void Track()
{
  unsigned long currentMillis; // = millis();
  switch(ch)
  {
    currentMillis = millis();
    case 1:
    {
      currentMillis = millis();
      if (currentMillis - previousMillis >= y)
      {
        previousMillis = currentMillis;
        analogWrite(CoilA_PWM, max_P);
        analogWrite(CoilB_PWM, 0);
        digitalWrite(A, HIGH);
        digitalWrite(A_bar, LOW);
        digitalWrite(B, LOW);
        digitalWrite(B_bar, LOW);
        ++stepCount;

        update_everything();

        //update track time
        update_time();
        ++ch;
      }
      break;
    }

    case 2:
    {
    currentMillis = millis();
    if (currentMillis - previousMillis >= y)
    {
      previousMillis = currentMillis;
      analogWrite(CoilA_PWM, min_P);
      analogWrite(CoilB_PWM, min_P);
      digitalWrite(A, HIGH);
      digitalWrite(A_bar, LOW);
      digitalWrite(B, HIGH);
      digitalWrite(B_bar, LOW);
      ++stepCount;

      update_everything();

      //update track time
      update_time();
      ++ch;
    }
    break;
    }

    case 3:
    {

    currentMillis = millis();
    if (currentMillis - previousMillis >= y)
    {
      previousMillis = currentMillis;
      analogWrite(CoilA_PWM, 0);
      analogWrite(CoilB_PWM, max_P);
      digitalWrite(A, LOW);
      digitalWrite(A_bar, LOW);
      digitalWrite(B, HIGH);
      digitalWrite(B_bar, LOW);
      ++stepCount;

      update_everything();

      //update track time
      update_time();
      ++ch;
    }
    break;
    }

    case 4:
    {

    currentMillis = millis();
    if (currentMillis - previousMillis >= y)
    {
      previousMillis = currentMillis;
      analogWrite(CoilA_PWM, min_P);
      analogWrite(CoilB_PWM, min_P);
      digitalWrite(A, LOW);
      digitalWrite(A_bar, HIGH);
      digitalWrite(B, HIGH);
      digitalWrite(B_bar, LOW);
      ++stepCount;
      update_everything();

      //update track time
      update_time();
      ++ch;
    }
    break;
    }

    case 5:
    {

    currentMillis = millis();
    if (currentMillis - previousMillis >= y)
    {
      previousMillis = currentMillis;
      analogWrite(CoilA_PWM, max_P);
      analogWrite(CoilB_PWM, 0);
      digitalWrite(A, LOW);
      digitalWrite(A_bar, HIGH);
      digitalWrite(B, LOW);
      digitalWrite(B_bar, LOW);
      ++stepCount;

      update_everything();

      //update track time
      update_time();
      ++ch;
    }
    break;
    }

    case 6:
    {

    currentMillis = millis();
    if (currentMillis - previousMillis >= y)
    {
      previousMillis = currentMillis;
      analogWrite(CoilA_PWM, min_P);
      analogWrite(CoilB_PWM, min_P);
      digitalWrite(A, LOW);
      digitalWrite(A_bar, HIGH);
      digitalWrite(B, LOW);
      digitalWrite(B_bar, HIGH);
      ++stepCount;

      update_everything();

      //update track time
      update_time();
      ++ch;
    }
    break;
    }

    case 7:
    {

    currentMillis = millis();
    if (currentMillis - previousMillis >= y)
    {
      previousMillis = currentMillis;
      analogWrite(CoilA_PWM, 0);
      analogWrite(CoilB_PWM, max_P);
      digitalWrite(A, LOW);
      digitalWrite(A_bar, LOW);
      digitalWrite(B, LOW);
      digitalWrite(B_bar, HIGH);
      ++stepCount;

      update_everything();

      //update track time
      update_time();
      ++ch;
    }
    break;
    }

    case 8:
    {

    currentMillis = millis();
    if (currentMillis - previousMillis >= y)
    {
      previousMillis = currentMillis;
      analogWrite(CoilA_PWM, min_P);
      analogWrite(CoilB_PWM, min_P);
      digitalWrite(A, HIGH);
      digitalWrite(A_bar, LOW);
      digitalWrite(B, LOW);
      digitalWrite(B_bar, HIGH);
      ++stepCount;

      update_everything();

      //update track time
      update_time();
      ch=1;
    }
    break;
    }


    default:
    {
      //Serial.print("NO CASE ------- ");
      //Serial.println(ch);
      break;
    }

  }
}
