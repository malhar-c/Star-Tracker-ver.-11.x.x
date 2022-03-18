# Star-Tracker-ver.-11.x.x
# DIY Advanced Star Tracker (Arduino + L298N Stepper Driver)

[This was a path finder and has officially ended... visit here for the new version: https://github.com/malhar-c/Advanced-Arduino-Star-Tracker-OuttaSyllabus.git]

*by Malhar Chakraborty (OuttaSyllabus)*

*date 07/03/2021 - started (version 0.1 full step)*

### RnD and Code Journey


[0.5]
* micro stepping implementation

[2.0]
* 1/4 micro-stepped
* cons- less torque

[2.1]
* increasing voltage to increase torque
* cons- less torque in intermediate micro-steps

[3.0]
* {Pro} 1. precise time display in serial monitor
* {Pro} 2. angle measurement of the final base with each micro-step
* {Pro} 3. extra controls of motorized alignment and adjustment (using interrupt)*

[3.1]*
* scraping the idea of interrupt implementation

[4.0]
* millis() implementation for non blocking code

[5.0]
* remoteXY config (updated)

[6.0]
* scrubbing idea of portability, switching to barn door type
  - Barn Door concept
  - 1 RPM drive
  - Quarter stepping (192 Steps per revolution)
  - 0.3125s gap between steps
  - 312 milliseconds + 500 microseconds

[6.1]
* plank size reduced
  - 0.5 RPM Drive
  - quarter stepping
  - intermediate gear (white gear with 60 teeth)
  - 0.01041666 s gap between steps
  - 10 ms + 416 us

[7.0]
* Scrubbed intermediate gear
  - scrubbed using of white gear because of wobble (the gear itself is wobbly)
  - back to direct motor drive (motor shaft to drive bolt)
  - quarter stepping
  - 625 ms gap between each micro steps (for 300mm focal length)

[8.0]
* Switch to Half Stepping
  - not enough torque at 192 microsteps
  - switching to half steps instead (96 microsteps)
  - 1250 ms gap between steps
  - may be just enough for 300 mm focal length


----Star Tracker SN2----

 [10.0.1]
 * Switch to Atom IDE and new RemoteXY UI
    - New GUI (simplified first page)
    - Provision to automatically stop the drive bolt (when reached limits)
    - one button fast rewind with protection.
    - better manual control.
    - STATUS page shows the status of the system, uptime, tracktime,
    motor stepping, position limit sensors, base angle, etc.

[10.0.2]
  - Fixed a bug which don't show the base angle on homepage

[10.0.3 to 10.0.x]

 - [FIXED] uptime resetting when stopping the tracking (first time)
 - [FIXED] main status shows "STOPPED" always
 - [FIXED] sensor status shows NULL
 - [FIXED] virtual LEDs malfunction
 - [FIXED] motor level indicator glitching

[10.1.0] Automatic Calibration

* Adding a calibration function which will detect and calibrate the stepCount and base angle offset with the help of position/limit sensors/switches.
* The calibration will run at the start everytime the star tracker is turned on and connected to wifi.

[10.1.1]
- [FIXED] Not connecting to wifi whenever calibration function is called

[10.1.2 to 10.1.x]
- [FIXED] runtime error for unsigned data type of step count, now step count
 can be in negative initially, just because of calibration.
 - [TWEAKED] virtual LEDs
 - [TWEAKED] For calibration, no need to connect to wifi, as soon as the ESP is ready and AP is configed. It will start calibrating.
 - [MAJOR PROBLEM] when wifi disconnects, tracking force reduces... (coil voltage reduces probably, or some step skips).


 ```
FULL TIME TEST 1.0 (FTT 1.0)

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
 ```

 ```
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
 ```

```
 Base angle calibration

 (all are with the axis of rotation)
 - bottom plank = 3 degrees
 - top plank = 17 degrees max***
 - top plank = -- degrees min***
 [to redo]
 ```

[10.2.x]
* (First Run problem fixes and improvements)
  - [FIXED] Lower limit switch/sensor solder padded (height increased)
  - [EXPECTATION] now the max tracking time should reduce a lil bit (as the
lower limit height increased)
  - [IMPROVED] (Still to test whether fixed or not) drive bolt top touching
surface re-enforced with M-Seal and silicone lubed (surface smoothed too)
now the drive head should not stick as the top plank load increases.

```
[NEW] base angle calibration
(all are with the axis of rotation)
- bottom plank = 0 degrees [CALIBRATED]
- top plank = 15.05 degrees max
- top plank = 4.6 degrees min
```

[11.0.0]

**(UI changes and new features)**
 - UI version SN2.1
 - RemoteXY lib. ver. 3.1.6
 - will turn on the fan when its needed
 - live temperature readings of the ESP and motor driver
 - turn on (or off) the fan automatically
 - manual fan control as well (failsafe)

[11.0.x]

 - [FIXED] Glitch in fan on/off and status display
 - [FIXED] Temperature readings changing drastically (jumping all over the place) now, the main variable will take an average of 200 samples and process accordingly. (this may cause performance issues)*
 - [NOTE*] Temperature sensors calibrated w.r.t. 12V adaptor
 - [ADDED] PWM control for the fan, the fan now turns on, adjusts speed according to the temperature of the wifi mod and motor driver.
 - [PROBLEM*] Temperature sensor values changes as the input voltage changes

**doubtful, may have to revisit once everything is running normally with battery**
