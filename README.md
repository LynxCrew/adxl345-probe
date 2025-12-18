# This is a fork of the ADXL-Probe by jniebuhr to allow x and y homing with it, full credits go to the original project and the original license still applies

# ADXL345 Probe
[![](https://dcbadge.vercel.app/api/server/APw7rgPGPf)](https://delta2.eu/discord)
[![CC BY-NC-SA 4.0][cc-by-nc-sa-shield]][cc-by-nc-sa]

**!!! This project is in a BETA state, use at your own risk !!!**

The ADXL345 has an interesting feature called tap detection. With the appropriate tuning, this can be used to implement a nozzle probe on 3D printers.
This project aims to support nozzle probing through tap detection for printers using Klipper.

You can watch this thing in action here:
[https://www.youtube.com/shorts/_qd0kMkrVZw](https://www.youtube.com/shorts/_qd0kMkrVZw)

Results you can expect for a properly tuned system (This was on a Voron Trident):

```
probe accuracy results: maximum 0.007500, minimum 0.000937, range 0.006563, average 0.004031, median 0.004219, standard deviation 0.001841
```

I also measured the force on the bed using a standard kitchen scale, this was approximately 200g. A CAN bus board was used, so a direct connection might result in a quicker stop (See Multi MCU homing in the Klipper docs for more information on this).

## Installation

```bash
cd ~
wget -O - https://raw.githubusercontent.com/LynxCrew/adxl345-probe/main/install.sh | bash
```

## Physical setup

This code requires the ADXL int1 or int2 pins to be wired to one of your boards (preferrably the one that controls Z motion).
For a ADXL345 breakout board, simply run a wire. If you're using a CAN toolboard, the following boards are supported as they have wired the pins:

## Supported Boards

| Board  | Supported | int_pin | probe_pin | Link |
| ------ | :-------: | ------- | --------- | ---- |
| Mellow Fly SB2040 (v1/v2) | ✓ | int1 | gpio21 | https://aliexpress.com/item/1005004675264551.html |
| Mellow Fly SHT36 v2 | ✓ | int1 | PA10 | https://aliexpress.com/item/1005004675264551.html |
| Huvud | ✓ | ? | ? | |
| NiteHawk | ✓ | int1 | gpio21 |
| EBB36 | with soldering | int1/int2 | choose | |

## Configuration

This configuration must be **below** your adxl345 section.

```
[adxl345_probe]
probe_pin: <pin for either int1 or int2>
int_pin: int1 # select either int1 or int2, depending on your choice of wiring
tap_thresh: 12000 # this needs to be tuned
tap_thresh_x: 12000 # tap threshold for x homing, defaults to tap_thresh
tap_thresh_y: 12000 # tap threshold for y homing, defaults to tap_thresh
tap_thresh_z: 12000 # tap threshold for probing, defaults to tap_thresh
untrigger_thresh: 12000 # needed for beacon, needs to be tuned
untrigger_thresh_x: 12000 # untrigger threshold for x homing, defaults to untrigger_thresh
untrigger_thresh_y: 12000 # untrigger threshold for y homing, defaults to untrigger_thresh
untrigger_thresh_z: 12000 # untrigger threshold for z homing, defaults to untrigger_thresh
tap_dur: 0.01 # this needs to be tuned
tap_dur_x: 0.01 # tap duration for x homing, defaults to tap_dur
tap_dur_y: 0.01 # tap duration for y homing, defaults to tap_dur
tap_dur_z: 0.01 # tap duration for probing, defaults to tap_dur
speed: 20 # this needs to be tuned
z_offset: 0
# Adjust this to your liking
samples: 3
sample_retract_dist: 3.0
samples_result: median
samples_tolerance: 0.01
samples_tolerance_retries: 20
enable_x_homing: False
#   Define whether the adxl probe should be used to home x
enable_y_homing: False
#   Define whether the adxl probe should be used to home y
enable_probe: True
#   Define whether the adxl_probe should register as a probe
log_homing_data: False
#   Log accelerometer data to a file
stepper_enable_dwell_time:
#   Time to dwell after enabling the steppers before homing
#disable_fans:
#   Fans to be disabled while homing or probing
#activate_gcode:
#   Gcode to run before homing/probing
#activate_gcode_x:
#   Same as above but specific to homing x, defaults to activate_gcode
#activate_gcode_y:
#   Same as above but specific to homing y, defaults to activate_gcode
#activate_gcode_z:
#   Same as above but specific to homing/probing z, defaults to activate_gcode
#deactivate_gcode:
#   Gcode to run after homing/probing
#deactivate_gcode_x:
#   Same as above but specific to homing x, defaults to activate_gcode
#deactivate_gcode_y:
#   Same as above but specific to homing y, defaults to activate_gcode
#deactivate_gcode_z:
#   Same as above but specific to homing/probing z, defaults to activate_gcode
```

If you want to use the probe as endstops as well:

```
[stepper_z]
... your remaining config ...
endstop_pin: probe:z_virtual_endstop
```
Make sure to remove `position_endstop` in this case.


```
[stepper_x]
... your remaining config ...
endstop_pin: accelerometer_endstop_x:virtual_endstop
```

```
[stepper_y]
... your remaining config ...
endstop_pin: accelerometer_endstop_y:virtual_endstop
```

## Tuning guide

ADXL homing is sensible to the starting conditions: to ensure reliable homing make sure the steppers are enabled *before* starting the homing procedure.

Lower homing accelerations (<500mm/s) are suggested to prevent early triggers at the start of the homing move.

The first parameter to tune is the `tap_treshold`, remembering that *lower* values result in a *higher* sensitivity.

Tap duration tuning still in progress...

## License

This work is licensed under a
[Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License][cc-by-nc-sa].

[![CC BY-NC-SA 4.0][cc-by-nc-sa-image]][cc-by-nc-sa]

[cc-by-nc-sa]: http://creativecommons.org/licenses/by-nc-sa/4.0/
[cc-by-nc-sa-image]: https://licensebuttons.net/l/by-nc-sa/4.0/88x31.png
[cc-by-nc-sa-shield]: https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg?style=for-the-badge
