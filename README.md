# This fork of the two upstream repos adds the ability to use the ADXL345's ACT mode, which allows far better sensitivity than TAP.

[![CC BY-NC-SA 4.0][cc-by-nc-sa-shield]][cc-by-nc-sa]

https://delta2.eu/discord

**!!! This project is in a BETA state, use at your own risk !!!**

The ADXL345 can detect a sudden "bump" using either of two modes - "TAP" and "ACT" (action). With the appropriate tuning, this can be used to implement a nozzle probe as well as X/Y on 3D printers.
This project aims to support nozzle probing and X/Y homing through ADXL detection for printers using Klipper.

You can watch this thing in action here:
[https://www.youtube.com/shorts/_qd0kMkrVZw](https://www.youtube.com/shorts/_qd0kMkrVZw)

Results you can expect for a properly tuned system (This was on a Voron Trident, and similar was achieved on a Voron 0):

```
probe accuracy results: maximum 0.007500, minimum 0.000937, range 0.006563, average 0.004031, median 0.004219, standard deviation 0.001841
```

Force on the bed was measured using a standard kitchen scale, this was approximately 200g. A CAN bus board was used, so a direct connection might result in a quicker stop (See Multi MCU homing in the Klipper docs for more information on this).
(This will probably have improved with the move from TAP mode to ACT.)

## Installation

```bash
cd $HOME
git clone https://github.com/3d-olympics/adxl345-probe
cd adxl345-probe
./scripts/install.sh
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

This Klipper configuration must be **below** your adxl345 section.
This configuration is for ACT mode, which is far more sensitive than TAP mode, and takes different parameters - see below for more details. But this fork still supports using TAP mode with your old config from one of the upstream repos.

```
[adxl345_probe]
mode: act # You can also use the legacy "tap" mode, which takes different parameters as outlined in the original upstream repos, but "act" is far superior
probe_pin: <pin for either int1 or int2>
int_pin: int1 # select either int1 or int2, depending on your choice of wiring
act_thresh_x: 13
act_thresh_y: 13
act_thresh_z: 5 # these all need to be tuned
speed: 14
z_offset: 0
samples: 3
sample_retract_dist: 3.0
samples_result: median
samples_tolerance: 0.01
samples_tolerance_retries: 20
enable_x_homing: True
enable_y_homing: True
enable_probe: True
log_homing_data: False  # Log accelerometer data to a file
stepper_enable_dwell_time: 0.1  # Time to dwell after enabling the steppers before homing (currently not working sorry)
```

If you want to use the probe as endstops as well:

```
[stepper_z]
... your remaining config ...
endstop_pin: probe:z_virtual_endstop
```

```
[stepper_x]
... your remaining config ...
endstop_pin: adxl_probe_x:virtual_endstop
```

```
[stepper_y]
... your remaining config ...
endstop_pin: adxl_probe_y:virtual_endstop
```

Make sure to remove `position_endstop` from your stepper_z config section in this case.

## More info
The ADXL345's TAP mode, which was used in earlier code, has a huge disadvantage in that the 9800mm/s2 of acceleration that is gravity is not removed (or "AC coupled out") of the ADXL's readings before the threshold is applied. This meant that you had to use a threshold above 9800, and then it wasn't very sensitive. This actually was addressable by manually writing an offset to the ADXL, but this would have required extra code.

Also, TAP mode requires the specification of a "TAP_DUR", or duration of the bump that the accelerometer is expecting to see. This doesn't work well because it's actually a maximum, not a minimum. It says "only trigger if we've received a bump that was shorter than the specified duration". This means that the interrupt isn't triggered until the bump has settled back down, which will be slightly later than the actual impact by an inconsistent amount - not to mention that if the bump is too long, it won't trigger at all. Even if it does get triggered, readings will be a bit inconsistent, and it won't be triggered as early as it could have, meaning the toolhead / bed will have kept moving into each other longer than necessary.

The far better alternative is ACT mode, which detects a bump in the same way, but has an "AC coupled" mode to remove the effect of gravity, allowing thresholds several times lower to be used, and has no maximum bump duration to complicate things.

The act_thresh params that this mode takes are in the raw numeric format that the ADXL works with, as you'll probably want to experiment precisely with these. 1 unit of act_thresh is "worth" 613.125 units of the older tap_thresh. Or in other words, an act_thresh of 20 would be equivalent to the original recommended tap_thresh value of 12000 mm/s2. Except now you can use an act_thresh potentially as low as 3 or even 2.

## Further setup
You will probably want to create a homing_override script for Klipper which does things like lower accelerations, including for the Z axis, before probing or homing. It will also need to engage the motors before homing to avoid a sudden jerk and false triggering, which was traditionally done with an M17 command, but Klipper doesn't support this so you can use e.g. `SET_STEPPER_ENABLE STEPPER=stepper_x ENABLE=1`

You will also need to disable fans in order to use the most sensitive settings and to improve accuracy.

## Tuning guide

In progress...

## License

This work is licensed under a
[Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License][cc-by-nc-sa].

[![CC BY-NC-SA 4.0][cc-by-nc-sa-image]][cc-by-nc-sa]

[cc-by-nc-sa]: http://creativecommons.org/licenses/by-nc-sa/4.0/
[cc-by-nc-sa-image]: https://licensebuttons.net/l/by-nc-sa/4.0/88x31.png
[cc-by-nc-sa-shield]: https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg?style=for-the-badge
