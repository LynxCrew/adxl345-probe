# This fork of the two upstream repos adds the ability to use the ADXL345's ACT mode, which allows far better sensitivity than TAP.

[![CC BY-NC-SA 4.0][cc-by-nc-sa-shield]][cc-by-nc-sa]

https://delta2.eu/discord

**!!! This project is in a BETA state, use at your own risk !!!**

The ADXL345 can detect a sudden "bump" using either of two modes - "TAP" and "ACT" (action). With the appropriate tuning, this can be used to implement a nozzle probe as well as X/Y on 3D printers.
This project aims to support nozzle probing and X/Y homing through ADXL detection for printers using Klipper.

New video of homing and probing on a Voron 0.1 / 0.2:
https://www.youtube.com/watch?v=m7PLHfCfIJk

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
act_thresh_z: 5 # These all need to be tuned
speed: 14 # Tune this too. Too fast leads to inaccuracy and increased strain after collision. Too low and it won't trigger.
z_offset: 0
samples: 3
sample_retract_dist: 3 # Too short, and the Z movement won't have accelerated to full speed, leading to inconsistency and potentially failure to trigger
samples_result: median
samples_tolerance: 0.01
samples_tolerance_retries: 20
enable_x_homing: True
enable_y_homing: True
enable_probe: True
log_homing_data: False  # Log accelerometer data to a file
stepper_enable_dwell_time: 0.1  # Time to dwell after enabling the steppers before homing
disable_fans: heater_fan hotend_fan # Comma-separated list. Disabling the fans at least leads to better accuracy, and may be needed to avoid false triggering.
```

If you want to use the probe as X/Y endstops as well:
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

And to use the ADXL for homing Z, use something like the following and make sure to remove `position_endstop` from your `[stepper_z]` config section:
```
[stepper_z]
... your remaining config ...
endstop_pin: probe:z_virtual_endstop
homing_positive_dir: false
homing_speed: 14 # These should be the same as you've tuned in [adxl345_probe]
homing_retract_dist: 0 # Disables slower second homing - it won't help when using an ADXL, you've already tuned in the ideal speed
```

## More info on this fork
The ADXL345's TAP mode, which was used in earlier code, has a huge disadvantage in that the 9800mm/s2 of acceleration that is gravity is not removed (or "AC coupled out") of the ADXL's readings before the threshold is applied. This meant that you had to use a threshold above 9800, and then it wasn't very sensitive. This actually was addressable by manually writing an offset to the ADXL, but this would have required extra code.

Also, TAP mode requires the specification of a "TAP_DUR", or duration of the bump that the accelerometer is expecting to see. This doesn't work well because it's actually a maximum, not a minimum. It says "only trigger if we've received a bump that was shorter than the specified duration". This means that the interrupt isn't triggered until the bump has settled back down, which will be slightly later than the actual impact by an inconsistent amount - not to mention that if the bump is too long, it won't trigger at all. Even if it does get triggered, readings will be a bit inconsistent, and it won't be triggered as early as it could have, meaning the toolhead / bed will have kept moving into each other longer than necessary.

The far better alternative is ACT mode, which detects a bump in the same way, but has an "AC coupled" mode to remove the effect of gravity, allowing thresholds several times lower to be used, and has no maximum bump duration to complicate things.

The act_thresh params that this mode takes are in the raw numeric format that the ADXL works with, as you'll probably want to experiment precisely with these. 1 unit of act_thresh is "worth" 613.125 units of the older tap_thresh. Or in other words, an act_thresh of 20 would be equivalent to the original recommended tap_thresh value of 12000 mm/s2. Except now you can use an act_thresh potentially as low as 3 or even 2.

## Tuning guide
Try setting this up for just probing, not endstops, at first.
Use `PROBE_ACCURACY SAMPLES=1` on the console to make the printer try one probing move. You may like to set `act_thresh_z` to something very low like 1 at first to be safe.

If it falsely triggers, try raising `act_thresh_z`, or reducing `speed`, or if it falsely triggers immediately as the z move begins, try reducing `max_z_accel` under `[printer]`.

Or if it fails to trigger, your nozzle and bed will probably collide, which isn't ideal in terms of strain on your machine. You could modify `position_min` under `[stepper_z]` to a smaller negative number like -0.5 (if you previously had it bigger) so that the printer won't push very far in such a case.
Or, if it doesn't trigger and your bed and nozzle haven't made contact yet, try using a bigger negative number like -1.5 to ensure that it will move far enough. Keep in mind, if adjusting this later, that the nozzle will decelerate before it reaches this position, meaning it could be travelling at a lower `speed` than you set when the nozzle and bed collide, especially if you've reduced `max_z_accel` under `[printer]`. So after you're fairly sure that things are operating safely, a bigger negative number like -1.5 is good here - don't try to tune it as small as possible, because speed at the time of the bump, and hence the accuracy of the readings in my experience, will be inconsistent.

Next, you'll want to try higher `act_thresh_z` values, to eliminate false triggering. You might find that on different days, slightly different lower thresholds work. Maybe establish the lowest `act_thresh_z` that avoids false triggering, and the highest `act_thresh_z` that avoids failure-to-trigger (if you can bear to have your nozzle and bed collide a couple times), and then settle on a value halfway inbetween. Before doing too much fine-tuning at one point on your bed, you might want to try different points or even a full bed mesh probing (which you'll need to set up), because different spots will falsely trigger, or fail to trigger, at different thresholds. For me, values between about 3 and 7 worked fairly consistently across the bed, so I settled on 5. Perhaps avoid probing at the outer couple of mm of the bed - it can be more inconsistent.

And by now you'll probably also be using `PROBE_ACCURACY SAMPLES=10` or more, to get an idea of the probe's consistency. Standard deviation values under 0.002mm were consistently achievable on my Voron 0. Different `act_thresh_z` values may give slightly different accuracies. You'll also want to cut the fans (the config above should do this automatically), and there may be a slight benefit to reducing Z motor current.

And then you can also tune `speed`. Too low will cause a failure to trigger, depending on your `act_thresh_z` value. Too high, and accuracy will go down as well as your printer receiving more of a knock each time. You can try different speeds and run `PROBE_ACCURACY` to see how they compare. I found 14 to be ideal - as accurate as any lower speeds and still relatively gentle - whereas higher speed quickly reduced accuracy.

## Setting up homing
In addition to the config file changes outlined above, you will probably want to create a `[homing_override]` script for Klipper which does things like lower accelerations and maybe motor currents before homing. The following script has been created for a Voron 0.2, and is for using the ADXL to home all 3 axes including Z. It should basically also work if some of these axes are not using the ADXL, so long as your Z axis homes in a "negative" direction, i.e. the endstop triggers when your nozzle and bed are close.
```
[homing_override]
gcode:
    SET_TMC_CURRENT STEPPER=stepper_x CURRENT=0.03 # X/Y current low so we don't damage anything if ADXL virtual endstop fails to trigger
    SET_TMC_CURRENT STEPPER=stepper_y CURRENT=0.03
    SET_TMC_CURRENT STEPPER=stepper_z CURRENT=0.01 # Z current very low since we're going to lower the bed before determining whether that's safe.
    M204 S600 # Set X/Y acceleration, lowish so no false triggers during homing
    G91 # Relative positioning
    SET_KINEMATIC_POSITION Z=0 # Zero Z axis
    G1 Z1 F500 # Move Z away initially, out of way of toolhead as we home X and Y
    G90 # Absolute positioning

    G28 X # Home X
    G1 X60 F3600 # Move X to centre
	
	SET_TMC_CURRENT STEPPER=stepper_z CURRENT=0.1	# Set Z current for homing Z. A relatively low value can slightly improve accuracy.
													# You'd also want to set this before probing. And you can measure that accuracy with PROBE_ACCURACY.
													# We do this awkwardly between homing X and Y so that the Z motor has settled after its last move, and has time to readjust before homing.
    G28 Y # Home Y
    G1 Y60 F3600 # Move Y to centre
	
    G28 Z # Home Z
    G1 Z10 F1800 # Move Z away

    # Set motor current back to normal
    SET_TMC_CURRENT STEPPER=stepper_x CURRENT={printer.configfile.settings['tmc2209 stepper_x'].run_current}
    SET_TMC_CURRENT STEPPER=stepper_y CURRENT={printer.configfile.settings['tmc2209 stepper_y'].run_current}
    SET_TMC_CURRENT STEPPER=stepper_z CURRENT={printer.configfile.settings['tmc2209 stepper_z'].run_current}
	
    M204 S{printer.configfile.settings['printer'].max_accel} # Set acceleration back to normal
```

## License

This work is licensed under a
[Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License][cc-by-nc-sa].

[![CC BY-NC-SA 4.0][cc-by-nc-sa-image]][cc-by-nc-sa]

[cc-by-nc-sa]: http://creativecommons.org/licenses/by-nc-sa/4.0/
[cc-by-nc-sa-image]: https://licensebuttons.net/l/by-nc-sa/4.0/88x31.png
[cc-by-nc-sa-shield]: https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg?style=for-the-badge
