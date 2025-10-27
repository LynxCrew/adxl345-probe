import datetime
import os

from . import probe, adxl345

REG_THRESH_TAP = 0x1D
REG_DUR = 0x21
REG_INT_MAP = 0x2F
REG_TAP_AXES = 0x2A
REG_INT_ENABLE = 0x2E
REG_INT_SOURCE = 0x30

DUR_SCALE = 0.000625  # 0.625 msec / LSB
TAP_SCALE = 0.0625 * adxl345.FREEFALL_ACCEL  # 62.5mg/LSB * Earth gravity in mm/s**2

ADXL345_REST_TIME = 0 # 0.1

REG_OFSX = 0x1E
REG_OFSY = 0x1F
REG_OFSZ = 0x20
REG_THRESH_ACT = 0x24
REG_ACT_INACT_CTL  = 0x27

class ADXL345Endstop:
    def __init__(self, adxl345probe, axis=None):
        self.adxl345probe = adxl345probe
        self.printer = adxl345probe.printer
        self.axis = axis
        self.mcu_endstop = None
        self.stepper_enable = self.printer.load_object(
            self.adxl345probe.config, "stepper_enable"
        )
        self.gcode = self.printer.lookup_object("gcode")
        self.aclient = None

    def setup_pin(self, pin_type, pin_params):
        # Validate pin
        ppins = self.printer.lookup_object("pins")
        if pin_type != "endstop" or pin_params["pin"] != "virtual_endstop":
            raise ppins.error("probe virtual endstop only useful as endstop")
        if pin_params["invert"] or pin_params["pullup"]:
            raise ppins.error("Can not pullup/invert tmc virtual pin")
        # Setup for sensorless homing
        self.printer.register_event_handler(
            "homing:homing_move_begin",
            lambda hmove: self.handle_homing_move_begin(hmove, self.axis),
        )
        self.printer.register_event_handler(
            "homing:homing_move_end",
            lambda hmove: self.handle_homing_move_end(hmove, self.axis),
        )
        if self.axis == "x":
            self.mcu_endstop = self.adxl345probe.mcu_endstop_x
        elif self.axis == "y":
            self.mcu_endstop = self.adxl345probe.mcu_endstop_y
        return self.mcu_endstop

    def handle_homing_move_begin(self, hmove, axis=None):
        if self.mcu_endstop not in hmove.get_mcu_endstops() or axis != self.axis:
            return

        for stepper in self.adxl345probe.get_steppers(self.axis):
            #self.gcode.respond_info(stepper.get_name())
            self.stepper_enable.set_motors_enable([stepper.get_name()], True)
            
        self.printer.lookup_object("toolhead").dwell(
            self.adxl345probe.stepper_enable_dwell_time # TODO: only dwell if the stepper was not already enabled.
        )

        self.adxl345probe.init_adxl(self.axis)
        if self.adxl345probe.log_homing_data:
            self.aclient = self.adxl345probe.adxl345.start_internal_client()

        self.adxl345probe.probe_prepare(hmove, axis=self.axis)

    def handle_homing_move_end(self, hmove, axis=None):
        if self.mcu_endstop not in hmove.get_mcu_endstops() or axis != self.axis:
            return

        if self.adxl345probe.log_homing_data:
            self.aclient.finish_measurements()
            raw_name = self.get_filename()
            self.aclient.write_to_file(raw_name)
            self.gcode.respond_info("Writing homing data to %s file" % raw_name)
        self.adxl345probe.probe_finish(hmove, axis=self.axis)

    def get_filename(self):
        name = "adxl_homing-"
        time = datetime.datetime.now()
        return os.path.join("/tmp", name + time.strftime("%Y-%m-%d_%H:%M:%S") + ".csv")


class ADXL345Probe:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        gcode_macro = self.printer.load_object(config, "gcode_macro")
        self.activate_gcode = gcode_macro.load_template(config, "activate_gcode", "")
        self.deactivate_gcode = gcode_macro.load_template(
            config, "deactivate_gcode", ""
        )
        int_pin = config.get("int_pin").strip()
        self.inverted = False
        self.is_measuring = False
        self._in_multi_probe = False
        if int_pin.startswith("!"):
            self.inverted = True
            int_pin = int_pin[1:].strip()
        if int_pin != "int1" and int_pin != "int2":
            raise config.error("int_pin must specify one of int1 or int2 pins")
        probe_pin = config.get("probe_pin")
        adxl345_name = config.get("chip", "adxl345")
        self.int_map = 0x40 if int_pin == "int2" else 0x0
        
        self.mode = config.get("mode", "tap")
        if self.mode != "tap" and self.mode != "act":
            raise config.error("adxl345_probe mode must be either 'tap' or 'act'. The default is tap for backwards compatibility, but we recommend act")
        
        if self.mode == "tap":
            self.int_reg_value = 0x40
            self.tap_thresh = config.getfloat(
                "tap_thresh", 5000, minval=TAP_SCALE, maxval=100000.0
            )
            self.tap_thresh_x = self.tap_thresh
            self.tap_thresh_y = self.tap_thresh
            self.tap_thresh_z = self.tap_thresh
            self.tap_dur = config.getfloat("tap_dur", 0.01, above=DUR_SCALE, maxval=0.1)
            self.tap_dur_x = self.tap_dur
            self.tap_dur_y = self.tap_dur
            self.tap_dur_z = self.tap_dur
        else: # "act" mode
            self.int_reg_value = 0x10

        self.position_endstop = config.getfloat("z_offset")
        self.disable_fans = [
            fan.strip() for fan in config.get("disable_fans", "").split(",") if fan
        ]
        
        self.adxl345 = self.printer.lookup_object(adxl345_name)
        self.next_cmd_time = self.action_end_time = 0.0
        # Create an "endstop" object to handle the sensor pin
        ppins = self.printer.lookup_object("pins")
        pin_params = ppins.lookup_pin(probe_pin, can_invert=True, can_pullup=True)
        mcu = pin_params["chip"]
        self.mcu_endstop_z = mcu.setup_pin("endstop", pin_params)
        self.mcu_endstop_x = mcu.setup_pin("endstop", pin_params)
        self.mcu_endstop_y = mcu.setup_pin("endstop", pin_params)
        self.enable_x_homing = config.getboolean("enable_x_homing", False)
        self.enable_y_homing = config.getboolean("enable_y_homing", False)
        self.enable_probe = config.getboolean("enable_probe", True)
        self.log_homing_data = config.getboolean("log_homing_data", False)
        self.stepper_enable_dwell_time = config.getfloat(
            "stepper_enable_dwell_time", 0.1
        )
        # Add wrapper methods for endstops
        self.get_mcu = self.mcu_endstop_z.get_mcu
        self.steppers = {}
        self.home_start = self.mcu_endstop_z.home_start
        self.home_wait = self.mcu_endstop_z.home_wait
        self.query_endstop = self.mcu_endstop_z.query_endstop
        # Register commands and callbacks
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_mux_command(
            "SET_ACCEL_PROBE",
            "CHIP",
            None,
            self.cmd_SET_ACCEL_PROBE,
            desc=self.cmd_SET_ACCEL_PROBE_help,
        )
        self.gcode.register_mux_command(
            "HOTEND_FAN_OFF",
            "CHIP",
            None,
            self.cmd_HOTEND_FAN_OFF,
        )
        self.gcode.register_mux_command(
            "HOTEND_FAN_ON",
            "CHIP",
            None,
            self.cmd_HOTEND_FAN_ON,
        )
        if self.enable_probe:
            self.cmd_helper = probe.ProbeCommandHelper(config, self, self.query_endstop)
            self.probe_offsets = probe.ProbeOffsetsHelper(config)
            self.param_helper = probe.ProbeParameterHelper(config)
            self.homing_helper = probe.HomingViaProbeHelper(config, self, self.param_helper)
            self.probe_session = probe.ProbeSessionHelper(config, self.param_helper, self.homing_helper.start_probe_session)
            self.printer.add_object("probe", self)

            if self.mode == "tap":
                self.tap_thresh_z = config.getfloat(
                    "tap_thresh_z", self.tap_thresh, minval=TAP_SCALE, maxval=100000.0
                )
                self.tap_dur_z = config.getfloat(
                    "tap_dur_z", self.tap_dur, above=DUR_SCALE, maxval=0.1
                )
            else: # "act" mode
                self.act_thresh_z = config.getfloat(
                    "act_thresh_z", minval=1, maxval=255
                )

        if self.enable_x_homing:
            x_endstop = ADXL345Endstop(self, "x")
            ppins.register_chip("adxl_probe_x", x_endstop)

            if self.mode == "tap":
                self.tap_thresh_x = config.getfloat(
                    "tap_thresh_x", self.tap_thresh, minval=TAP_SCALE, maxval=100000.0
                )
                self.tap_dur_x = config.getfloat(
                    "tap_dur_x", self.tap_dur, above=DUR_SCALE, maxval=0.1
                )
            else: # "act" mode
                self.act_thresh_x = config.getfloat(
                    "act_thresh_x", minval=1, maxval=255
                )
        if self.enable_y_homing:
            y_endstop = ADXL345Endstop(self, "y")
            ppins.register_chip("adxl_probe_y", y_endstop)

            if self.mode == "tap":
                self.tap_thresh_y = config.getfloat(
                    "tap_thresh_y", self.tap_thresh, minval=TAP_SCALE, maxval=100000.0
                )
                self.tap_dur_y = config.getfloat(
                    "tap_dur_y", self.tap_dur, above=DUR_SCALE, maxval=0.1
                )
            else: # "act" mode
                self.act_thresh_y = config.getfloat(
                    "act_thresh_y", minval=1, maxval=255
                )
        self.printer.register_event_handler("klippy:connect", self.init_adxl)
        self.printer.register_event_handler(
            "klippy:mcu_identify", self.handle_mcu_identify
        )

    def init_adxl(self, axis=None):
        chip = self.adxl345
        chip.set_reg(adxl345.REG_POWER_CTL, 0x00)
        chip.set_reg(adxl345.REG_DATA_FORMAT, 0x0B)
        if self.inverted:
            chip.set_reg(adxl345.REG_DATA_FORMAT, 0x2B)
        chip.set_reg(REG_INT_MAP, self.int_map)
        if self.mode == "tap":
            tap_thresh = self.tap_thresh
            tap_dur = self.tap_dur
            if axis == "x":
                tap_thresh = self.tap_thresh_x
                tap_dur = self.tap_dur_x
            elif axis == "y":
                tap_thresh = self.tap_thresh_y
                tap_dur = self.tap_dur_y
            elif axis == "z":
                tap_thresh = self.tap_thresh_z
                tap_dur = self.tap_dur_z
            chip.set_reg(REG_TAP_AXES, 0x7)
            chip.set_reg(REG_DUR, int(tap_dur / DUR_SCALE))
            chip.set_reg(REG_THRESH_TAP, int(tap_thresh / TAP_SCALE))
        else: # "act" mode
            if axis == "x":
                act_thresh = self.act_thresh_x
            elif axis == "y":
                act_thresh = self.act_thresh_y
            else: # z or none
                act_thresh = self.act_thresh_z
            chip.set_reg(REG_ACT_INACT_CTL, 0xF0) # AC mode (cancels out gravity), and enable all 3 axes.
            chip.set_reg(REG_THRESH_ACT, int(act_thresh))

    def handle_mcu_identify(self):
        self.phoming = self.printer.lookup_object("homing")
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis("z"): # Multiple ones of these can be true, i.e. a motor might have a part in multiple axes in a core-xy or delta printer.
                self.add_stepper(stepper, "z")
                self.mcu_endstop_z.add_stepper(stepper)
            if stepper.is_active_axis("x"):
                self.add_stepper(stepper, "x")
                self.mcu_endstop_x.add_stepper(stepper)
            if stepper.is_active_axis("y"):
                self.add_stepper(stepper, "y")
                self.mcu_endstop_y.add_stepper(stepper)

    def control_fans(self, disable=True, delay_if_necessary=False):
        for fan in self.disable_fans:
            fan = self.printer.lookup_object(fan)
            if fan.fan_speed != 0:
                fan._fan_speed = fan.fan_speed
            if disable:
                if fan.fan_speed != 0:
                    fan.fan_speed = 0
                    if delay_if_necessary:
                        toolhead = self.printer.lookup_object("toolhead")
                        toolhead.dwell(2.5)
            else:
                fan.fan_speed = fan._fan_speed
                fan._fan_speed = 0

    def multi_probe_begin(self):
        self._in_multi_probe = True
        self._done_init_for_multi_probe_yet = False
        self.control_fans(disable=True, delay_if_necessary=True)

    def multi_probe_end(self):
        self.control_fans(disable=False)
        chip = self.adxl345
        chip.set_reg(adxl345.REG_POWER_CTL, 0x00)
        self._in_multi_probe = False

    def probing_move(self, pos, speed):
        return self.phoming.probing_move(self, pos, speed)

    def get_position_endstop(self):
        return self.position_endstop

    def get_probe_params(self, gcmd=None):
        return self.param_helper.get_probe_params(gcmd)

    def get_offsets(self):
        return self.probe_offsets.get_offsets()

    def get_status(self, eventtime):
        return self.cmd_helper.get_status(eventtime)

    def start_probe_session(self, gcmd):
        return self.probe_session.start_probe_session(gcmd)

    def _try_clear_int(self):
        chip = self.adxl345
        tries = 24 # 8
        while tries > 0:
            val = chip.read_reg(REG_INT_SOURCE)
            if not (val & self.int_reg_value): # That masks either TAP and ACT
                return True
            tries -= 1
        return False
        
    def probe_prepare(self, hmove, axis="z"):
        chip = self.adxl345

        # We don't switch the fans off before homing X and Y. If you want to, use HOTEND_FAN_OFF in your g-code.
        if axis=="z":
            self.control_fans(disable=True, delay_if_necessary=(axis=="z"))
            
        if not self._in_multi_probe or not self._done_init_for_multi_probe_yet:
            self.init_adxl(axis)    # We only want to do one init_adxl() per multi-probe - otherwise, it gets tripped up when other corners are cut to increase speed.
                                    # But we can't do this in multi_probe_begin() because we don't know the axis yet there - even though surely it's always gotta be z.
            chip.set_reg(adxl345.REG_POWER_CTL, 0x08)
            self._done_init_for_multi_probe_yet = True
            
        self.activate_gcode.run_gcode_from_command()
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.flush_step_generation()
        toolhead.dwell(ADXL345_REST_TIME)
        print_time = toolhead.get_last_move_time()
        clock = self.adxl345.mcu.print_time_to_clock(print_time)
        chip.set_reg(REG_INT_ENABLE, 0x00, minclock=clock) # If we could only get rid of the minclock=clock, sometimes it goes wicked-fast! But sometimes ends up "triggered prior to movement".
        chip.read_reg(REG_INT_SOURCE)
        chip.set_reg(REG_INT_ENABLE, self.int_reg_value) # Enables either TAP or ACT
        if not self._try_clear_int():
            raise self.printer.command_error(
                "ADXL345 triggered before move, it may be set too sensitive."
            )
            
    def probe_finish(self, hmove, axis="z"): # We want this function to run and finish as quickly as possible, so the probe can be pulled up away from the bed.
        chip = self.adxl345
        toolhead = self.printer.lookup_object("toolhead")
        #toolhead.dwell(ADXL345_REST_TIME)
        #print_time = toolhead.get_last_move_time()
        #clock = chip.mcu.print_time_to_clock(print_time)
        #chip.set_reg(REG_INT_ENABLE, 0x00, minclock=clock) # This one slows it down a whole lot! We can leave this out so long as we only call init_adxl() once per multi-probe.
        self.deactivate_gcode.run_gcode_from_command()

        if not self._in_multi_probe:
            chip.set_reg(adxl345.REG_POWER_CTL, 0x00)
            if axis == "z": # Fans were only automatically disabled for Z
                self.control_fans(disable=False)

    def add_stepper(self, stepper, axis=None):
        if axis is not None:
            if axis in self.steppers:
                self.steppers[axis].append(stepper)
            else:
                self.steppers[axis] = [stepper]

    def get_steppers(self, axis=None):
        if axis is not None:
            return self.steppers[axis]
        return self.mcu_endstop_z.get_steppers() # This would return just the Z stepper(s)

    cmd_SET_ACCEL_PROBE_help = "Configure ADXL345 parameters related to probing"

    def cmd_SET_ACCEL_PROBE(self, gcmd):
        chip = self.adxl345
        if self.mode == "tap":
            self.tap_thresh = gcmd.get_float(
                "TAP_THRESH", self.tap_thresh, minval=TAP_SCALE, maxval=100000.0
            )
            self.tap_thresh_x = gcmd.get_float(
                "TAP_THRESH_X", self.tap_thresh_x, minval=TAP_SCALE, maxval=100000.0
            )
            self.tap_thresh_y = gcmd.get_float(
                "TAP_THRESH_Y", self.tap_thresh_y, minval=TAP_SCALE, maxval=100000.0
            )
            self.tap_dur = gcmd.get_float(
                "TAP_DUR", self.tap_dur, above=DUR_SCALE, maxval=0.1
            )
            chip.set_reg(REG_DUR, int(self.tap_dur / DUR_SCALE))
        
        else: # "act" mode
            self.act_thresh_x = gcmd.get_float(
                "ACT_THRESH_X", self.act_thresh_x, minval=1, maxval=255
            )
            self.act_thresh_y = gcmd.get_float(
                "ACT_THRESH_Y", self.act_thresh_y, minval=1, maxval=255
            )
            self.act_thresh_z = gcmd.get_float(
                "ACT_THRESH_Z", self.act_thresh_z, minval=1, maxval=255
            )

    def cmd_HOTEND_FAN_OFF(self, gcmd):
        self.control_fans(disable=True)

    def cmd_HOTEND_FAN_ON(self, gcmd):
        self.control_fans(disable=False)




def load_config(config):
    return ADXL345Probe(config)
