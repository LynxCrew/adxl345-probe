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

ADXL345_REST_TIME = 0.1


class AccelerometerEndstopWrapper:
    def __init__(self, accelerometer_endstop, axis):
        self.accelerometer_endstop = accelerometer_endstop
        self.printer = accelerometer_endstop.printer
        self.gcode = accelerometer_endstop.gcode
        self.axis = axis
        self.mcu_endstop = None
        self.aclient = None
        self.tap_thresh = self.accelerometer_endstop.config.getfloat(
            f"tap_thresh_{axis}",
            self.accelerometer_endstop.default_tap_thresh,
            minval=TAP_SCALE,
            maxval=100000.0,
        )
        self.untap_thresh = self.accelerometer_endstop.config.getfloat(
            f"untap_thresh_{axis}",
            self.accelerometer_endstop.default_untap_thresh,
            minval=TAP_SCALE,
            maxval=100000.0,
        )
        self.tap_dur = self.accelerometer_endstop.config.getfloat(
            f"tap_dur_{axis}",
            self.accelerometer_endstop.default_tap_dur,
            above=DUR_SCALE,
            maxval=0.1,
        )
        self.stepper_enable_dwell_time = config.getfloat(
            f"stepper_enable_dwell_time_{axis}",
            self.accelerometer_endstop.default_stepper_enable_dwell_time,
            minval=0.0
        )

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
        self.mcu_endstop = self.accelerometer_endstop.endstop
        return self.mcu_endstop

    def handle_homing_move_begin(self, hmove, axis=None):
        if self.mcu_endstop not in hmove.get_mcu_endstops() or axis != self.axis:
            return

        self.accelerometer_endstop.accelerometer.init_accelerometer(self.axis)
        if self.accelerometer_endstop.log_homing_data:
            self.aclient = (
                self.accelerometer_endstop.accelerometer.adxl345.start_internal_client()
            )

        self.accelerometer_endstop.probe_prepare(hmove, axis=self.axis)

    def handle_homing_move_end(self, hmove, axis=None):
        if self.mcu_endstop not in hmove.get_mcu_endstops() or axis != self.axis:
            return

        if self.accelerometer_endstop.log_homing_data:
            self.aclient.finish_measurements()
            raw_name = self.get_filename()
            self.aclient.write_to_file(raw_name)
            self.gcode.respond_info("Writing homing data to %s file" % raw_name)
        self.accelerometer_endstop.probe_finish(hmove, axis=self.axis)

    def get_filename(self):
        name = "adxl_homing-"
        time = datetime.datetime.now()
        return os.path.join("/tmp", name + time.strftime("%Y-%m-%d_%H:%M:%S") + ".csv")


class AccelerometerEndstop:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.toolhead = None
        self.gcode = self.printer.lookup_object("gcode")
        gcode_macro = self.printer.load_object(config, "gcode_macro")
        self.phoming = self.printer.load_object(config, "homing")
        self.stepper_enable = self.printer.load_object(self.config, "stepper_enable")
        self.activate_gcode = gcode_macro.load_template(config, "activate_gcode", "")
        self.deactivate_gcode = gcode_macro.load_template(
            config, "deactivate_gcode", ""
        )
        accelerometer_name = config.get("chip", "adxl345")
        self.inverted = False
        self._in_multi_probe = False

        self.default_tap_thresh = config.getfloat(
            "tap_thresh", 5000, minval=TAP_SCALE, maxval=100000.0
        )
        self.default_untap_thresh = config.getfloat(
            "untap_thresh", self.default_tap_thresh, minval=TAP_SCALE, maxval=100000.0
        )
        self.default_tap_dur = config.getfloat(
            "tap_dur", 0.01, above=DUR_SCALE, maxval=0.1
        )
        self.default_stepper_enable_dwell_time = config.getfloat(
            "stepper_enable_dwell_time", 0.1, minval=0.0
        )
        self.disable_fans = [
            fan.strip() for fan in config.get("disable_fans", "").split(",") if fan
        ]

        self.accelerometer = (
            BeaconEndstop(self.config, accelerometer_name, self)
            if "beacon" in accelerometer_name
            else ADXL345Endstop(self.config, self.printer, accelerometer_name, self)
        )
        self.next_cmd_time = self.action_end_time = 0.0

        self.enable_x_homing = config.getboolean("enable_x_homing", False)
        self.enable_y_homing = config.getboolean("enable_y_homing", False)
        self.enable_probe = False
        if "beacon" not in accelerometer_name:
            self.enable_probe = config.getboolean("enable_probe", True)
        self.log_homing_data = config.getboolean("log_homing_data", False)
        # Add wrapper methods for endstops
        self.mcu_endstop = self.accelerometer.enstop
        self.get_mcu = self.accelerometer.get_mcu
        self.add_stepper = self.accelerometer.add_stepper
        self.get_steppers = self.accelerometer.get_steppers
        self.probing_move = self.phoming.probing_move
        self.probe_prepare = self.accelerometer.probe_prepare
        self.probe_finish = self.accelerometer.probe_finish
        self.home_start = self.accelerometer.home_start
        self.home_wait = self.accelerometer.home_wait
        self.query_endstop = self.accelerometer.query_endstop
        self.registered_endstops = {}
        # Register commands and callbacks
        if self.enable_x_homing:
            self.registered_endstops["x"] = AccelerometerEndstopWrapper(self, "x")
            ppins.register_chip(
                "accelerometer_endstop_x", self.registered_endstops["x"]
            )
        if self.enable_y_homing:
            self.registered_endstops["y"] = AccelerometerEndstopWrapper(self, "y")
            ppins.register_chip(
                "accelerometer_endstop_y", self.registered_endstops["y"]
            )
        if self.enable_probe:
            self.registered_endstops["z"] = self
            self.printer.add_object("probe", self)

            self.position_endstop = config.getfloat("z_offset")

            self.tap_thresh = config.getfloat(
                "tap_thresh_z",
                self.default_tap_thresh,
                minval=TAP_SCALE,
                maxval=100000.0,
            )
            self.untap_thresh = config.getfloat(
                "untap_thresh_z",
                self.default_untap_thresh,
                minval=TAP_SCALE,
                maxval=100000.0,
            )
            self.tap_dur = config.getfloat(
                "tap_dur_z", self.default_tap_dur, above=DUR_SCALE, maxval=0.1
            )
            self.stepper_enable_dwell_time = config.getfloat(
                "stepper_enable_dwell_time_z", self.default_stepper_enable_dwell_time, minval=0.0
            )
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.printer.register_event_handler(
            "klippy:mcu_identify", self.handle_mcu_identify
        )
        self.printer.register_event_handler(
            "homing:home_rails_begin", self.handle_homing_rails_begin
        )
        self.printer.register_event_handler(
            "homing:home_rails_end", self.handle_homing_rails_end
        )

    def handle_connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")
        self.accelerometer.init_accelerometer()

    def handle_mcu_identify(self):
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis("z"):
                self.add_stepper(stepper)

    def control_fans(self, disable=True):
        for fan in self.disable_fans:
            fan = self.printer.lookup_object(fan)
            if disable:
                fan._fan_speed = fan.fan_speed
                fan.fan_speed = 0
            else:
                fan.fan_speed = fan._fan_speed
                fan._fan_speed = 0

    def multi_probe_begin(self):
        self._in_multi_probe = True
        self.control_fans(disable=True)

    def multi_probe_end(self):
        self.control_fans(disable=False)
        self._in_multi_probe = False

    def get_position_endstop(self):
        return self.position_endstop

    def handle_homing_rails_begin(self, homing, rails):
        dwell_time = 0.0
        homing_axes = [rail.get_name(short=True) for rail in rails]
        affected_rails = set()
        for axis_name in homing_axes:
            if axis_name in self.registered_endstops.keys():
                partial_rails = self.toolhead.get_active_rails_for_axis(axis_name)
                affected_rails = affected_rails | set(partial_rails)
                dwell_time = max(
                    dwell_time,
                    self.registered_endstops[axis_name].stepper_enable_dwell_time,
                )

        for rail in affected_rails:
            for stepper in rail.get_steppers():
                self.gcode.respond_info(stepper.get_name())
                self.stepper_enable.motor_debug_enable(stepper.get_name(), True)

        self.toolhead.dwell(dwell_time)

    def handle_homing_rails_end(self, homing, rails):
        pass


class ADXL345Endstop:
    def __init__(self, config, printer, adxl_name, accelerometer_endstop):
        self.printer = printer
        self.adxl345 = self.printer.load_object(config, adxl_name)
        self.accelerometer_endstop = accelerometer_endstop
        self.is_measuring = False

        int_pin = config.get("int_pin").strip()
        if int_pin.startswith("!"):
            self.inverted = True
            int_pin = int_pin[1:].strip()
        if int_pin != "int1" and int_pin != "int2":
            raise config.error("int_pin must specify one of int1 or int2 pins")
        probe_pin = config.get("probe_pin")
        self.int_map = 0x40 if int_pin == "int2" else 0x0

        # Create an "endstop" object to handle the sensor pin
        ppins = self.printer.lookup_object("pins")
        pin_params = ppins.lookup_pin(probe_pin, can_invert=True, can_pullup=True)
        mcu = pin_params["chip"]
        self.endstop = mcu.setup_pin("endstop", pin_params)

        self.get_mcu = self.endstop.get_mcu
        self.add_stepper = self.endstop.add_stepper
        self.get_steppers = self.endstop.get_steppers
        self.query_endstop = self.endstop.query_endstop
        self.home_start = self.endstop.home_start
        self.home_wait = self.endstop.home_wait

        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_mux_command(
            "SET_ACCEL_ENDSTOP",
            "CHIP",
            None,
            self.cmd_SET_ACCEL_ENDSTOP,
            desc=self.cmd_SET_ACCEL_ENDSTOP_help,
        )

    def init_accelerometer(self, axis=None):
        tap_thresh = self.accelerometer_endstop.default_tap_thresh
        tap_dur = self.accelerometer_endstop.default_tap_dur
        if axis is not None:
            tap_thresh = self.accelerometer_endstop.registered_endstops[axis].tap_thresh
            tap_dur = self.accelerometer_endstop.registered_endstops[axis].tap_dur
        chip = self.adxl345
        chip.set_reg(adxl345.REG_POWER_CTL, 0x00)
        chip.set_reg(adxl345.REG_DATA_FORMAT, 0x0B)
        if self.inverted:
            chip.set_reg(adxl345.REG_DATA_FORMAT, 0x2B)
        chip.set_reg(REG_INT_MAP, self.int_map)
        chip.set_reg(REG_TAP_AXES, 0x7)
        chip.set_reg(REG_THRESH_TAP, int(tap_thresh / TAP_SCALE))
        chip.set_reg(REG_DUR, int(tap_dur / DUR_SCALE))

    def try_clear_tap(self):
        chip = self.adxl345
        tries = 8
        while tries > 0:
            val = chip.read_reg(REG_INT_SOURCE)
            if not (val & 0x40):
                return True
            tries -= 1
        return False

    def probe_prepare(self, hmove, axis="z"):
        self.init_accelerometer(axis)
        self.accelerometer_endstop.activate_gcode.run_gcode_from_command()
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.flush_step_generation()
        toolhead.dwell(ADXL345_REST_TIME)
        print_time = toolhead.get_last_move_time()
        clock = self.adxl345.mcu.print_time_to_clock(print_time)
        self.adxl345.set_reg(REG_INT_ENABLE, 0x00, minclock=clock)
        self.adxl345.read_reg(REG_INT_SOURCE)
        self.adxl345.set_reg(REG_INT_ENABLE, 0x40, minclock=clock)
        self.is_measuring = self.adxl345.read_reg(adxl345.REG_POWER_CTL) == 0x08
        if not self.is_measuring:
            self.adxl345.set_reg(adxl345.REG_POWER_CTL, 0x08, minclock=clock)
        if not self.try_clear_tap():
            raise self.printer.command_error(
                "ADXL345 tap triggered before move, it may be set too sensitive."
            )
        if axis != "z" or not self.accelerometer_endstop._in_multi_probe:
            self.control_fans(disable=True)

    def probe_finish(self, hmove, axis="z"):
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.dwell(ADXL345_REST_TIME)
        print_time = toolhead.get_last_move_time()
        clock = self.adxl345.mcu.print_time_to_clock(print_time)
        self.adxl345.set_reg(REG_INT_ENABLE, 0x00, minclock=clock)
        if not self.is_measuring:
            self.adxl345.set_reg(adxl345.REG_POWER_CTL, 0x00)
        self.accelerometer_endstop.deactivate_gcode.run_gcode_from_command()
        if not self.try_clear_tap():
            raise self.printer.command_error(
                "ADXL345 tap triggered after move, it may be set too sensitive."
            )
        if axis != "z" or not self.accelerometer_endstop._in_multi_probe:
            self.control_fans(disable=False)
        # self.init_adxl()

    cmd_SET_ACCEL_ENDSTOP_help = "Configure ADXL345 parameters related to probing"

    def cmd_SET_ACCEL_ENDSTOP(self, gcmd):
        axes_str = gcmd.get("AXES", None)
        if axes_str is None:
            axes = self.accelerometer_endstop.registered_endstops.keys()
        else:
            axes = axes_str.split(",")
        for axis in axes:
            self.accelerometer_endstop.registered_endstops[
                axis
            ].tap_thresh = gcmd.get_float(
                "TAP_THRESH", self.tap_thresh_default, minval=TAP_SCALE, maxval=100000.0
            )
            self.accelerometer_endstop.registered_endstops[
                axis
            ].tap_dur = gcmd.get_float(
                "TAP_DUR", self.tap_dur_default, above=DUR_SCALE, maxval=0.1
            )


class BeaconEndstop:
    def __init__(self, config, beacon_name, accelerometer_endstop):
        self.printer = printer
        self.beacon = self.printer.load_object(config, beacon_name)
        self._mcu = self.beacon._mcu
        self._shared = self.beacon._endstop_shared
        self.accelerometer_endstop = accelerometer_endstop
        self.beacon_accel_set_threshhold = None
        self.beacon_accel_home = None
        self.beacon_accel_stop_home = None
        self.beacon_accel_home_query = None
        self.is_homing = False
        self._mcu.register_config_callback(self._build_config)

    def _build_config(self):
        version_info = self._check_mcu_version()

        try:
            self.beacon_accel_set_threshhold = self._mcu.lookup_command(
                "beacon_accel_set_threshold trigger=%hu untrigger=%hu duration=%hu",
                cq=self.beacon.command_queue,
            )
            self.beacon_accel_home = self._mcu.lookup_command(
                "beacon_accel_home trsync_oid=%c trigger_reason=%c",
                cq=self.beacon.command_queue,
            )
            self.beacon_accel_stop_home = self._mcu.lookup_command(
                "beacon_accel_stop_home", cq=self.beacon.command_queue
            )
            self.beacon_accel_home_query = self._mcu.lookup_command(
                "beacon_accel_home_query",
                "beacon_accel_home_state triggered=%c detect_clock=%u",
                cq=self.beacon.command_queue,
            )
        except msgproto.error as e:
            if version_info != "":
                raise msgproto.error(version_info + "\n\n" + str(e))
            raise

    def get_mcu(self):
        return self._mcu

    def add_stepper(self, stepper):
        self._shared.add_stepper(stepper)

    def get_steppers(self):
        return self._shared.get_steppers()

    def init_accelerometer(self, axis=None):
        tap_thresh = self.accelerometer_endstop.default_tap_thresh
        untap_thresh = self.accelerometer_endstop.default_untap_thresh
        tap_dur = self.accelerometer_endstop.default_tap_dur
        if axis is not None:
            tap_thresh = self.accelerometer_endstop.registered_endstops[axis].tap_thresh
            untap_thresh = self.accelerometer_endstop.registered_endstops[
                axis
            ].untap_thresh
            tap_dur = self.accelerometer_endstop.registered_endstops[axis].tap_dur
        if self.beacon_accel_set_threshhold is not None:
            self.beacon_accel_set_threshhold.send(tap_thresh, untap_thresh, tap_dur)

    def probe_prepare(self, hmove, axis="z"):
        self.init_accelerometer(axis)
        self.accelerometer_endstop.activate_gcode.run_gcode_from_command()
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.flush_step_generation()
        toolhead.dwell(ADXL345_REST_TIME)
        if axis != "z" or not self.accelerometer_endstop._in_multi_probe:
            self.control_fans(disable=True)

    def probe_finish(self, hmove, axis="z"):
        chip = self.adxl345
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.dwell(ADXL345_REST_TIME)
        self.accelerometer_endstop.deactivate_gcode.run_gcode_from_command()
        if axis != "z" or not self.accelerometer_endstop._in_multi_probe:
            self.control_fans(disable=False)

    def home_start(
        self, print_time, sample_time, sample_count, rest_time, triggered=True
    ):
        self.is_homing = True
        self.beacon._sample_async()
        self._shared.trsync_start(print_time)
        etrsync = self.beacon._endstop_shared._trsync
        self.beacon_accel_home(
            [
                etrsync.get_oid(),
                etrsync.REASON_ENDSTOP_HIT,
            ]
        )
        return self._shared._trigger_completion

    def home_wait(self, home_end_time):
        try:
            ret = self._shared.trsync_stop(home_end_time)
            if ret is not None:
                return ret
            if self._mcu.is_fileoutput():
                return home_end_time
            self.beacon.toolhead.wait_moves()
            deadline = self.beacon.reactor.monotonic() + 0.5
            while True:
                ret = self.beacon_accel_home_query.send([])
                if ret["triggered"] == 0:
                    now = self.beacon.reactor.monotonic()
                    if now >= deadline:
                        raise self.beacon.printer.command_error(
                            "Timeout getting contact time"
                        )
                    self.beacon.reactor.pause(now + 0.001)
                    continue
                time = self.beacon._clock32_to_time(ret["detect_clock"])
                ffi_main, ffi_lib = chelper.get_ffi()
                data = ffi_main.new("struct pull_move[1]")
                count = ffi_lib.trapq_extract_old(self.beacon.trapq, data, 1, 0.0, time)
                if time >= home_end_time:
                    return 0.0
                if count:
                    accel = data[0].accel
                    if accel < 0:
                        logging.info("Accelerometer triggered while decelerating")
                        raise self.beacon.printer.command_error(
                            "No trigger on probe after full movement"
                        )
                    elif accel > 0:
                        raise self.beacon.printer.command_error(
                            "Accelerometer triggered while accelerating"
                        )
                    return time
        finally:
            self.beacon.beacon_contact_stop_home_cmd.send()

    def query_endstop(self, print_time):
        return 0


def load_config(config):
    return AccelerometerEndstop(config)
