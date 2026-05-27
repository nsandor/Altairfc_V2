from __future__ import annotations

import logging
import math
import time

from pymavlink import mavutil

from config.settings import SerialPortConfig
from core.datastore import DataStore
from core.task_base import BaseTask

logger = logging.getLogger(__name__)

# MAVLink message types this task subscribes to.
# GPS_RAW_INT is used instead of GLOBAL_POSITION_INT — PX4 streams it by default.
# LOCAL_POSITION_NED provides relative altitude (above home) since GPS_RAW_INT omits it.
_SUBSCRIBED_TYPES = ("ATTITUDE", "GPS_RAW_INT", "GPS2_RAW", "LOCAL_POSITION_NED", "SCALED_PRESSURE", "VFR_HUD")


class MavlinkTask(BaseTask):
    """
    Reads MAVLink messages from the Pixhawk 6X mini and writes them to the DataStore.

    Runs at 50 Hz (configurable). Uses non-blocking recv_match() so the scheduler
    controls cadence. Multiple message types can be added to _SUBSCRIBED_TYPES
    without changing the task loop.

    DataStore keys written:
        mavlink.attitude.roll        (float, rad)
        mavlink.attitude.pitch       (float, rad)
        mavlink.attitude.yaw         (float, rad)
        mavlink.attitude.rollspeed   (float, rad/s)
        mavlink.attitude.pitchspeed  (float, rad/s)
        mavlink.attitude.yawspeed    (float, rad/s)
        mavlink.gps.lat              (float, deg)   — from GPS_RAW_INT
        mavlink.gps.lon              (float, deg)
        mavlink.gps.alt              (float, m)     — MSL altitude
        mavlink.gps.relative_alt     (float, m)     — above home, from LOCAL_POSITION_NED (-z)
        mavlink.gps.hdg              (float, deg)   — vehicle heading 0-360, from GPS_RAW_INT
        mavlink.gps.num_sv           (int)          — satellites visible, from GPS_RAW_INT
        mavlink.heading              (float, deg)   — dual-antenna yaw 0-360, from GPS2_RAW (only written when valid)
        mavlink.environment.press_abs    (float, hPa)  — from SCALED_PRESSURE
        mavlink.environment.press_diff   (float, hPa)
        mavlink.environment.temperature  (float, °C)   — centidegrees converted
        mavlink.environment.baro_alt     (float, m)    — from VFR_HUD
        mavlink.environment.climb        (float, m/s)
        mavlink.environment.airspeed     (float, m/s)
        mavlink.environment.groundspeed  (float, m/s)
        system.pixhawk_connected         (float, 0.0/1.0)
    """

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        port_config: SerialPortConfig,
        heartbeat_timeout_s: float = 10.0,
        connect_retry_s: float = 5.0,
    ) -> None:
        super().__init__(name, period_s, datastore)
        self._port_config = port_config
        self._heartbeat_timeout_s = heartbeat_timeout_s
        self._connect_retry_s = connect_retry_s
        self._master = None

    def setup(self) -> None:
        while not self._stop_event.is_set():
            try:
                logger.info(
                    "MavlinkTask: connecting to %s @ %d baud",
                    self._port_config.port,
                    self._port_config.baud,
                )
                self._master = mavutil.mavlink_connection(
                    self._port_config.port,
                    baud=self._port_config.baud,
                )
                self._master.wait_heartbeat(timeout=self._heartbeat_timeout_s)
                logger.info(
                    "MavlinkTask: heartbeat received (system %d, component %d)",
                    self._master.target_system,
                    self._master.target_component,
                )
                self.datastore.write("system.pixhawk_connected", 1.0)
                self._request_message_rates()
                return  # connected successfully
            except Exception as e:
                logger.debug(
                    "MavlinkTask: connection failed (%s) — retrying in %.0fs",
                    e,
                    self._connect_retry_s,
                )
                self.datastore.write("system.pixhawk_connected", 0.0)
                self._stop_event.wait(timeout=self._connect_retry_s)

    def _request_message_rates(self) -> None:
        """
        Explicitly ask the Pixhawk to stream required message types.
        Uses MAV_CMD_SET_MESSAGE_INTERVAL (command 511).
        Interval is in microseconds; -1 disables, 0 = default rate.
        """
        # (message_id, interval_us)
        requests = [
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,            20_000),   # 50 Hz
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 20_000),   # 50 Hz 
            (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,        200_000),   #  5 Hz
            (mavutil.mavlink.MAVLINK_MSG_ID_GPS2_RAW,           200_000),   #  5 Hz
            (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 200_000),   #  5 Hz
            (mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE,    200_000),   #  5 Hz
            (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,            200_000),   #  5 Hz
        ]
        for msg_id, interval_us in requests:
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,           # confirmation
                msg_id,      # param1: message ID
                interval_us, # param2: interval in microseconds
                0, 0, 0, 0,  # params 3-6 unused
                0,           # param7 unused
            )
            logger.info(
                "MavlinkTask: requested MSG_ID=%d at %.1f Hz",
                msg_id, 1e6 / interval_us,
            )

    def execute(self) -> None:
        if self._master is None:
            return
        try:
            msg = self._master.recv_match(type=list(_SUBSCRIBED_TYPES), blocking=True, timeout=self.period_s)
            if msg is not None:
                self._handle_message(msg)
                while True:
                    msg = self._master.recv_match(type=list(_SUBSCRIBED_TYPES), blocking=False)
                    if msg is None:
                        break
                    self._handle_message(msg)
        except Exception as e:
            logger.warning("MavlinkTask: serial error — reconnecting (%s)", e)
            self.datastore.write("system.pixhawk_connected", 0.0)
            try:
                self._master.close()
            except Exception:
                pass
            self._master = None
            self.setup()

    @staticmethod
    def _f(value: float, fallback: float = 0.0) -> float:
        """Return fallback if value is NaN or infinite, else value."""
        return fallback if not math.isfinite(value) else value

    def _handle_message(self, msg) -> None:
        f = self._f
        msg_type = msg.get_type()
        if msg_type == "ATTITUDE":
            self.datastore.write("mavlink.attitude.roll",       f(msg.roll))
            self.datastore.write("mavlink.attitude.pitch",      f(msg.pitch))
            self.datastore.write("mavlink.attitude.yaw",        f(msg.yaw))
            self.datastore.write("mavlink.attitude.rollspeed",  f(msg.rollspeed))
            self.datastore.write("mavlink.attitude.pitchspeed", f(msg.pitchspeed))
            self.datastore.write("mavlink.attitude.yawspeed",   f(msg.yawspeed))

        elif msg_type == "ATTITUDE_QUATERNION":
            self.datastore.write("mavlink.quaternion.w", f(msg.q1))
            self.datastore.write("mavlink.quaternion.x", f(msg.q2))
            self.datastore.write("mavlink.quaternion.y", f(msg.q3))
            self.datastore.write("mavlink.quaternion.z", f(msg.q4))
            
        elif msg_type == "GPS_RAW_INT":
            # lat/lon in 1e-7 deg, alt in mm, cog (course over ground) in cdeg
            self.datastore.write("mavlink.gps.lat", f(msg.lat / 1e7))
            self.datastore.write("mavlink.gps.lon", f(msg.lon / 1e7))
            self.datastore.write("mavlink.gps.alt", f(msg.alt / 1e3))
            self.datastore.write("mavlink.gps.hdg", f(msg.cog / 1e2))
            self.datastore.write("mavlink.gps.num_sv", int(msg.satellites_visible))

        elif msg_type == "GPS2_RAW":
            # yaw in cdeg from dual-antenna RTK source; 0 = unavailable, 65535 = no fix yet
            yaw_cdeg = msg.yaw
            if yaw_cdeg not in (0, 65535):
                self.datastore.write("mavlink.heading", f(yaw_cdeg / 1e2))

        elif msg_type == "LOCAL_POSITION_NED":
            # NED frame: z is positive downward, so relative_alt = -z
            self.datastore.write("mavlink.gps.relative_alt", f(-msg.z))

        elif msg_type == "SCALED_PRESSURE":
            self.datastore.write("mavlink.environment.press_abs",   f(msg.press_abs))
            self.datastore.write("mavlink.environment.press_diff",  f(msg.press_diff))
            self.datastore.write("mavlink.environment.temperature", f(msg.temperature / 100.0))

        elif msg_type == "VFR_HUD":
            self.datastore.write("mavlink.environment.baro_alt",    f(msg.alt))
            self.datastore.write("mavlink.environment.climb",       f(msg.climb))
            self.datastore.write("mavlink.environment.airspeed",    f(msg.airspeed))
            self.datastore.write("mavlink.environment.groundspeed", f(msg.groundspeed))

    def teardown(self) -> None:
        if self._master is not None:
            try:
                self._master.close()
            except Exception:
                pass
            self._master = None
            logger.info("MavlinkTask: connection closed")
