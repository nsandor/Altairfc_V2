from __future__ import annotations

import logging
import time

from config.settings import RadioConfig
from core.datastore import DataStore
from core.task_base import BaseTask
from tasks.flight_stage_task import STAGE_ARMED
from telemetry.transport import SerialTransport

logger = logging.getLogger(__name__)

# Seconds to wait after ACK is sent before switching channels, giving the GS
# time to receive the ACK on the old channel before we move.
_ACK_PROPAGATION_S = 0.3


class RadioConfigTask(BaseTask):
    """
    Manages LR900P radio configuration via the shared SerialTransport.

    Startup:
      Reads the current modem config; applies settings.toml values if they differ.

    Runtime (execute at 2 Hz):
      Polls command.radio_* DataStore keys written by RadioConfigCommandPacket.
      Blocked when flight stage >= STAGE_ARMED.

    Channel-change handshake:
      1. CommandReceiverTask sends the ACK on the current channel (before this
         task ever sees the command keys — ACK happens synchronously in
         CommandReceiverTask._dispatch()).
      2. This task waits _ACK_PROPAGATION_S so the ACK frame finishes
         transmitting, then switches channels.
      3. A watchdog timer starts. Every command received from the GS on the new
         channel (any command) resets it via system.last_gs_contact_t.
      4. If the watchdog expires with no GS contact, the config rolls back to
         the settings.toml defaults.

    All serial I/O (heartbeat, framing, queues) is handled by SerialTransport.
    """

    def __init__(
        self,
        name: str,
        period_s: float,
        datastore: DataStore,
        transport: SerialTransport,
        radio_config: RadioConfig,
    ) -> None:
        super().__init__(name=name, period_s=period_s, datastore=datastore)
        self._transport = transport
        self._cfg       = radio_config

        # Set when we switch away from toml defaults; cleared on rollback.
        self._watchdog_deadline: float | None = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def setup(self) -> None:
        if not self._transport.wait_until_open(timeout=10.0):
            logger.warning("RadioConfigTask: transport not open after 10 s — skipping startup read")
            return
        cfg = self._transport.read_config(timeout=4.0)
        if cfg is None:
            logger.warning("RadioConfigTask: could not read modem config at startup")
            return
        self._publish(cfg.data_rate, cfg.tx_power, cfg.channel)
        if (cfg.data_rate != self._cfg.data_rate or
                cfg.tx_power != self._cfg.tx_power or
                cfg.channel  != self._cfg.channel):
            logger.info(
                "RadioConfigTask: modem %d/%d/%d differs from toml %d/%d/%d — applying",
                cfg.data_rate, cfg.tx_power, cfg.channel,
                self._cfg.data_rate, self._cfg.tx_power, self._cfg.channel,
            )
            self._switch(self._cfg.data_rate, self._cfg.tx_power, self._cfg.channel)
        else:
            logger.info("RadioConfigTask: modem config matches settings.toml")

    def execute(self) -> None:
        # Check watchdog first — independent of incoming commands.
        self._check_watchdog()

        # Block all config changes once armed or beyond.
        stage = int(self.datastore.read("event.flight_stage", default=0))
        if stage >= STAGE_ARMED:
            # Discard any pending command so it doesn't linger for post-flight.
            self._clear_command_keys()
            return

        dr  = self.datastore.read("command.radio_data_rate", default=None)
        txp = self.datastore.read("command.radio_tx_power",  default=None)
        ch  = self.datastore.read("command.radio_channel",   default=None)
        if all(v is not None for v in (dr, txp, ch)):
            self._clear_command_keys()
            dr, txp, ch = int(dr), int(txp), int(ch)

            is_default = (dr  == self._cfg.data_rate and
                          txp == self._cfg.tx_power  and
                          ch  == self._cfg.channel)

            # Wait for the ACK (sent synchronously by CommandReceiverTask) to
            # finish propagating to the GS before we switch channels.
            self._stop_event.wait(timeout=_ACK_PROPAGATION_S)
            if self._stop_event.is_set():
                return

            self._switch(dr, txp, ch)

            if is_default:
                # Switched back to defaults — cancel the watchdog.
                self._watchdog_deadline = None
                logger.info("RadioConfigTask: restored to toml defaults, watchdog cancelled")
            else:
                # Start watchdog: GS must contact us within the configured window.
                self._watchdog_deadline = time.monotonic() + self._cfg.watchdog_s
                logger.info(
                    "RadioConfigTask: watchdog started (%.0f s) — will rollback if GS silent",
                    self._cfg.watchdog_s,
                )

    def teardown(self) -> None:
        pass

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _check_watchdog(self) -> None:
        if self._watchdog_deadline is None:
            return

        last_contact = self.datastore.read("system.last_gs_contact_t", default=None)
        if last_contact is not None and float(last_contact) > (self._watchdog_deadline - self._cfg.watchdog_s):
            # GS has been in contact since we switched — reset the deadline.
            self._watchdog_deadline = time.monotonic() + self._cfg.watchdog_s
            return

        if time.monotonic() >= self._watchdog_deadline:
            logger.warning(
                "RadioConfigTask: no GS contact for %.0f s — rolling back to toml defaults",
                self._cfg.watchdog_s,
            )
            self._watchdog_deadline = None
            self._switch(self._cfg.data_rate, self._cfg.tx_power, self._cfg.channel)

    def _switch(self, data_rate: int, tx_power: int, channel: int) -> None:
        logger.info("RadioConfigTask: writing data_rate=%d tx_power=%d channel=%d",
                    data_rate, tx_power, channel)
        ack = self._transport.write_config(data_rate, tx_power, channel, timeout=3.0)
        if ack is None:
            logger.warning("RadioConfigTask: write_config ack timed out")
        else:
            logger.info("RadioConfigTask: modem ack received — rebooting")

        # Modem reboots after a config write; wait before re-reading.
        self._stop_event.wait(timeout=3.5)
        if self._stop_event.is_set():
            return

        confirmed = self._transport.read_config(timeout=4.0)
        if confirmed is None:
            logger.warning("RadioConfigTask: post-write read_config timed out")
            return
        self._publish(confirmed.data_rate, confirmed.tx_power, confirmed.channel)
        logger.info("RadioConfigTask: confirmed data_rate=%d tx_power=%d channel=%d",
                    confirmed.data_rate, confirmed.tx_power, confirmed.channel)

    def _publish(self, data_rate: int, tx_power: int, channel: int) -> None:
        self.datastore.write("radio.data_rate", float(data_rate))
        self.datastore.write("radio.tx_power",  float(tx_power))
        self.datastore.write("radio.channel",   float(channel))

    def _clear_command_keys(self) -> None:
        self.datastore.write("command.radio_data_rate", None)
        self.datastore.write("command.radio_tx_power",  None)
        self.datastore.write("command.radio_channel",   None)
