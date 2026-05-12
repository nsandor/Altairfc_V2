from __future__ import annotations

import logging

from config.settings import RadioConfig
from core.datastore import DataStore
from core.task_base import BaseTask
from telemetry.transport import SerialTransport

logger = logging.getLogger(__name__)


class RadioConfigTask(BaseTask):
    """
    Manages LR900P radio configuration via the shared SerialTransport.

    setup():
      Reads the current modem config; applies settings.toml values if they differ.

    execute():
      Polls command.radio_* DataStore keys written by RadioConfigCommandPacket.
      When all three are present, applies the new config and re-reads to confirm.

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

    def setup(self) -> None:
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
            self._apply(self._cfg.data_rate, self._cfg.tx_power, self._cfg.channel)
        else:
            logger.info("RadioConfigTask: modem config matches settings.toml")

    def execute(self) -> None:
        dr  = self.datastore.read("command.radio_data_rate", default=None)
        txp = self.datastore.read("command.radio_tx_power",  default=None)
        ch  = self.datastore.read("command.radio_channel",   default=None)
        if all(v is not None for v in (dr, txp, ch)):
            self.datastore.write("command.radio_data_rate", None)
            self.datastore.write("command.radio_tx_power",  None)
            self.datastore.write("command.radio_channel",   None)
            self._apply(int(dr), int(txp), int(ch))

    def teardown(self) -> None:
        pass

    # ------------------------------------------------------------------

    def _apply(self, data_rate: int, tx_power: int, channel: int) -> None:
        logger.info("RadioConfigTask: writing data_rate=%d tx_power=%d channel=%d",
                    data_rate, tx_power, channel)
        ack = self._transport.write_config(data_rate, tx_power, channel, timeout=3.0)
        if ack is None:
            logger.warning("RadioConfigTask: write_config ack timed out")
        else:
            logger.info("RadioConfigTask: ack received — modem rebooting")
        # Modem reboots after a write; wait before re-reading.
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
