import time
from drivers.mcp23017 import MCP23017, HIGH, LOW


class IntegratorDriver:
    """
    Driver for controlling IVC102 and ACF2101 switched integrators
    via an MCP23017 I2C GPIO expander.

    Pin mapping:
    """

    def __init__(self, io: MCP23017) -> None:
        self.io = io
        self.PD_Reset = 3
        self.IVC_SOLDIER_SW2 = 7
        self.IVC_SOLDIER_SW1 = 6

        self.IVC_SERGEANT_SW1 = 13
        self.IVC_SERGEANT_SW2 = 12

        self.ACF_HOLD = 5
        self.ACF_RESET = 4

        # Configure pins as outputs
        for pin in [
            self.PD_Reset,
            self.IVC_SOLDIER_SW2,
            self.IVC_SOLDIER_SW1,
            self.IVC_SERGEANT_SW1,
            self.IVC_SERGEANT_SW2,
            self.ACF_HOLD,
            self.ACF_RESET,
        ]:
            self.io.set_output(pin)

        self.reset()
        self.io.set(self.PD_Reset, HIGH)

    def _set_pins_fast(
        self,
        s1_sol: int,
        s2_sol: int,
        s1_serg: int,
        s2_serg: int,
        acf_hold: int,
        acf_rst: int,
    ) -> None:
        """
        Updates the MCP23017 GPIO states directly using grouped I2C transactions
        for much better timing accuracy compared to setting one pin at a time.
        """
        # Port A (Pins 0-7)
        mask_a = (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7)
        val_a = (acf_rst << 4) | (acf_hold << 5) | (s1_sol << 6) | (s2_sol << 7)

        # Port B (Pins 8-15 -> Bits 0-7 of Port B)
        mask_b = (1 << 4) | (1 << 5)
        val_b = (s2_serg << 4) | (s1_serg << 5)

        # Modify shadows to keep state consistent
        self.io._gpio[0] = (self.io._gpio[0] & ~mask_a) | val_a
        self.io._gpio[1] = (self.io._gpio[1] & ~mask_b) | val_b

        # Write both GPIOA (0x12) and GPIOB (0x13) in a single I2C transaction
        # using block write to halve I2C latency overhead.
        # This relies on the MCP23017's default IOCON.SEQOP=0 (auto-increment).
        self.io._bus.write_i2c_block_data(self.io._addr, 0x12, [self.io._gpio[0], self.io._gpio[1]])

    def reset(self) -> None:
        """
        Reset the integrators.
        IVC102: SW1 open, SW2 closed.
        ACF2101: HOLD low, RESET high.
        """
        self._set_pins_fast(
            s1_sol=LOW,
            s2_sol=LOW,
            s1_serg=LOW,
            s2_serg=LOW,
            acf_hold=LOW,
            acf_rst=LOW,
        )

    def integrate_and_hold(
        self, time_us: float, print_timing: bool = True
    ) -> tuple[float, float]:
        """
        Integrate for time_us microseconds, and then hold.
        Uses busy waiting with time.perf_counter for the highest possible timing accuracy.
        Note: I2C write latency (typically ~100-300us depending on bus speed) will add
        some baseline delay to the exact moment integration starts and stops.

        Returns:
            tuple[float, float]: The exact start and end perf_counter timestamps.

        Set print_timing=False when the caller records or displays the returned
        timing itself.
        """
        wait_s = time_us / 1_000_000.0

        # Begin Integration
        # IVC102: SW1 closed, SW2 open.
        # ACF2101: HOLD low, RESET low.
        self._set_pins_fast(
            s1_sol=LOW,
            s2_sol=HIGH,
            s1_serg=LOW,
            s2_serg=HIGH,
            acf_hold=LOW,
            acf_rst=HIGH,
        )

        t_start = time.perf_counter()
        target_time = t_start + wait_s

        # Spin for the requested integration time
        while time.perf_counter() < target_time:
            pass

        # Hold
        # IVC102: SW1 open, SW2 open.
        # ACF2101: HOLD high, RESET low.
        self._set_pins_fast(
            s1_sol=HIGH,
            s2_sol=HIGH,
            s1_serg=HIGH,
            s2_serg=HIGH,
            acf_hold=HIGH,
            acf_rst=HIGH,
        )

        t_end = time.perf_counter()
        actual_time_us = (t_end - t_start) * 1_000_000.0
        if print_timing:
            print(
                f"Integration started at {t_start:.6f}s and ended at {t_end:.6f}s "
                f"(Actual duration: {actual_time_us:.2f} us)"
            )

        return t_start, t_end
