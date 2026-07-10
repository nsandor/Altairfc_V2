import time
from drivers.mcp23017 import MCP23017, HIGH, LOW

class IntegratorDriver:
    """
    Driver for controlling IVC102 and ACF2101 switched integrators
    via an MCP23017 I2C GPIO expander.
    
    Pin mapping:
    - GPA2 (pin 2): Soldier IVC102 SW2
    - GPA3 (pin 3): Soldier IVC102 SW1
    - GPA4 (pin 4): Sergeant IVC102 SW1
    - GPA5 (pin 5): Sergeant IVC102 SW2
    - GPB4 (pin 12): ACF2101 HOLD
    - GPB5 (pin 13): ACF2101 RESET
    """

    def __init__(self, io: MCP23017) -> None:
        self.io = io
        
        self.IVC_SOLDIER_SW2 = 2
        self.IVC_SOLDIER_SW1 = 3
        
        self.IVC_SERGEANT_SW1 = 4
        self.IVC_SERGEANT_SW2 = 5
        
        self.ACF_HOLD = 12
        self.ACF_RESET = 13
        
        # Configure pins as outputs
        for pin in [
            self.IVC_SOLDIER_SW2, self.IVC_SOLDIER_SW1,
            self.IVC_SERGEANT_SW1, self.IVC_SERGEANT_SW2,
            self.ACF_HOLD, self.ACF_RESET
        ]:
            self.io.set_output(pin)
            
        self.reset()
        
    def _set_pins_fast(self, s1_sol: int, s2_sol: int, 
                       s1_serg: int, s2_serg: int, 
                       acf_hold: int, acf_rst: int) -> None:
        """
        Updates the MCP23017 GPIO states directly using grouped I2C transactions
        for much better timing accuracy compared to setting one pin at a time.
        """
        # Port A (Pins 0-7)
        mask_a = (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5)
        val_a = (s2_sol << 2) | (s1_sol << 3) | (s1_serg << 4) | (s2_serg << 5)
        
        # Port B (Pins 8-15 -> Bits 0-7 of Port B)
        mask_b = (1 << 4) | (1 << 5)
        val_b = (acf_hold << 4) | (acf_rst << 5)
        
        # Modify shadows to keep state consistent
        self.io._gpio[0] = (self.io._gpio[0] & ~mask_a) | val_a
        self.io._gpio[1] = (self.io._gpio[1] & ~mask_b) | val_b
        
        # Write directly to GPIOA (0x12) and GPIOB (0x13) to minimize I2C latency overhead
        self.io._bus.write_byte_data(self.io._addr, 0x12, self.io._gpio[0])
        self.io._bus.write_byte_data(self.io._addr, 0x13, self.io._gpio[1])

    def reset(self) -> None:
        """
        Reset the integrators.
        IVC102: SW1 open, SW2 closed.
        ACF2101: HOLD low, RESET high.
        """
        self._set_pins_fast(
            s1_sol=LOW, s2_sol=HIGH,
            s1_serg=LOW, s2_serg=HIGH,
            acf_hold=LOW, acf_rst=HIGH
        )
        
    def integrate_and_hold(self, time_us: float) -> None:
        """
        Integrate for time_us microseconds, and then hold.
        Uses busy waiting with time.perf_counter for the highest possible timing accuracy.
        Note: I2C write latency (typically ~100-300us depending on bus speed) will add
        some baseline delay to the exact moment integration starts and stops.
        """
        wait_s = time_us / 1_000_000.0
        
        # Begin Integration
        # IVC102: SW1 closed, SW2 open.
        # ACF2101: HOLD low, RESET low.
        self._set_pins_fast(
            s1_sol=HIGH, s2_sol=LOW,
            s1_serg=HIGH, s2_serg=LOW,
            acf_hold=LOW, acf_rst=LOW
        )
        
        target_time = time.perf_counter() + wait_s
        
        # Spin for the requested integration time
        while time.perf_counter() < target_time:
            pass
            
        # Hold
        # IVC102: SW1 open, SW2 open.
        # ACF2101: HOLD high, RESET low.
        self._set_pins_fast(
            s1_sol=LOW, s2_sol=LOW,
            s1_serg=LOW, s2_serg=LOW,
            acf_hold=HIGH, acf_rst=LOW
        )
