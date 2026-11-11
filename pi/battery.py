"""Low-rate, non-blocking battery voltage sampling for the PiRacer Pro INA219."""
from __future__ import annotations

import threading
import time


class BatteryMonitor:
    """Read INA219 bus voltage off the vehicle loop and expose the latest sample.

    The PiRacer Pro INA219 is commonly found at 0x42 on I2C bus 1. The address is
    configurable because board revisions can differ. Only the voltage register is
    read: current readings require a known INA219 calibration setup.
    """

    def __init__(self, bus=1, address=0x42, interval_s=1.0, clock=time.monotonic):
        self.bus_number = int(bus)
        self.address = int(address)
        self.interval_s = max(0.25, float(interval_s))
        self.clock = clock
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._bus = None
        self._voltage_v = None
        self._sample_t = None
        self._error = "Waiting for battery sensor"

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._poll, name="battery-monitor", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)
        bus, self._bus = self._bus, None
        if bus is not None:
            try:
                bus.close()
            except Exception:
                pass

    def snapshot(self):
        with self._lock:
            voltage, sampled, error = self._voltage_v, self._sample_t, self._error
        age = None if sampled is None else max(0.0, self.clock() - sampled)
        return {"voltage_v": voltage, "age_s": age, "available": voltage is not None and age < 4.0,
                "error": error}

    def _open_bus(self):
        try:
            from smbus2 import SMBus
        except ImportError:
            from smbus import SMBus
        self._bus = SMBus(self.bus_number)

    def _read_voltage(self):
        if self._bus is None:
            self._open_bus()
        # INA219 register 0x02 is bus voltage. Read the two register bytes in
        # device order, matching Waveshare's INA219 driver.
        data = self._bus.read_i2c_block_data(self.address, 0x02, 2)
        raw = (data[0] << 8) | data[1]
        voltage = ((raw >> 3) & 0x1FFF) * 0.004
        if not 5.0 <= voltage <= 15.0:
            raise OSError(f"INA219 returned implausible pack voltage ({voltage:.2f} V)")
        return round(voltage, 3)

    def _poll(self):
        while not self._stop.is_set():
            try:
                voltage = self._read_voltage()
                with self._lock:
                    self._voltage_v = voltage
                    self._sample_t = self.clock()
                    self._error = None
            except Exception as exc:  # optional sensor must never affect driving
                with self._lock:
                    self._voltage_v = None
                    self._sample_t = None
                    self._error = f"{type(exc).__name__}: {exc}"
                if self._bus is not None:
                    try:
                        self._bus.close()
                    except Exception:
                        pass
                    self._bus = None
            self._stop.wait(self.interval_s)
