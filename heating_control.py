#!/usr/bin/env python3

"""
Heating Control System - Uses MCP3204 thermistor to control relay heating

THERMISTOR: Vishay NTCALUG01A103J (10kΩ NTC, β=3984K)
REFERENCE RESISTOR: 10kΩ (voltage divider - INVERTED)
MCP3204 SUPPLY: 3.3V
ADC REFERENCE: 3.3V
RELAY: GPIO25 (Raspberry Pi Compute Module)
CIRCUIT (INVERTED): 3.3V ─── Rt ─── CH0 ─── 10kΩ ─── GND
"""

import time
import sys
import math
import spidev
import RPi.GPIO as GPIO
from datetime import datetime
import os
import json  # ADDED: For writing thermistor data

# ============================================================================
# THERMISTOR & CIRCUIT CONFIGURATION
# ============================================================================

SUPPLY_VOLTAGE = 3.3  # MCP3204 supply voltage (3.3V)
ADC_REFERENCE_VOLTAGE = 3.3  # MCP3204 uses 3.3V as reference for 0-4095 range
THERMISTOR_BETA = 3984  # Vishay NTCALUG01A103J beta coefficient (Kelvin)
THERMISTOR_REFERENCE_RESISTANCE = 11450  # 10kΩ at 25°C (thermistor rated value)
THERMISTOR_REFERENCE_TEMP = 21.60  # Reference temperature in Celsius
VOLTAGE_DIVIDER_RESISTOR = 10000  # 2.2kΩ reference resistor

# ============================================================================
# VOLTAGE DIVIDER CONFIGURATION
# ============================================================================

# Set to True for INVERTED configuration (3.3V ─── Rt ─── CH0 ─── R_fixed ─── GND)
# Set to False for NORMAL configuration (3.3V ─── R_fixed ─── CH0 ─── Rt ─── GND)
VOLTAGE_DIVIDER_INVERTED = False

# ============================================================================
# GPIO & RELAY CONFIGURATION
# ============================================================================

RELAY_GPIO_PIN = 26  # GPIO26 on Raspberry Pi
RELAY_ACTIVE_HIGH = True  # Set to False if relay activates on LOW
USE_GPIOD = False  # Set to True to use libgpiod instead of RPi.GPIO (newer method)

# ============================================================================
# PID CONTROL CONFIGURATION
# ============================================================================

PID_SAMPLE_TIME = 1.0  # Sample time in seconds
PID_KP = 0.1106  # Proportional gain
PID_KI = 0.021  # Integral gain
PID_KD = 0.2768  # Derivative gain
PID_OUTPUT_MIN = 0  # Min output (relay off)
PID_OUTPUT_MAX = 1  # Max output (relay on)
TEMPERATURE_DEADBAND = 0.5  # Hysteresis in °C (turn off at target + deadband)

# ============================================================================
# STARTUP & SAFETY CONFIGURATION
# ============================================================================

MAX_TEMPERATURE = 150.0  # Safety cutoff temperature in °C
MIN_TEMPERATURE = -10.0  # Sensor error detection threshold in °C
STARTUP_DELAY = 0.5  # Delay in seconds before starting relay checks
SAMPLE_RATE = 1  # Readings per second
DEBUG_MODE = True  # Enable detailed GPIO debug output

# ============================================================================
# JSON OUTPUT CONFIGURATION
# ============================================================================

JSON_OUTPUT_FILE = "/tmp/heater_thermistor.json"  # File to write heater temp


class TemperatureController:
    """PID-based temperature controller for heating system"""

    def __init__(self, target_temp, kp=PID_KP, ki=PID_KI, kd=PID_KD,
                 sample_time=PID_SAMPLE_TIME, deadband=TEMPERATURE_DEADBAND):
        self.target_temp = target_temp
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.sample_time = sample_time
        self.deadband = deadband

        # PID state
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.heating = False

    def update(self, current_temp):
        """
        Calculate relay output based on current temperature
        Args:
            current_temp: Current temperature in °C
        Returns:
            bool: True = relay ON (heating), False = relay OFF
        """
        now = time.time()
        dt = now - self.last_time

        # Calculate error
        error = self.target_temp - current_temp

        # Proportional term
        p_term = self.kp * error

        # Integral term (with anti-windup)
        self.integral += error * dt
        self.integral = max(-10, min(10, self.integral))  # Clamp integral
        i_term = self.ki * self.integral

        # Derivative term
        if dt > 0:
            d_term = self.kd * (error - self.last_error) / dt
        else:
            d_term = 0

        # Hysteresis control
        if self.heating:
            # Currently heating - turn off if target + deadband reached
            if current_temp >= (self.target_temp + self.deadband):
                self.heating = False
        else:
            # Currently off - turn on if below target
            if current_temp < self.target_temp:
                self.heating = True

        self.last_error = error
        self.last_time = now

        return self.heating

    def set_target(self, target_temp):
        """Set new target temperature"""
        self.target_temp = target_temp
        self.integral = 0  # Reset integral on setpoint change


def calculate_temperature_from_voltage(voltage):
    """
    Calculate temperature from measured voltage using voltage divider equation.
    Args:
        voltage: Measured voltage from ADC (0-3.3V)
    Returns:
        dict with 'resistance' and 'temperature_c', or None if invalid
    """
    # Validate voltage range
    if voltage < 0 or voltage >= SUPPLY_VOLTAGE:
        return None

    # Avoid division by zero
    if voltage == 0:
        return None

    # Calculate thermistor resistance based on configuration
    if VOLTAGE_DIVIDER_INVERTED:
        # INVERTED: Rt = R_fixed * (Vin - Vout) / Vout
        resistance = VOLTAGE_DIVIDER_RESISTOR * (SUPPLY_VOLTAGE - voltage) / voltage
    else:
        # NORMAL: Rt = R_fixed * Vout / (Vin - Vout)
        resistance = VOLTAGE_DIVIDER_RESISTOR * voltage / (SUPPLY_VOLTAGE - voltage)

    if resistance <= 0:
        return None

    # Use Steinhart-Hart simplified form (beta equation)
    # 1/T = 1/T0 + (1/beta) * ln(R/R0)
    T_ref_kelvin = THERMISTOR_REFERENCE_TEMP + 273.15
    temp_kelvin = 1.0 / (
        1.0 / T_ref_kelvin +
        (1.0 / THERMISTOR_BETA) * math.log(resistance / THERMISTOR_REFERENCE_RESISTANCE)
    )
    temp_celsius = temp_kelvin - 273.15

    return {
        'resistance': resistance,
        'temperature_c': temp_celsius
    }


def read_mcp3204_channel(spi, channel):
    """
    Read a single channel from MCP3204
    Args:
        spi: SPI device object
        channel: Channel number (0-3)
    Returns:
        tuple: (raw_value, voltage, temperature_data or None)
    """
    try:
        # MCP3204 Read Channel (Single-ended)
        cmd = [0x06 | (channel >> 2), (channel << 6) & 0xC0, 0x00]
        response = spi.xfer2(cmd)

        # Extract 12-bit value
        raw_value = ((response[1] & 0x0F) << 8) | response[2]

        # Convert to voltage (3.3V reference)
        voltage = (raw_value / 4095.0) * ADC_REFERENCE_VOLTAGE

        # Calculate temperature only for channel 0 (with thermistor)
        temp_data = None
        if channel == 0:
            temp_data = calculate_temperature_from_voltage(voltage)

        return raw_value, voltage, temp_data

    except Exception as e:
        print(f"✗ Error reading channel {channel}: {e}")
        return None, None, None


def setup_gpio():
    """Initialize GPIO for relay control"""
    try:
        # Reset GPIO to clean state
        if DEBUG_MODE:
            print(f"[GPIO DEBUG] Attempting GPIO setup for pin {RELAY_GPIO_PIN}...")

        # Try to cleanup any previous state
        try:
            GPIO.cleanup()
            time.sleep(0.2)
        except:
            pass

        # Set mode
        GPIO.setmode(GPIO.BCM)
        if DEBUG_MODE:
            print(f"[GPIO DEBUG] GPIO mode set to BCM")

        # Setup pin as output
        GPIO.setup(RELAY_GPIO_PIN, GPIO.OUT, initial=GPIO.LOW)
        if DEBUG_MODE:
            print(f"[GPIO DEBUG] GPIO{RELAY_GPIO_PIN} setup as OUTPUT (initial LOW)")

        # Verify setup
        state = GPIO.input(RELAY_GPIO_PIN)
        if DEBUG_MODE:
            print(f"[GPIO DEBUG] GPIO{RELAY_GPIO_PIN} read back as: {state} (should be 0)")

        print(f"✓ GPIO{RELAY_GPIO_PIN} initialized (Relay control)")

    except Exception as e:
        print(f"✗ GPIO initialization failed: {e}")
        print(f"✗ Make sure you are running with sudo: sudo python3 script.py <target_temp>")
        sys.exit(1)


def set_relay(state):
    """
    Control relay state with debug output
    Args:
        state: True = ON (heating), False = OFF
    """
    try:
        if RELAY_ACTIVE_HIGH:
            gpio_state = GPIO.HIGH if state else GPIO.LOW
        else:
            gpio_state = GPIO.LOW if state else GPIO.HIGH

        GPIO.output(RELAY_GPIO_PIN, gpio_state)

        if DEBUG_MODE:
            read_state = GPIO.input(RELAY_GPIO_PIN)
            print(f"[GPIO DEBUG] Set GPIO{RELAY_GPIO_PIN} to {gpio_state} (read back: {read_state})")

    except Exception as e:
        print(f"✗ Error setting relay: {e}")


def cleanup_gpio():
    """Cleanup GPIO resources"""
    try:
        GPIO.output(RELAY_GPIO_PIN, GPIO.LOW)
        GPIO.cleanup()
    except:
        pass


def write_heater_json(temperature_c):
    """
    Write heater thermistor temperature to JSON file
    Flask reads this file to include heater data in logs
    """
    try:
        data = {
            "temperature_c": temperature_c,
            "timestamp": time.time()
        }
        with open(JSON_OUTPUT_FILE, 'w') as f:
            json.dump(data, f)
    except Exception as e:
        print(f"✗ Error writing JSON: {e}")


def main():
    print("\n" + "="*80)
    print("Heating Control System - Temperature Control via Relay")
    print("="*80)
    print(f"Target Temperature: Set via command line or interactive mode")
    print(f"Relay GPIO: {RELAY_GPIO_PIN} (Raspberry Pi)")
    print(f"Relay Logic: {'ACTIVE HIGH' if RELAY_ACTIVE_HIGH else 'ACTIVE LOW'}")
    print(f"Control Method: Hysteresis (deadband: {TEMPERATURE_DEADBAND}°C)")
    print(f"Max Safe Temperature: {MAX_TEMPERATURE}°C")
    print(f"Configuration: {'INVERTED' if VOLTAGE_DIVIDER_INVERTED else 'NORMAL'}")
    print(f"Debug Mode: {DEBUG_MODE}")
    print(f"JSON Output: {JSON_OUTPUT_FILE}")
    print("="*80 + "\n")

    # Check if running as root
    if os.geteuid() != 0:
        print("⚠ WARNING: This script should be run with sudo for GPIO control!")
        print("Usage: sudo python3 script.py <target_temp>\n")

    # Get target temperature
    target_temp = None

    if len(sys.argv) > 1:
        try:
            target_temp = float(sys.argv[1])
        except ValueError:
            print("✗ Invalid target temperature. Usage: sudo python3 script.py <target_temp>")
            sys.exit(1)
    else:
        print("Interactive Mode: Enter target temperature")
        try:
            target_temp = float(input("Target Temperature (°C): "))
        except ValueError:
            print("✗ Invalid input")
            sys.exit(1)

    # Validate target temperature
    if target_temp < 0 or target_temp > MAX_TEMPERATURE:
        print(f"✗ Target temperature must be between 0°C and {MAX_TEMPERATURE}°C")
        sys.exit(1)

    # Initialize GPIO
    setup_gpio()

    # Initialize controller
    controller = TemperatureController(
        target_temp=target_temp,
        kp=PID_KP,
        ki=PID_KI,
        kd=PID_KD,
        sample_time=PID_SAMPLE_TIME,
        deadband=TEMPERATURE_DEADBAND
    )

    try:
        # Open SPI device
        spi = spidev.SpiDev()
        spi.open(0, 0)  # /dev/spidev0.0
        spi.max_speed_hz = 1000000  # 1 MHz
        spi.mode = 0

        print(f"✓ SPI initialized (1 MHz)")
        time.sleep(STARTUP_DELAY)

        # Print header
        print(f"{'Time':<10} {'Temp(°C)':<12} {'Target':<12} {'Relay':<10} {'Voltage':<10} {'Status':<15}")
        print("-" * 80)

        iteration = 0

        while True:
            iteration += 1
            timestamp = datetime.now().strftime("%H:%M:%S")

            # Read temperature from CH0
            ch0_raw, ch0_volt, ch0_temp = read_mcp3204_channel(spi, 0)

            if ch0_temp is None:
                print(f"{timestamp:<10} {'ERROR':<12} {target_temp:<12.2f} {'OFF':<10} {ch0_volt:<10} {'Sensor Error':<15}")
                set_relay(False)
                time.sleep(1.0 / SAMPLE_RATE)
                continue

            current_temp = ch0_temp['temperature_c']

            # ===== WRITE JSON FOR FLASK ===== (ADDED - THIS IS THE FIX)
            write_heater_json(current_temp)

            # Safety check
            if current_temp > MAX_TEMPERATURE:
                print(f"{timestamp:<10} {current_temp:<12.2f} {target_temp:<12.2f} {'OFF':<10} {ch0_volt:<10.4f} {'OVERHEAT - SHUTOFF':<15}")
                set_relay(False)
                break

            if current_temp < MIN_TEMPERATURE:
                print(f"{timestamp:<10} {current_temp:<12.2f} {target_temp:<12.2f} {'OFF':<10} {ch0_volt:<10.4f} {'Sensor Error':<15}")
                set_relay(False)
                time.sleep(1.0 / SAMPLE_RATE)
                continue

            # Update controller
            relay_on = controller.update(current_temp)

            # Set relay state
            set_relay(relay_on)

            # Status message
            error = target_temp - current_temp

            if abs(error) < 0.5:
                status = "✓ On target"
            elif relay_on:
                status = f"↑ Heating (+{error:.1f}°C)"
            else:
                status = f"↓ Cooling ({error:.1f}°C)"

            relay_str = "ON" if relay_on else "OFF"

            # Print status line
            print(f"{timestamp:<10} {current_temp:<12.2f} {target_temp:<12.2f} {relay_str:<10} {ch0_volt:<10.4f} {status:<15}")

            time.sleep(1.0 / SAMPLE_RATE)

    except KeyboardInterrupt:
        print("\n\n✓ Stopped by user")
        set_relay(False)

    except Exception as e:
        print(f"\n✗ FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        set_relay(False)
        sys.exit(1)

    finally:
        try:
            spi.close()
        except:
            pass

        cleanup_gpio()
        print("✓ Relay disabled and GPIO cleaned up")


if __name__ == "__main__":
    main()
