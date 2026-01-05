#!/usr/bin/env python3
"""
PID Auto-Tuner for Heating System - Ziegler-Nichols Method
Uses relay oscillation to characterize system and calculate optimal PID values
THERMISTOR: Vishay NTCALUG01A103J (10kΩ NTC, β=3984K)
REFERENCE RESISTOR: 2.2kΩ (voltage divider - INVERTED)
MCP3204 SUPPLY: 3.3V
ADC REFERENCE: 3.3V
RELAY: GPIO4 (Raspberry Pi Compute Module)

CIRCUIT (INVERTED): 3.3V ─── Rt ─── CH0 ─── 2.2kΩ ─── GND
"""

import time
import sys
import math
import spidev
import RPi.GPIO as GPIO
from datetime import datetime
import os

# ============================================================================
# THERMISTOR & CIRCUIT CONFIGURATION
# ============================================================================

SUPPLY_VOLTAGE = 3.3
ADC_REFERENCE_VOLTAGE = 3.3
THERMISTOR_BETA = 3984
THERMISTOR_REFERENCE_RESISTANCE = 11450
THERMISTOR_REFERENCE_TEMP = 21.60
VOLTAGE_DIVIDER_RESISTOR = 2200

# ============================================================================
# VOLTAGE DIVIDER CONFIGURATION
# ============================================================================
VOLTAGE_DIVIDER_INVERTED = True

# ============================================================================
# GPIO & RELAY CONFIGURATION
# ============================================================================
RELAY_GPIO_PIN = 25
RELAY_ACTIVE_HIGH = True

# ============================================================================
# AUTO-TUNING CONFIGURATION
# ============================================================================
TUNING_DURATION = 600  # Max tuning time in seconds (10 minutes)
MIN_OSCILLATIONS = 3  # Minimum number of complete cycles to collect
TEMPERATURE_TOLERANCE = 0.3  # ±0.3°C around target
MAX_TEMPERATURE = 140.0
MIN_TEMPERATURE = -10.0
DEBUG_MODE = False

class ZieglerNicholsTuner:
    """Ziegler-Nichols relay auto-tuning"""
    
    def __init__(self, target_temp):
        self.target_temp = target_temp
        self.relay_on = False
        self.last_switch_time = time.time()
        self.cycle_times = []  # Period of oscillation
        self.temp_max = target_temp
        self.temp_min = target_temp
        self.temp_samples = []
        self.time_samples = []
        self.relay_state_changes = []  # List of (time, state, temp) tuples
        self.start_time = time.time()
        
    def get_relay_state(self, current_temp):
        """
        Relay hysteresis control
        Returns: True = ON, False = OFF
        """
        error = self.target_temp - current_temp
        
        if self.relay_on:
            # Currently ON - turn OFF if above target
            if current_temp >= (self.target_temp + TEMPERATURE_TOLERANCE):
                self.relay_on = False
                self.relay_state_changes.append((time.time(), False, current_temp))
                return False
        else:
            # Currently OFF - turn ON if below target
            if current_temp < self.target_temp:
                self.relay_on = True
                self.relay_state_changes.append((time.time(), True, current_temp))
                return True
        
        return self.relay_on
    
    def record_measurement(self, current_temp):
        """Record temperature sample"""
        self.temp_samples.append(current_temp)
        self.time_samples.append(time.time())
        self.temp_max = max(self.temp_max, current_temp)
        self.temp_min = min(self.temp_min, current_temp)
    
    def is_oscillating(self):
        """Check if system is oscillating steadily"""
        if len(self.relay_state_changes) < (MIN_OSCILLATIONS * 2):
            return False
        return True
    
    def calculate_pid(self):
        """
        Calculate PID parameters using Ziegler-Nichols method
        Returns: (kp, ki, kd) tuple
        """
        if len(self.relay_state_changes) < 4:
            return None
        
        # Calculate ultimate period from oscillations
        # Find pairs of relay switches (ON to OFF, OFF to ON)
        periods = []
        
        for i in range(1, len(self.relay_state_changes) - 1, 2):
            if self.relay_state_changes[i][1] == False and self.relay_state_changes[i+1][1] == True:
                # Found ON->OFF->ON cycle
                period = self.relay_state_changes[i+1][0] - self.relay_state_changes[i-1][0]
                periods.append(period)
        
        if not periods:
            return None
        
        # Use average period
        tu = sum(periods) / len(periods)  # Ultimate period
        
        # Calculate ultimate gain (Ku) from temperature oscillation
        if len(self.temp_samples) > 4:
            temp_range = self.temp_max - self.temp_min
            if temp_range > 0:
                # Approximation: Ku relates to system gain
                # Using hysteresis band as oscillation amplitude reference
                amplitude = temp_range / 2.0
                ku = (2 * TEMPERATURE_TOLERANCE) / amplitude if amplitude > 0 else 0.5
            else:
                ku = 1.0
        else:
            ku = 1.0
        
        # Ziegler-Nichols tuning rules for relay response
        # Standard form (slightly aggressive for heating systems)
        kp = 0.6 * ku
        ki = 1.2 * ku / tu
        kd = 3.0 * ku * tu / 40.0
        
        return (kp, ki, kd, tu, ku)
    
    def get_metrics(self):
        """Return tuning metrics"""
        metrics = {
            'oscillations': len(self.relay_state_changes) // 2,
            'temp_min': self.temp_min,
            'temp_max': self.temp_max,
            'temp_range': self.temp_max - self.temp_min,
            'samples': len(self.temp_samples),
            'elapsed': time.time() - self.start_time
        }
        return metrics

def calculate_temperature_from_voltage(voltage):
    """Calculate temperature from ADC voltage"""
    
    if voltage < 0 or voltage >= SUPPLY_VOLTAGE:
        return None
    
    if voltage == 0:
        return None
    
    if VOLTAGE_DIVIDER_INVERTED:
        resistance = VOLTAGE_DIVIDER_RESISTOR * (SUPPLY_VOLTAGE - voltage) / voltage
    else:
        resistance = VOLTAGE_DIVIDER_RESISTOR * voltage / (SUPPLY_VOLTAGE - voltage)
    
    if resistance <= 0:
        return None
    
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
    """Read MCP3204 channel"""
    try:
        cmd = [0x06 | (channel >> 2), (channel << 6) & 0xC0, 0x00]
        response = spi.xfer2(cmd)
        
        raw_value = ((response[1] & 0x0F) << 8) | response[2]
        voltage = (raw_value / 4095.0) * ADC_REFERENCE_VOLTAGE
        
        temp_data = None
        if channel == 0:
            temp_data = calculate_temperature_from_voltage(voltage)
        
        return raw_value, voltage, temp_data
    
    except Exception as e:
        print(f"✗ Error reading channel {channel}: {e}")
        return None, None, None

def setup_gpio():
    """Initialize GPIO"""
    try:
        try:
            GPIO.cleanup()
            time.sleep(0.2)
        except:
            pass
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RELAY_GPIO_PIN, GPIO.OUT, initial=GPIO.LOW)
        
        print(f"✓ GPIO{RELAY_GPIO_PIN} initialized")
    except Exception as e:
        print(f"✗ GPIO initialization failed: {e}")
        print(f"✗ Make sure you are running with sudo: sudo python3 script.py")
        sys.exit(1)

def set_relay(state):
    """Control relay"""
    try:
        if RELAY_ACTIVE_HIGH:
            gpio_state = GPIO.HIGH if state else GPIO.LOW
        else:
            gpio_state = GPIO.LOW if state else GPIO.HIGH
        
        GPIO.output(RELAY_GPIO_PIN, gpio_state)
    except Exception as e:
        print(f"✗ Error setting relay: {e}")

def cleanup_gpio():
    """Cleanup GPIO"""
    try:
        GPIO.output(RELAY_GPIO_PIN, GPIO.LOW)
        GPIO.cleanup()
    except:
        pass

def save_tuning_results(target_temp, tuner, kp, ki, kd, tu, ku):
    """Save tuning results to file"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"pid_tune_{target_temp}C_{timestamp}.txt"
    
    try:
        with open(filename, 'w') as f:
            f.write("="*80 + "\n")
            f.write("PID AUTO-TUNING RESULTS (Ziegler-Nichols Method)\n")
            f.write("="*80 + "\n\n")
            
            f.write(f"Target Temperature: {target_temp}°C\n")
            f.write(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            metrics = tuner.get_metrics()
            f.write("TUNING METRICS:\n")
            f.write(f"  Oscillations: {metrics['oscillations']}\n")
            f.write(f"  Temperature Range: {metrics['temp_min']:.2f}°C - {metrics['temp_max']:.2f}°C (ΔT={metrics['temp_range']:.2f}°C)\n")
            f.write(f"  Samples Collected: {metrics['samples']}\n")
            f.write(f"  Tuning Duration: {metrics['elapsed']:.1f}s\n\n")
            
            f.write("CALCULATED PARAMETERS:\n")
            f.write(f"  Ultimate Period (Tu): {tu:.2f} seconds\n")
            f.write(f"  Ultimate Gain (Ku): {ku:.3f}\n\n")
            
            f.write("PID COEFFICIENTS:\n")
            f.write(f"  Kp (Proportional): {kp:.4f}\n")
            f.write(f"  Ki (Integral):     {ki:.4f}\n")
            f.write(f"  Kd (Derivative):   {kd:.4f}\n\n")
            
            f.write("HOW TO USE:\n")
            f.write("  1. Update heating_control_tuning.py with these values:\n")
            f.write(f"     PID_KP = {kp:.4f}\n")
            f.write(f"     PID_KI = {ki:.4f}\n")
            f.write(f"     PID_KD = {kd:.4f}\n\n")
            
            f.write("  2. Run heating controller:\n")
            f.write(f"     sudo python3 heating_control_tuning.py {target_temp}\n\n")
            
            f.write("STATE CHANGES LOG:\n")
            f.write("  Time(s)  | Relay | Temp(°C) | Note\n")
            f.write("  " + "-"*50 + "\n")
            
            for i, (t, state, temp) in enumerate(tuner.relay_state_changes):
                elapsed = t - tuner.relay_state_changes[0][0]
                state_str = "ON " if state else "OFF"
                note = "→ ON" if (i == 0 or not tuner.relay_state_changes[i-1][1]) and state else ""
                note = "→ OFF" if (i > 0 and tuner.relay_state_changes[i-1][1]) and not state else note
                f.write(f"  {elapsed:7.1f}  | {state_str} | {temp:7.2f}  | {note}\n")
        
        print(f"\n✓ Results saved to: {filename}")
        return filename
    except Exception as e:
        print(f"✗ Error saving results: {e}")
        return None

def main():
    print("\n" + "="*80)
    print("PID AUTO-TUNER - Ziegler-Nichols Relay Oscillation Method")
    print("="*80)
    print(f"Relay GPIO: {RELAY_GPIO_PIN} (RPi Compute Module)")
    print(f"Configuration: {'INVERTED' if VOLTAGE_DIVIDER_INVERTED else 'NORMAL'}")
    print(f"Max Tuning Duration: {TUNING_DURATION}s")
    print(f"Min Oscillations: {MIN_OSCILLATIONS}")
    print("="*80 + "\n")
    
    # Check sudo
    if os.geteuid() != 0:
        print("⚠ WARNING: This script should be run with sudo!")
        print("Usage: sudo python3 pid_autotuner.py\n")
    
    # Get target temperature
    target_temp = None
    if len(sys.argv) > 1:
        try:
            target_temp = float(sys.argv[1])
        except ValueError:
            print("✗ Invalid target temperature. Usage: sudo python3 pid_autotuner.py <target_temp_celsius>")
            sys.exit(1)
    else:
        print("Enter target temperature for tuning")
        try:
            target_temp = float(input("Target Temperature (°C): "))
        except ValueError:
            print("✗ Invalid input")
            sys.exit(1)
    
    # Validate
    if target_temp < 0 or target_temp > MAX_TEMPERATURE:
        print(f"✗ Target must be between 0°C and {MAX_TEMPERATURE}°C")
        sys.exit(1)
    
    print(f"\n⚠ TUNING WILL BEGIN IN 3 SECONDS")
    print(f"⚠ Relay will oscillate around {target_temp}°C to characterize system")
    print(f"⚠ Press Ctrl+C to abort\n")
    
    time.sleep(3)
    
    # Setup
    setup_gpio()
    tuner = ZieglerNicholsTuner(target_temp)
    
    try:
        # Open SPI
        spi = spidev.SpiDev()
        spi.open(0, 0)
        spi.max_speed_hz = 1000000
        spi.mode = 0
        
        print(f"✓ SPI initialized")
        time.sleep(0.5)
        
        # Print header
        print(f"{'Time':<10} {'Temp(°C)':<12} {'Target':<12} {'Relay':<10} {'Voltage':<10} {'Status':<20}")
        print("-" * 90)
        
        iteration = 0
        start_time = time.time()
        
        while True:
            iteration += 1
            elapsed = time.time() - start_time
            timestamp = datetime.now().strftime("%H:%M:%S")
            
            # Read temperature
            ch0_raw, ch0_volt, ch0_temp = read_mcp3204_channel(spi, 0)
            
            if ch0_temp is None:
                print(f"{timestamp:<10} {'ERROR':<12} {target_temp:<12.2f} {'OFF':<10} {ch0_volt:<10} {'Sensor Error':<20}")
                set_relay(False)
                time.sleep(1.0)
                continue
            
            current_temp = ch0_temp['temperature_c']
            
            # Safety check
            if current_temp > MAX_TEMPERATURE:
                print(f"{timestamp:<10} {current_temp:<12.2f} {target_temp:<12.2f} {'OFF':<10} {ch0_volt:<10.4f} {'OVERHEAT - ABORT':<20}")
                set_relay(False)
                print("\n✗ TUNING ABORTED: Temperature exceeded safety limit!")
                break
            
            if current_temp < MIN_TEMPERATURE:
                print(f"{timestamp:<10} {current_temp:<12.2f} {target_temp:<12.2f} {'OFF':<10} {ch0_volt:<10.4f} {'Sensor Error':<20}")
                set_relay(False)
                time.sleep(1.0)
                continue
            
            # Get relay state from tuner
            relay_on = tuner.get_relay_state(current_temp)
            set_relay(relay_on)
            tuner.record_measurement(current_temp)
            
            # Status
            relay_str = "ON " if relay_on else "OFF"
            metrics = tuner.get_metrics()
            
            if tuner.is_oscillating():
                status = f"⚡ Oscillating ({metrics['oscillations']} cycles)"
            else:
                status = f"◐ Settling... ({metrics['oscillations']} cycles)"
            
            print(f"{timestamp:<10} {current_temp:<12.2f} {target_temp:<12.2f} {relay_str:<10} {ch0_volt:<10.4f} {status:<20}")
            
            # Check completion criteria
            if elapsed > TUNING_DURATION:
                print(f"\n✓ TUNING COMPLETE: Max duration reached ({elapsed:.1f}s)")
                break
            
            if tuner.is_oscillating() and elapsed > 60:
                print(f"\n✓ TUNING COMPLETE: Steady oscillation achieved ({metrics['oscillations']} cycles, {elapsed:.1f}s)")
                break
            
            time.sleep(1.0)
        
        # Turn off relay
        set_relay(False)
        
        # Calculate PID
        result = tuner.calculate_pid()
        
        if result is None:
            print("\n✗ Could not calculate PID - insufficient oscillation data")
            print("Try:")
            print("  1. Lower target temperature (system needs more time to heat/cool)")
            print("  2. Adjust TEMPERATURE_TOLERANCE in script")
            print("  3. Check thermistor and relay connections")
        else:
            kp, ki, kd, tu, ku = result
            
            print("\n" + "="*80)
            print("PID TUNING RESULTS")
            print("="*80)
            print(f"\nUltimate Period (Tu): {tu:.2f} seconds")
            print(f"Ultimate Gain (Ku): {ku:.4f}\n")
            print(f"Recommended PID Values:")
            print(f"  Kp = {kp:.4f}")
            print(f"  Ki = {ki:.4f}")
            print(f"  Kd = {kd:.4f}\n")
            
            # Save results
            save_tuning_results(target_temp, tuner, kp, ki, kd, tu, ku)
            
            print("\n✓ Ready to use in heating_control_tuning.py")
    
    except KeyboardInterrupt:
        print("\n\n✗ TUNING ABORTED by user")
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
