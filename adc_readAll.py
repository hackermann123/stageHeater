#!/usr/bin/env python3
"""
ADC Read All Channels - Reads all 4 MCP3204 channels and displays values
THERMISTOR: Vishay NTCALUG01A103J (10kΩ NTC, β=3984K)
REFERENCE RESISTOR: 2.2kΩ (voltage divider - INVERTED)
MCP3204 SUPPLY: 3.3V
ADC REFERENCE: 3.3V

CIRCUIT (INVERTED): 3.3V ─── Rt ─── CH0 ─── 2.2kΩ ─── GND
"""

import time
import sys
import math
import spidev

# ============================================================================
# THERMISTOR & CIRCUIT CONFIGURATION
# ============================================================================

SUPPLY_VOLTAGE = 3.3  # MCP3204 supply voltage (3.3V)
ADC_REFERENCE_VOLTAGE = 3.3  # MCP3204 uses 3.3V as reference for 0-4095 range
THERMISTOR_BETA = 3984  # Vishay NTCALUG01A103J beta coefficient (Kelvin)
THERMISTOR_REFERENCE_RESISTANCE = 11450  # 10kΩ at 25°C (thermistor rated value)
THERMISTOR_REFERENCE_TEMP = 21.60  # Reference temperature in Celsius
VOLTAGE_DIVIDER_RESISTOR = 2200  # 2.2kΩ reference resistor

# ============================================================================
# VOLTAGE DIVIDER CONFIGURATION
# ============================================================================
# Set to True for INVERTED configuration (3.3V ─── Rt ─── CH0 ─── R_fixed ─── GND)
# Set to False for NORMAL configuration (3.3V ─── R_fixed ─── CH0 ─── Rt ─── GND)
VOLTAGE_DIVIDER_INVERTED = True

# ============================================================================
# VOLTAGE DIVIDER CIRCUIT MATH
# ============================================================================
# 
# INVERTED Configuration (VOLTAGE_DIVIDER_INVERTED = True):
#     3.3V ─── Rt (10k NTC) ─── MCP3204_CH0
#                              │
#                         2.2kΩ resistor
#                              │
#                             GND
#
#     Vout = Vin * R_fixed / (Rt + R_fixed)
#     Rt = R_fixed * (Vin - Vout) / Vout
#
#     At 25°C (Rt=10k): Vout = 3.3 * 2200 / (10000 + 2200) = 0.59V
#     At 0°C (Rt≈25k): Vout = 3.3 * 2200 / (25000 + 2200) = 0.25V
#     At 50°C (Rt≈4k): Vout = 3.3 * 2200 / (4000 + 2200) = 1.18V
#     As temperature increases, Rt decreases, Vout increases ✓
#
# NORMAL Configuration (VOLTAGE_DIVIDER_INVERTED = False):
#     3.3V ─── 2.2kΩ resistor ─── MCP3204_CH0
#                                 │
#                            Rt (10k NTC)
#                                 │
#                                GND
#
#     Vout = Vin * Rt / (R_fixed + Rt)
#     Rt = R_fixed * Vout / (Vin - Vout)
#
#     At 25°C (Rt=10k): Vout = 3.3 * 10000 / (2200 + 10000) = 2.74V
#     At 0°C (Rt≈25k): Vout = 3.3 * 25000 / (2200 + 25000) = 3.05V
#     At 50°C (Rt≈4k): Vout = 3.3 * 4000 / (2200 + 4000) = 1.88V
#     As temperature increases, Rt decreases, Vout decreases ✓
#
# ============================================================================

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
        print(f"Error reading channel {channel}: {e}")
        return None, None, None

def main():
    config_name = "INVERTED" if VOLTAGE_DIVIDER_INVERTED else "NORMAL"
    circuit_diagram = "3.3V ─── Rt ─── CH0 ─── 2.2kΩ ─── GND" if VOLTAGE_DIVIDER_INVERTED else "3.3V ─── 2.2kΩ ─── CH0 ─── Rt ─── GND"
    
    print("\n" + "="*80)
    print(f"MCP3204 Read All Channels ({config_name} Configuration)")
    print("="*80)
    print(f"Supply Voltage: {SUPPLY_VOLTAGE}V")
    print(f"ADC Reference: {ADC_REFERENCE_VOLTAGE}V")
    print(f"Thermistor β: {THERMISTOR_BETA}K")
    print(f"Divider Resistor: {VOLTAGE_DIVIDER_RESISTOR/1000:.1f}kΩ")
    print(f"Configuration: {circuit_diagram}\n")
    
    try:
        # Open SPI device
        spi = spidev.SpiDev()
        spi.open(0, 0)  # /dev/spidev0.0
        spi.max_speed_hz = 1000000  # 1 MHz
        spi.mode = 0
        
        print(f"{'Sample':<8} {'CH0':<12} {'CH1':<12} {'CH2':<12} {'CH3':<12} {'Temp':<10}")
        print(f"{'#':<8} {'(V)':<12} {'(V)':<12} {'(V)':<12} {'(V)':<12} {'(°C)':<10}")
        print("-" * 80)
        
        iteration = 0
        while True:
            iteration += 1
            
            # Read all 4 channels
            ch0_raw, ch0_volt, ch0_temp = read_mcp3204_channel(spi, 0)
            ch1_raw, ch1_volt, _ = read_mcp3204_channel(spi, 1)
            ch2_raw, ch2_volt, _ = read_mcp3204_channel(spi, 2)
            ch3_raw, ch3_volt, _ = read_mcp3204_channel(spi, 3)
            
            # Format temperature
            temp_str = f"{ch0_temp['temperature_c']:.2f}" if ch0_temp else "N/A"
            
            # Print line
            print(f"{iteration:<8} {ch0_volt:<12.4f} {ch1_volt:<12.4f} {ch2_volt:<12.4f} {ch3_volt:<12.4f} {temp_str:<10}")
            
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\n\n✓ Stopped by user")
    except Exception as e:
        print(f"\n✗ FATAL ERROR: {e}")
        sys.exit(1)
    finally:
        try:
            spi.close()
        except:
            pass

if __name__ == "__main__":
    main()
