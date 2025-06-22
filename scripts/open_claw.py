#!/usr/bin/env python3
"""
Simple claw control script for opening the servo claw
"""
import RPi.GPIO as GPIO
import time
import sys

# Pin mapping for Jetson boards
output_pins = {
    'JETSON_XAVIER': 18,
    'JETSON_NANO': 33,
    'JETSON_NX': 33,
    'CLARA_AGX_XAVIER': 18,
    'JETSON_TX2_NX': 32,
    'JETSON_ORIN': 18,
    'JETSON_ORIN_NX': 33,
    'JETSON_ORIN_NANO': 33
}

# Get correct pin
output_pin = output_pins.get(GPIO.model, None)
if output_pin is None:
    print(f"ERROR: PWM not supported on this board: {GPIO.model}")
    sys.exit(1)

def move_servo(duty_cycle):
    """Move servo to specified duty cycle and then stop signal"""
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)  # Wait for servo to move
    pwm.ChangeDutyCycle(0)  # Stop signal

def open_claw():
    """Open the claw (1 = open)"""
    try:
        # Setup GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)
        global pwm
        pwm = GPIO.PWM(output_pin, 50)  # 50 Hz for servo
        pwm.start(0)
        
        print(f"Opening claw on pin {output_pin}...")
        move_servo(3.0)  # Open position (3% duty cycle)
        print("Claw opened successfully!")
        
    except Exception as e:
        print(f"ERROR opening claw: {e}")
        return False
    finally:
        # Cleanup
        try:
            pwm.stop()
            GPIO.cleanup()
        except:
            pass
    
    return True

def close_claw():
    """Close the claw (0 = close)"""
    try:
        # Setup GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)
        global pwm
        pwm = GPIO.PWM(output_pin, 50)  # 50 Hz for servo
        pwm.start(0)
        
        print(f"Closing claw on pin {output_pin}...")
        move_servo(12.0)  # Close position (12% duty cycle)
        print("Claw closed successfully!")
        
    except Exception as e:
        print(f"ERROR closing claw: {e}")
        return False
    finally:
        # Cleanup
        try:
            pwm.stop()
            GPIO.cleanup()
        except:
            pass
    
    return True

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 open_claw.py [open|close]")
        sys.exit(1)
    
    command = sys.argv[1].lower()
    
    if command == "open":
        success = open_claw()
    elif command == "close":
        success = close_claw()
    else:
        print("Invalid command. Use 'open' or 'close'")
        sys.exit(1)
    
    sys.exit(0 if success else 1)
