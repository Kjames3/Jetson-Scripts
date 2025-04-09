# TODO: add pid control to this script to make it more accurate and smooth

import Jetson.GPIO as GPIO
import time

# Servo configuration
SERVO_PIN = 33  # Using Jetson Nano GPIO pin 33 (PWM capable)
FREQ = 50       # PWM frequency in Hz (50Hz is standard for most servos)

# Servo pulse width settings (in microseconds)
MIN_PULSE = 500   # 0 degrees
MAX_PULSE = 2500  # 180 degrees
MID_PULSE = 1500  # 90 degrees (neutral position)

# PID constants
KP = 0.1  # Proportional gain
KI = 0.01 # Integral gain
KD = 0.05 # Derivative gain

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, setpoint, current):
        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Calculate error
        error = setpoint - current
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        d_term = self.kd * derivative
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Update previous values
        self.prev_error = error
        self.last_time = current_time
        
        return output


def setup_servo():
    # Set GPIO numbering mode to BOARD
    GPIO.setmode(GPIO.BOARD)
   
    # Set up the pin as PWM output
    GPIO.setup(SERVO_PIN, GPIO.OUT)
   
    # Initialize PWM
    pwm = GPIO.PWM(SERVO_PIN, FREQ)
    pwm.start(0)  # Start with 0% duty cycle
   
    return pwm

def set_pulse(pwm, pulse_width):
    # Clamp pulse width to valid range
    pulse_width = max(MIN_PULSE, min(MAX_PULSE, pulse_width))
    duty_cycle = (pulse_width / 20000) * 100
    pwm.ChangeDutyCycle(duty_cycle)
    return pulse_width

def move_servo_smooth(pwm, pid, target_pulse, duration=1.0):
    start_time = time.time()
    current_pulse = MIN_PULSE  # Starting position
    
    while time.time() - start_time < duration:
        # Calculate PID output
        pid_output = pid.compute(target_pulse, current_pulse)
        
        # Update current pulse
        current_pulse += pid_output
        current_pulse = set_pulse(pwm, current_pulse)
        
        # Small delay for smooth movement
        time.sleep(0.01)
    
    # Ensure final position
    set_pulse(pwm, target_pulse)

def set_angle(pwm, pulse_width):
    # Convert pulse width to duty cycle
    # Duty cycle = (pulse_width / (1/frequency)) * 100
    duty_cycle = (pulse_width / 20000) * 100  # 20000us = period at 50Hz
    pwm.ChangeDutyCycle(duty_cycle)

def main():
    try:
        # Set up servo and PID
        pwm = setup_servo()
        pid = PIDController(KP, KI, KD)
        
        # Initial position
        print("Moving to starting position (0 degrees)")
        set_pulse(pwm, MIN_PULSE)
        time.sleep(1)
        
        # Move to 90 degrees smoothly
        print("Moving to 90 degrees")
        move_servo_smooth(pwm, pid, MID_PULSE, duration=1.0)
        time.sleep(0.5)
        
        # Return to 0 degrees smoothly
        print("Returning to 0 degrees")
        move_servo_smooth(pwm, pid, MIN_PULSE, duration=1.0)
        
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
        
    finally:
        pwm.stop()
        GPIO.cleanup()
        print("GPIO cleanup completed")

if __name__ == "__main__":
    main()