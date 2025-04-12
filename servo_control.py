# TODO: add pid control to this script to make it more accurate and smooth

import RPi.GPIO as GPIO  # Changed for to work for Raspberry Pi
import time
import math

# Servo configuration
FREQ = 50       # PWM frequency in Hz (50Hz is standard for most servos)
BASE_PIN = 13
SHOULDER_PIN = 23
ELBOW_PIN = 12
GRIPPER_PIN = 5

# Servo pulse width settings (in microseconds)
MIN_PULSE = 500   # 0 degrees
MAX_PULSE = 2500  # 180 degrees
MID_PULSE = 1500  # 90 degrees (neutral position)
GRIPPER_OPEN = 500 # Fully open
GRIPPER_CLOSE = 1500 # Fully closed

# Arm link lengths (in centimeters)
L1 = 10.0  # Base to shoulder (vertical height)
L2 = 12.0  # Shoulder to elbow
L3 = 10.0  # Elbow to gripper tip

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


def setup_servo(pin):   
    # Set up the pin as PWM output
    GPIO.setup(pin, GPIO.OUT)
   
    # Initialize PWM
    pwm = GPIO.PWM(pin, FREQ)
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

def angle_to_pulse(angle):
    # Convert angle (0-180 degrees) to pulse width (500-2500 us)
    return MIN_PULSE + (angle / 180.0) * (MAX_PULSE - MIN_PULSE)

def inverse_kinematics(x, y, z):
    """
    Calculate joint angles for the Armstrong 3 DOF arm to reach (x, y, z).
    Returns (theta1, theta2, theta3) in degrees for base, shoulder, elbow.
    Assumes L1 (base height), L2 (shoulder to elbow), L3 (elbow to gripper).
    Returns None if position is unreachable.
    """
    try:
        # Adjust z for base height (L1 is vertical)
        z_effective = z - L1
        
        # Base angle (theta1) from x, y projection
        theta1 = math.atan2(y, x)  # Angle in radians
        theta1_deg = math.degrees(theta1)
        if theta1_deg < 0 or theta1_deg > 180:
            print("Base angle out of range")
            return None
        
        # Distance in xy-plane from base to target
        r = math.sqrt(x**2 + y**2)
        
        # 2D IK for shoulder (theta2) and elbow (theta3) in the r-z plane
        # Shoulder joint is at (0, 0, L1), target at (r, 0, z_effective)
        d = math.sqrt(r**2 + z_effective**2)  # Distance from shoulder to target
        
        # Check reachability
        if d > (L2 + L3) or d < abs(L2 - L3):
            print("Position unreachable: distance out of range")
            return None
        
        # Cosine law for elbow angle (theta3)
        cos_theta3 = (L2**2 + L3**2 - d**2) / (2 * L2 * L3)
        cos_theta3 = max(min(cos_theta3, 1.0), -1.0)  # Clamp to [-1, 1]
        theta3 = math.acos(cos_theta3)  # Radians
        theta3_deg = math.degrees(theta3)
        
        # Cosine law for shoulder angle (theta2)
        cos_alpha = (L2**2 + d**2 - L3**2) / (2 * L2 * d)
        cos_alpha = max(min(cos_alpha, 1.0), -1.0)
        alpha = math.acos(cos_alpha)
        
        # Angle of target relative to shoulder
        beta = math.atan2(z_effective, r)
        theta2 = beta + alpha  # Shoulder angle in radians
        theta2_deg = math.degrees(theta2)
        
        # Adjust angles to servo range (0-180)
        # For Armstrong, assume:
        # - theta2: 0° is horizontal forward, positive up
        # - theta3: positive is bending elbow inward
        # Servo-specific adjustments (may need tuning)
        theta2_deg = 90 - theta2_deg  # Adjust so 0° is vertical up
        theta3_deg = 180 - theta3_deg  # Invert elbow for natural motion
        
        if not (0 <= theta2_deg <= 180 and 0 <= theta3_deg <= 180):
            print("Shoulder or elbow angle out of range")
            return None
        
        return (theta1_deg, theta2_deg, theta3_deg)
    
    except Exception as e:
        print(f"IK calculation error: {e}")
        return None

def main():
    try:
        # Set up GPIO
        GPIO.setmode(GPIO.BOARD)
        
        # Initialize servos
        base_pwm = setup_servo(BASE_PIN)
        shoulder_pwm = setup_servo(SHOULDER_PIN)
        elbow_pwm = setup_servo(ELBOW_PIN)
        gripper_pwm = setup_servo(GRIPPER_PIN)
        
        # Initialize PID controllers
        base_pid = PIDController(KP, KI, KD)
        shoulder_pid = PIDController(KP, KI, KD)
        elbow_pid = PIDController(KP, KI, KD)
        
        # Example: Move to (x, y, z) = (15, 5, 15) cm
        target_x, target_y, target_z = 10.0, 5.0, 5.0
        
        # Calculate IK
        angles = inverse_kinematics(target_x, target_y, target_z)
        if angles is None:
            print("Cannot reach target position")
            return
        
        theta1, theta2, theta3 = angles
        print(f"Target angles: Base={theta1:.2f}°, Shoulder={theta2:.2f}°, Elbow={theta3:.2f}°")
        
        # Move servos to calculated angles
        print("Moving to target position")
        move_servo_smooth(base_pwm, base_pid, angle_to_pulse(theta1), duration=1.0)
        move_servo_smooth(shoulder_pwm, shoulder_pid, angle_to_pulse(theta2), duration=1.0)
        move_servo_smooth(elbow_pwm, elbow_pid, angle_to_pulse(theta3), duration=1.0)
        
        # Open and close gripper
        print("Opening gripper")
        set_pulse(gripper_pwm, GRIPPER_OPEN)
        time.sleep(1)
        
        print("Closing gripper")
        set_pulse(gripper_pwm, GRIPPER_CLOSED)
        time.sleep(1)
        
        # Return to initial position (0 degrees)
        print("Returning to initial position")
        move_servo_smooth(base_pwm, base_pid, MIN_PULSE, duration=1.0)
        move_servo_smooth(shoulder_pwm, shoulder_pid, MIN_PULSE, duration=1.0)
        move_servo_smooth(elbow_pwm, elbow_pid, MIN_PULSE, duration=1.0)
        
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
        
    finally:
        base_pwm.stop()
        shoulder_pwm.stop()
        elbow_pwm.stop()
        gripper_pwm.stop()
        GPIO.cleanup()
        print("GPIO cleanup completed")

if __name__ == "__main__":
    main()