import RPi.GPIO as GPIO
import time

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Define servo pin
SERVO_PIN = 18  # GPIO18

# Setup servo pin
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Create PWM instance (50Hz for HS-55)
pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(0)

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, setpoint, current):
        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 1e-6  # Avoid division by zero

        # Calculate error
        error = setpoint - current

        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / dt
        D = self.Kd * derivative

        # Compute output
        output = P + I + D

        # Update state
        self.prev_error = error
        self.last_time = current_time

        return output

def set_angle(angle, pid, current_angle, duration=0.5):
    """Move servo to target angle using PID control."""
    start_time = time.time()
    while time.time() - start_time < duration:
        # Compute PID output (angle adjustment)
        pid_output = pid.compute(angle, current_angle)

        # Update estimated current angle (simulate servo movement)
        current_angle += pid_output * 0.01  # Adjust based on PID output
        current_angle = max(0, min(180, current_angle))  # Clamp to valid range

        # Convert angle to duty cycle (HS-55: 2.5% at 0°, 12.5% at 180°)
        duty = 2.5 + (current_angle / 18.0)
        pwm.ChangeDutyCycle(duty)

        time.sleep(0.01)  # Short delay for smooth control

    # Stop PWM signal to prevent jitter
    pwm.ChangeDutyCycle(0)
    return current_angle

try:
    # Initialize PID controller (tuned for smooth servo control)
    pid = PIDController(Kp=0.5, Ki=0.1, Kd=0.05)

    # Initialize estimated current angle
    current_angle = 0

    while True:
        # 0 to 90 degrees clockwise
        current_angle = set_angle(90, pid, current_angle)
        time.sleep(0.5)

        # Back to 0
        current_angle = set_angle(0, pid, current_angle)
        time.sleep(0.5)

        # 0 to 90 degrees counter-clockwise
        current_angle = set_angle(90, pid, current_angle)
        time.sleep(0.5)

        # Back to 0
        current_angle = set_angle(0, pid, current_angle)
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nProgram interrupted by user")

finally:
    # Cleanup
    pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up")