import RPi.GPIO as GPIO
import time

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Define the GPIO pin for the servo motor
servo_pin = 18

# Set up the servo pin as an output
GPIO.setup(servo_pin, GPIO.OUT)

# Create a PWM instance with a frequency of 50 Hz
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

def set_angle(angle):
    duty = 2.5 + (angle / 18.0)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.75)
    pwm.ChangeDutyCycle(0)

try:
    while True:
        # Move the servo to 0 degrees
        set_angle(0)
        set_angle(90)

        # Back to 0 degrees
        set_angle(0)

        # 0 to 90 degrees counter-clockwise
        set_angle(90)

        # 90 to 0 degrees counter-clockwise
        set_angle(180)

        time.sleep(1)

except KeyboardInterrupt:
    print("\n...Program terminated...")
    pwm.stop()

finally:
    GPIO.cleanup()
    print("...GPIO cleanup complete...")