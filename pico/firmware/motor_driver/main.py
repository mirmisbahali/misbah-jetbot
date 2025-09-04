from machine import UART, Pin, PWM
import time
import re

# Initialize UART for communication with Jetson Nano
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

# Initialize PWM pins for motor control
IN1 = PWM(Pin(2))  # Motor A forward
IN2 = PWM(Pin(3))  # Motor A backward  
IN3 = PWM(Pin(4))  # Motor B forward
IN4 = PWM(Pin(5))  # Motor B backward

# Set PWM frequency (1 kHz works well for motors)
for pin in (IN1, IN2, IN3, IN4):
    pin.freq(1000)

# Motor control functions
def motorA_forward(speed):
    IN1.duty_u16(speed)
    IN2.duty_u16(0)

def motorA_backward(speed):
    IN1.duty_u16(0)
    IN2.duty_u16(speed)

def motorA_stop():
    IN1.duty_u16(0)
    IN2.duty_u16(0)

def motorB_forward(speed):
    IN3.duty_u16(speed)
    IN4.duty_u16(0)

def motorB_backward(speed):
    IN3.duty_u16(0)
    IN4.duty_u16(speed)

def motorB_stop():
    IN3.duty_u16(0)
    IN4.duty_u16(0)

def stop_all_motors():
    motorA_stop()
    motorB_stop()

# Parse velocity data from UART
def parse_velocity_data(data_str):
    try:
        # Expected format: "Linear: x=0.500 y=0.000 z=0.000, Angular: x=0.000 y=0.000 z=1.200"
        linear_x = 0.0
        angular_z = 0.0
        
        # Extract linear x value
        linear_match = re.search(r'Linear: x=([+-]?\d*\.?\d+)', data_str)
        if linear_match:
            linear_x = float(linear_match.group(1))
        
        # Extract angular z value
        angular_match = re.search(r'Angular: x=[+-]?\d*\.?\d+ y=[+-]?\d*\.?\d+ z=([+-]?\d*\.?\d+)', data_str)
        if angular_match:
            angular_z = float(angular_match.group(1))
        
        return linear_x, angular_z
    except:
        return 0.0, 0.0

# Convert velocity to PWM duty cycle
def velocity_to_pwm(velocity):
    # Convert velocity (-1.0 to 1.0) to PWM duty cycle (0 to 65535)
    # Scale and clamp the velocity
    velocity = max(-1.0, min(1.0, velocity))
    return int(abs(velocity) * 65535)

# Differential drive control
def control_motors(linear_x, angular_z):
    # Calculate left and right wheel velocities for differential drive
    # linear_x: forward/backward motion
    # angular_z: turning motion (positive = counter-clockwise)
    
    left_velocity = linear_x - angular_z
    right_velocity = linear_x + angular_z
    
    # Clamp velocities to valid range
    left_velocity = max(-1.0, min(1.0, left_velocity))
    right_velocity = max(-1.0, min(1.0, right_velocity))
    
    # Convert to PWM values
    left_pwm = velocity_to_pwm(left_velocity)
    right_pwm = velocity_to_pwm(right_velocity)
    
    # Control left motor (Motor A)
    if left_velocity > 0:
        motorA_forward(left_pwm)
    elif left_velocity < 0:
        motorA_backward(left_pwm)
    else:
        motorA_stop()
    
    # Control right motor (Motor B)
    if right_velocity > 0:
        motorB_forward(right_pwm)
    elif right_velocity < 0:
        motorB_backward(right_pwm)
    else:
        motorB_stop()
    
    print(f"Motors - Left: {left_velocity:.3f} ({left_pwm}), Right: {right_velocity:.3f} ({right_pwm})")

print("Pico Motor Controller Started")
print("Waiting for velocity commands via UART...")

# Main control loop
last_command_time = time.ticks_ms()
timeout_ms = 1000  # Stop motors if no command received for 1 second

try:
    while True:
        if uart.any():
            # Read and decode UART data
            received_data = uart.read()
            if received_data:
                try:
                    data_str = received_data.decode('utf-8').strip()
                    print(f"Received: {data_str}")
                    
                    # Parse velocity commands
                    linear_x, angular_z = parse_velocity_data(data_str)
                    
                    # Control motors based on velocity commands
                    control_motors(linear_x, angular_z)
                    
                    # Update last command timestamp
                    last_command_time = time.ticks_ms()
                    
                except Exception as e:
                    print(f"Error processing data: {e}")
        
        # Check for timeout - stop motors if no recent commands
        if time.ticks_diff(time.ticks_ms(), last_command_time) > timeout_ms:
            stop_all_motors()
            last_command_time = time.ticks_ms()  # Reset timer to avoid constant stopping
        
        time.sleep_ms(10)  # Small delay to prevent excessive CPU usage

except KeyboardInterrupt:
    print("Shutting down motor controller")
    stop_all_motors()
    
    # Set all pins to output low for safety
    for i in (2, 3, 4, 5):
        Pin(i, Pin.OUT)
        Pin(i).value(0)
    
    print("Motors stopped safely")