from machine import UART, Pin, PWM
import time

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


# ── Motor helpers ────────────────────────────────────────────────────────────

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


# ── Protocol parser ──────────────────────────────────────────────────────────

def parse_command(line):
    """Parse 'L=x.xxx R=y.yyy' and return (left, right) floats in [-1, 1].

    Returns (0.0, 0.0) on any parse error so the robot stays safe.
    """
    try:
        # Example line: "L=0.500 R=-0.250"
        parts = line.split()
        if len(parts) < 2:
            return 0.0, 0.0
        left  = float(parts[0].split('=')[1])
        right = float(parts[1].split('=')[1])
        # Clamp to valid range
        left  = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))
        return left, right
    except Exception:
        return 0.0, 0.0


# ── Motor driver ─────────────────────────────────────────────────────────────

def apply_motors(left, right):
    """Drive motors from normalised [-1, 1] values.

    The diff_drive_controller has already done the kinematic maths, so the
    Pico just applies left/right values directly — no mixing needed here.
    """
    left_pwm  = int(abs(left)  * 65535)
    right_pwm = int(abs(right) * 65535)

    if left > 0:
        motorA_forward(left_pwm)
    elif left < 0:
        motorA_backward(left_pwm)
    else:
        motorA_stop()

    if right > 0:
        motorB_forward(right_pwm)
    elif right < 0:
        motorB_backward(right_pwm)
    else:
        motorB_stop()

    print("L={:.3f} R={:.3f}".format(left, right))


# ── Main loop ────────────────────────────────────────────────────────────────

print("Pico Motor Controller Started")
print("Protocol: 'L=x.xxx R=y.yyy\\n'")
print("Waiting for commands via UART...")

last_command_time = time.ticks_ms()
TIMEOUT_MS = 1000  # stop motors if no command received for 1 s
rx_buffer = ""

try:
    while True:
        if uart.any():
            chunk = uart.read()
            if chunk:
                try:
                    rx_buffer += chunk.decode('utf-8')
                except Exception:
                    rx_buffer = ""  # discard garbled data

                # Process all complete lines in the buffer
                while '\n' in rx_buffer:
                    line, rx_buffer = rx_buffer.split('\n', 1)
                    line = line.strip()
                    if line.startswith('L=') and 'R=' in line:
                        left, right = parse_command(line)
                        apply_motors(left, right)
                        last_command_time = time.ticks_ms()

        # Safety timeout — stop if Jetson goes silent
        if time.ticks_diff(time.ticks_ms(), last_command_time) > TIMEOUT_MS:
            stop_all_motors()
            last_command_time = time.ticks_ms()  # reset to avoid log spam

        time.sleep_ms(10)

except KeyboardInterrupt:
    print("Shutting down motor controller")
    stop_all_motors()
    for i in (2, 3, 4, 5):
        p = Pin(i, Pin.OUT)
        p.value(0)
    print("Motors stopped safely")
