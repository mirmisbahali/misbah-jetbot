from machine import Pin, PWM
from time import sleep

# Assign pins
IN1 = PWM(Pin(2))
IN2 = PWM(Pin(3))
IN3 = PWM(Pin(4))
IN4 = PWM(Pin(5))

# Set PWM frequency (about 1 kHz works well)
for pin in (IN1, IN2, IN3, IN4):
    pin.freq(1000)

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

def steer_left(speed):
    motorA_forward(speed)
    motorB_backward(speed)

def steer_right(speed):
    motorA_backward(speed)
    motorB_forward(speed)


def motorB_stop():
    IN3.duty_u16(0)
    IN4.duty_u16(0)

# ---- Demo sequence ----
while True:
    try:
        motorA_forward(40000)   # ~60% duty
        motorB_forward(40000)
        sleep(2)

        motorA_backward(40000)
        motorB_backward(40000)
        sleep(2)

        steer_left(40000)
        sleep(2)

        steer_right(40000)
        sleep(2)

        motorA_stop()
        motorB_stop()
        sleep(1)
    except KeyboardInterrupt:
        break

for i in (2,3,4,5):
    Pin(i, Pin.OUT)

print("Finished...")
