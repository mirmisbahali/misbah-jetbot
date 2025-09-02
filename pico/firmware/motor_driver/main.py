from machine import Pin, PWM
from time import sleep

# Assign pins
IN1 = PWM(Pin(2))
IN2 = PWM(Pin(3))
IN3 = PWM(Pin(4))
IN4 = PWM(Pin(5))


def motorA_stop():
    IN1.duty_u16(0)
    IN2.duty_u16(0)

def motorB_stop():
    IN3.duty_u16(0)
    IN4.duty_u16(0)


# ---- Demo sequence ----
while True:
    try:
        motorA_stop()
        motorB_stop()
        sleep(1)
    except KeyboardInterrupt:
        break

for i in (2,3,4,5):
    Pin(i, Pin.OUT)

print("Finished...")