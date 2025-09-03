# On the Pico (MicroPython)
from machine import UART, Pin
u = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
while True:
    if u.any():
        received_data = u.read()
        print("Received:", received_data.decode())# echo back

