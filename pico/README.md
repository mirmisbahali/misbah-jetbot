The Jetson Nano B01 board only has 2 PWM pins so I used a raspberry pi pico to control the motors as I couldn't source a PWM compatible motor driver. This folder contains the firmware flashed on the Pico which is resonbile for driving the motors.


# MX1508 Motor Driver

```python
# Pin Configuration 
IN1 = 2
IN2 = 3
IN3 = 4
IN4 = 5
```