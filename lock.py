import time
import pigpio

servo_pin = 17
pi = pigpio.pi()  # pigpioを初期化


def set_angle(angle):
    assert 0 <= angle <= 180
    pulse_width = (angle / 180) * (2500 - 500) + 500
    pi.set_servo_pulsewidth(servo_pin, pulse_width)


set_angle(80)
