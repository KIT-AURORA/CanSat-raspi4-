import RPi.GPIO as GPIO
import time

pin = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(pin,GPIO.OUT,initial=GPIO.LOW)

p = GPIO.PWM(pin,1)
p.start(50)

p.ChangeFrequency(220)
time.sleep(3)

p.stop()
GPIO.cleanup()