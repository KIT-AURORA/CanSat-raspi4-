import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)


def goalsound():
    pin = 16
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

    p = GPIO.PWM(pin, 1)
    p.start(50)

    p.ChangeFrequency(698)
    time.sleep(0.15)
    p.stop()
    time.sleep(0.05)
    p.start(50)
    p.ChangeFrequency(698)
    time.sleep(0.15)
    p.stop()
    time.sleep(0.05)
    p.start(50)
    p.ChangeFrequency(698)
    time.sleep(0.15)
    p.stop()
    time.sleep(0.05)
    p.start(50)
    p.ChangeFrequency(698)
    time.sleep(0.15)
    p.stop()
    time.sleep(0.05)
    p.start(50)
    p.ChangeFrequency(698)
    time.sleep(0.15)
    p.stop()
    time.sleep(0.25)
    p.start(50)
    p.ChangeFrequency(630)
    time.sleep(0.5)
    p.stop()
    time.sleep(0.05)
    p.start(50)
    p.ChangeFrequency(784)
    time.sleep(0.5)
    p.stop()
    time.sleep(0.05)
    p.start(50)
    p.ChangeFrequency(698)
    time.sleep(1)

    p.stop()
    GPIO.cleanup()
