import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def detect_switch():
    """
    スイッチオンの時にTrueを返す
    """
    sw_status = GPIO.input(11)
    return sw_status == 0


while True:
    print(detect_switch())
    time.sleep(1)

