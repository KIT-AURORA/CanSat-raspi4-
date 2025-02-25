import RPi.GPIO as GPIO
import time

# モータードライバーのピン番号に合わせて変更してください
left_front = 12
right_front = 13
left_back = 18
right_back = 19

# GPIOの初期設定
GPIO.setmode(GPIO.BCM)
GPIO.setup(left_front, GPIO.OUT)
GPIO.setup(right_front, GPIO.OUT)
GPIO.setup(left_back, GPIO.OUT)
GPIO.setup(right_back, GPIO.OUT)

def motor_forward(speed, duration):
    GPIO.output(left_front, GPIO.HIGH)
    GPIO.output(right_front, GPIO.HIGH)
    GPIO.output(left_back, GPIO.LOW)
    GPIO.output(right_back, GPIO.LOW)
    time.sleep(duration)
    motor_stop()
    print("Forward")

def motor_backward(speed, duration):
    GPIO.output(left_front, GPIO.LOW)
    GPIO.output(right_front, GPIO.LOW)
    GPIO.output(left_back, GPIO.HIGH)
    GPIO.output(right_back, GPIO.HIGH)
    time.sleep(duration)
    motor_stop()
    print("Backward")

def motor_turn_left(speed, duration):
    GPIO.output(left_front, GPIO.LOW)
    GPIO.output(right_front, GPIO.HIGH)
    GPIO.output(left_back, GPIO.HIGH)
    GPIO.output(right_back, GPIO.LOW)
    time.sleep(duration)
    motor_stop()
    print("Turn Left")

def motor_turn_right(speed, duration):
    GPIO.output(left_front, GPIO.HIGH)
    GPIO.output(right_front, GPIO.LOW)
    GPIO.output(left_back, GPIO.LOW)
    GPIO.output(right_back, GPIO.HIGH)
    time.sleep(duration)
    motor_stop()
    print("Turn Right")

def motor_stop():
    GPIO.output(left_front, GPIO.LOW)
    GPIO.output(right_front, GPIO.LOW)
    GPIO.output(left_back, GPIO.LOW)
    GPIO.output(right_back, GPIO.LOW)
    print("Stop")

try:
    while True:
        # 前進（速度: 50%, 持続時間: 2秒）
        motor_forward(0.2, 2)
        time.sleep(1)
        
        
        # 後退（速度: 40%, 持続時間: 2秒）
        motor_backward(0.2, 2)
        time.sleep(1)

        # 左旋回（速度: 30%, 持続時間: 1秒）
        motor_turn_left(0.2, 2)
        time.sleep(1)


        # 右旋回（速度: 30%, 持続時間: 1秒）
        motor_turn_right(0.2, 2)
        time.sleep(1)

        # 停止（2秒）
        motor_stop()
        time.sleep(2)

except KeyboardInterrupt:
    # Ctrl+Cが押された場合にGPIOをクリーンアップして終了します
    GPIO.cleanup()
