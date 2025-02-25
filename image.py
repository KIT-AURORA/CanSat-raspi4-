from picamera2 import Picamera2
import cv2
import numpy as np
import gpiozero
import time
from gpiozero import Motor

# オブジェクトの生成、初期化など
camera = Picamera2()
camera_config = camera.create_preview_configuration(
    main={"format": "RGB888", "size": (640, 480)}
)
camera.configure(camera_config)
camera.start()
# 画像の大きさ
image_width = 640
image_height = 480
# 画像の中心座標
center_image_x = image_width / 2
center_image_y = image_height / 2
# 認識面積の設定
minimum_area = 1500
maximum_area = 300000
# モーター設定
left_motor = Motor(forward=12, backward=18)
right_motor = Motor(forward=13, backward=19)
# 画像認識設定
HUE_VAL = 120
lower_color = np.array([HUE_VAL - 10, 100, 55])
upper_color = np.array([HUE_VAL + 10, 255, 255])

try:
    while True:
        image = camera.capture_array()
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        color_mask = cv2.inRange(hsv, lower_color, upper_color)
        contours, _ = cv2.findContours(
            color_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
        )
        object_area = 0
        object_x = 0
        object_y = 0

        for contour in contours:
            x, y, width, height = cv2.boundingRect(contour)
            found_area = width * height
            center_x = x + (width / 2)
            center_y = y + (height / 2)
            if object_area < found_area:
                object_area = found_area
                object_x = center_x
                object_y = center_y
        if object_area > 0:
            ball_location = [object_area, object_x, object_y]
        else:
            ball_location = None
        if ball_location:
            if (ball_location[0] > minimum_area) and (ball_location[0] < maximum_area):
                if ball_location[1] > (center_image_x + (image_width / 6)):
                    left_motor.stop()
                    right_motor.forward(0.4)
                    print("left")
                elif ball_location[1] < (center_image_x - (image_width / 6)):
                    left_motor.forward(0.4)
                    right_motor.stop()
                    print("right")
                else:
                    right_motor.forward(1)
                    left_motor.forward(1)
                    print("forward")
            elif ball_location[0] < minimum_area:
                left_motor.forward(0.4)
                right_motor.stop()
                print("undetect")
            else:
                right_motor.stop()
                left_motor.stop()
                print("detect")
        else:
            left_motor.forward(0.4)
            right_motor.stop()
            print("ターゲットが見つかりません")
except KeyboardInterrupt:
    print("プログラムを終了します")
    right_motor.stop()
    left_motor.stop()
