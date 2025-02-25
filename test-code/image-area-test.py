from picamera2 import Picamera2
import cv2
import numpy as np

camera = Picamera2()
camera_config = camera.create_preview_configuration(
    main={"format": "RGB888", "size": (640, 480)}
)
camera.configure(camera_config)
camera.start()

image_width = 640
image_height = 480
center_image_x = image_width / 2
center_image_y = image_height / 2

HUE_VAL = 120  # 赤色のHUE値に合わせて設定

# 赤色のHSV範囲を指定
lower_color = np.array([HUE_VAL - 10, 100, 100])  # 赤色の下限値
upper_color = np.array([HUE_VAL + 10, 255, 255])  # 赤色の上限値

while True:
    image = camera.capture_array()

    # カメラ画像をHSVに変換
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    # 赤色マスクを作成
    color_mask = cv2.inRange(hsv, lower_color, upper_color)

    # マスクの輪郭を検出
    contours, _ = cv2.findContours(color_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    object_area = 0
    object_x = 0
    object_y = 0

    for contour in contours:
        x, y, width, height = cv2.boundingRect(contour)
        found_area = width * height
        if object_area < found_area:  # 一番大きな赤色の物体を検出
            object_area = found_area
            object_x = x + (width / 2)
            object_y = y + (height / 2)

    # 検出した赤色の面積を表示
    print(f"赤色の面積: {object_area}")

    # 赤色の物体に枠を描画
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # カメラ画像を表示
    cv2.imshow("Camera Image", image)
    cv2.imshow("Red Color Mask", color_mask)

    # キー入力の待機
    k = cv2.waitKey(1) & 0xFF
    if k == 27:  # Escキーでループを終了
        break

# カメラを停止し、ウィンドウを閉じる
camera.stop()
cv2.destroyAllWindows()
