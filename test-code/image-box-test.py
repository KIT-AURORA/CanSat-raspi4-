import time
import cv2
import numpy as np
from picamera2 import Picamera2


def stackImages(
    scale, imgArray
):  # 4つの画像を上手く表示させるためのコード。内容はchatgptに書かせたのでよくわからないけど動きます
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(
                        imgArray[x][y], (0, 0), None, scale, scale
                    )
                else:
                    imgArray[x][y] = cv2.resize(
                        imgArray[x][y],
                        (imgArray[0][0].shape[1], imgArray[0][0].shape[0]),
                        None,
                        scale,
                        scale,
                    )
                if len(imgArray[x][y].shape) == 2:
                    imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank] * rows
        hor_con = [imageBlank] * rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(
                    imgArray[x],
                    (imgArray[0].shape[1], imgArray[0].shape[0]),
                    None,
                    scale,
                    scale,
                )
            if len(imgArray[x].shape) == 2:
                imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver


# カメラの初期設定
camera = Picamera2()
camera_config = camera.create_preview_configuration(
    main={"format": "RGB888", "size": (640, 480)}
)
camera.configure(camera_config)
camera.start()

cv2.namedWindow("Result", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Result", 1280, 960)

while True:
    while True:
        try:  # Hue値の入力を受け取る
            hue_value = int(input("Hue value between 10 and 245: "))
            if (hue_value < 10) or (hue_value > 245):
                raise ValueError
        except ValueError:  # 不正な入力があった場合のエラーメッセージ
            print("That isn't an integer between 10 and 245, try again")
        else:
            break

    # 識別のの範囲
    lower_red = np.array(
        [hue_value - 10, 100, 50]
    )  # 場合によってパラメータを変更する必要がある。
    upper_red = np.array([hue_value + 10, 255, 150])

    while True:
        # フレームをキャプチャ
        image = camera.capture_array()
        # RGBからHSVに変換
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        # 色範囲に基づいてマスクを作成
        color_mask = cv2.inRange(hsv, lower_red, upper_red)
        # マスクを使って元の画像と色マスクを組み合わせる
        result = cv2.bitwise_and(image, image, mask=color_mask)

        # 画像を結合して表示
        stacked_images = stackImages(0.8, ([image, hsv], [color_mask, result]))
        cv2.imshow("Result", stacked_images)

        # キー入力の待機
        k = cv2.waitKey(1) & 0xFF
        if k == 27:  # Escキーでループを終了
            break

    if k == 27:
        break

# ウィンドウを閉じ、カメラを停止
cv2.destroyAllWindows()
camera.stop()
