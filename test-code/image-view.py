from picamera2 import Picamera2
import cv2
import numpy as np
import gpiozero
import time

# カメラの初期化
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
minimum_area = 10000  # パラメータ１。これより小さいと未検出とみなす
maximum_area = 300000  # パラメータ２。この面積以上でゴール判定
# モーターの設定
robot = gpiozero.Robot(left=(18, 12), right=(13, 19))  # モーターのピン配置の設定
forward_speed = 0.7  # パラメータ３
turn_speed = 0.002  # パラメータ４

HUE_VAL = 120  # パラメータ５。Hの値

# HSVの設定
lower_color = np.array([HUE_VAL - 10, 100, 55])  # パラメータ６
upper_color = np.array([HUE_VAL + 10, 255, 255])  # パラメータ７

try:
    while True:
        # 画像取得
        image = camera.capture_array()

        # 画像に分割ラインを描画
        cv2.line(
            image,
            (int(center_image_x - image_width / 6), 0),
            (int(center_image_x - image_width / 6), image_height),
            (0, 255, 0),
            2,
        )
        cv2.line(
            image,
            (int(center_image_x + image_width / 6), 0),
            (int(center_image_x + image_width / 6), image_height),
            (0, 255, 0),
            2,
        )

        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)  # RGBからHSVに変換

        color_mask = cv2.inRange(
            hsv, lower_color, upper_color
        )  # 目的の色だけ残すマスク

        contours, _ = cv2.findContours(
            color_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
        )  # マスクされた色の輪郭を描く

        # 変数の作成
        object_area = 0
        object_x = 0
        object_y = 0

        for contour in contours:  # 輪郭を描いいたオブジェクトに対してループ
            x, y, width, height = cv2.boundingRect(contour)
            found_area = width * height
            # ボックスの中心座標
            center_x = x + (width / 2)
            center_y = y + (height / 2)
            # 既に見つかっている最大面積と比較
            if object_area < found_area:
                object_area = found_area
                object_x = center_x
                object_y = center_y

        # 見つかったオブジェクトの情報を格納
        if object_area > 0:
            ball_location = [object_area, object_x, object_y]
            # 検出されたオブジェクトに矩形を描画
            cv2.rectangle(
                image,
                (int(object_x - width / 2), int(object_y - height / 2)),
                (int(object_x + width / 2), int(object_y + height / 2)),
                (255, 0, 0),
                2,
            )
        else:
            ball_location = None

        # 画像表示
        cv2.imshow("Robot Camera", image)
        cv2.imshow("Color Mask", color_mask)

        if ball_location:
            if (ball_location[0] > minimum_area) and (ball_location[0] < maximum_area):
                if ball_location[1] > (center_image_x + (image_width / 6)):
                    robot.right(turn_speed)
                    print("右折")
                elif ball_location[1] < (center_image_x - (image_width / 6)):
                    robot.left(turn_speed)
                    print("左折")
                else:
                    robot.forward(forward_speed)
                    print("前進")
            elif ball_location[0] < minimum_area:
                robot.left(turn_speed)
                print("ターゲット未検知, 捜索します")
            else:
                robot.stop()
                print("ターゲット到着, 停止します")
        else:
            robot.left(turn_speed)
            print("ターゲットが見つかりません")

        # キー入力待ち（'q'キーで終了）
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    print("プログラムを終了します")
    robot.stop()
finally:
    # クリーンアップ
    cv2.destroyAllWindows()
    camera.stop()
