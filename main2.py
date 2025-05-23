import math
import time
import pigpio
import RPi.GPIO as GPIO
import pynmea2
import serial
import smbus
from geopy.distance import geodesic
from gpiozero import Motor
from picamera2 import Picamera2
import cv2
import numpy as np
import matplotlib.pyplot as plt
import geopandas as gpd
from shapely.geometry import Point, LineString
import pandas as pd
import matplotlib.image as mpimg
from matplotlib.ticker import ScalarFormatter
import logging

# サーボ設定
servo_pin = 17
pi = pigpio.pi()  # pigpioを初期化
# 落下スイッチ　設定
GPIO.setmode(GPIO.BCM)
GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# BMX055アドレス設定
ACC_ADDRESS = 0x19
MAG_ADDRESS = 0x13
ACC_REGISTER_ADDRESS = 0x02
MAG_REGISTER_ADDRESS = 0x42
# ゴール座標
TARGET_LAT = 33.8923535  # 緯度
TARGET_LON = 130.8405258  # 経度
# モーター設定
left_motor = Motor(forward=12, backward=18)
right_motor = Motor(forward=13, backward=19)
# GPSの設定
serial_port = "/dev/serial0"
baud_rate = 9600
# オブジェクトの生成、初期化など
camera = Picamera2()
camera_config = camera.create_preview_configuration(
    main={"format": "RGB888", "size": (320, 240)}
)
camera.configure(camera_config)
camera.start()
# 画像の大きさ
image_width = 320
image_height = 240
# 画像の中心座標
center_image_x = image_width / 2
center_image_y = image_height / 2
# 認識面積の設定
minimum_area = 750
maximum_area = 75000
# 画像認識設定
HUE_VAL = 120
lower_color = np.array([HUE_VAL - 10, 100, 55])
upper_color = np.array([HUE_VAL + 10, 255, 255])
# GPSの軌跡を保存するリスト
gps_coordinates = []
# ログ設定
logging.basicConfig(
    filename="Navigation.log",
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
)


def detect_switch():
    """
    スイッチオンの時にTrueを返す
    """
    sw_status = GPIO.input(11)
    return sw_status == 0


def get_gps():
    """
    GPSのデータを取得して、緯度経度を返す関数
    """
    with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
        while True:
            try:
                nmea_sentence = ser.readline().decode("ascii", errors="ignore").strip()
                if nmea_sentence.startswith("$"):
                    msg = pynmea2.parse(nmea_sentence)
                    if isinstance(msg, pynmea2.types.talker.GGA):
                        latitude = msg.latitude
                        longitude = msg.longitude
                        gps_coordinates.append((longitude, latitude))
                        logging.info(f"GPS:[{latitude:.5f},{longitude:.5f}]")
                        return latitude, longitude
            except pynmea2.ParseError:
                continue
            except Exception as e:
                logging.error(f"GPSエラー:{e}")
                print(f"GPSエラー: {e}")
                return None, None


def get_bmx055_mag_data():
    """
    BMX055地磁気センサーデータx,y,zを返す関数
    """
    i2c = smbus.SMBus(1)
    i2c.write_byte_data(MAG_ADDRESS, 0x4B, 0x01)
    i2c.write_byte_data(MAG_ADDRESS, 0x4C, 0x00)
    try:
        data = i2c.read_i2c_block_data(MAG_ADDRESS, MAG_REGISTER_ADDRESS, 6)
        x = (data[1] << 8) | data[0]
        y = (data[3] << 8) | data[2]
        z = (data[5] << 8) | data[4]
        x = x if x < 32768 else x - 65536
        y = y if y < 32768 else y - 65536
        z = z if z < 32768 else z - 65536
        logging.info(f"BMX055:[{x:.5f},{y:.5f},{z:.5f}]")
        return x, y, z
    except IOError as e:
        logging.error(f"I2Cエラー:{e}")
        print(f"I2Cエラー: {e}")
        return None, None, None


def get_bmx055_accel_data():
    """
    BMX055加速度センサーデータx, y, zを返す関数
    """
    i2c = smbus.SMBus(1)
    i2c.write_byte_data(ACC_ADDRESS, 0x0F, 0x03)  # 加速度範囲の設定
    i2c.write_byte_data(ACC_ADDRESS, 0x10, 0x08)  # バンド幅の設定
    i2c.write_byte_data(ACC_ADDRESS, 0x11, 0x00)  # パワーモードの設定
    try:
        data = i2c.read_i2c_block_data(ACC_ADDRESS, ACC_REGISTER_ADDRESS, 6)
        x = (data[1] << 8) | data[0]
        y = (data[3] << 8) | data[2]
        z = (data[5] << 8) | data[4]
        x = x if x < 32768 else x - 65536
        y = y if y < 32768 else y - 65536
        z = z if z < 32768 else z - 65536
        return x * 0.00098, y * 0.00098, z * 0.00098  # 値の調整
    except IOError as e:
        print(f"I2Cエラー: {e}")
        return None, None, None


def calculate_acc(x, y, z):
    """
    accは静止状態で±0.5未満になるように調整
    """
    return (x**2 + y**2 + z**2) ** 0.5 - 16


def set_angle(angle):
    """
    サーボの角度設定
    """
    assert 0 <= angle <= 180
    pulse_width = (angle / 180) * (2500 - 500) + 500
    pi.set_servo_pulsewidth(servo_pin, pulse_width)


def calculate_heading(x, y):
    """
    引数は地磁気x,y.CanSatの向いている方位を示す。
    """
    initial_heading = math.atan2(y, x) * (180 / math.pi)
    compass_heading = (initial_heading - 90) % 360
    logging.info(f"compassheading = {compass_heading}")
    return compass_heading


def calculate_bearing(current_lat, current_lon, target_lat, target_lon):
    """
    返す値は、目標地点の方向
    """
    diff_lon = math.radians(target_lon - current_lon)
    lat1 = math.radians(current_lat)
    lat2 = math.radians(target_lat)
    x = math.sin(diff_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (
        math.sin(lat1) * math.cos(lat2) * math.cos(diff_lon)
    )
    initial_bearing = math.atan2(x, y) * (180 / math.pi)
    compass_bearing = (initial_bearing) % 360
    compass_bearing = (initial_bearing + 360) % 360
    logging.info(f"compass_bearing = {compass_bearing}")
    return compass_bearing


def control_motors(target_bearing, current_heading):
    """
    方位の誤差をもとにPID制御でモーターの値を返す
    """
    Kp = 0.01  # 比例制御。
    Ki = 0.0005  # 積分制御。
    Kd = -1  # 微分制御。

    error = target_bearing - current_heading
    error = (error + 180) % 360 - 180
    global integral, prev_error
    integral += error  # 積分項　これまでの誤差の蓄積。これにより長期間の誤差を補正
    derivative = error - prev_error  # 誤差の変化量。微分項。急激な変化に対応
    prev_error = error  # 現在の誤差を保存

    # 正の値で右旋回、負の値で左旋回。
    turn_rate = Kp * error + Ki * integral + Kd * derivative
    # モーターの速度設定
    based_speed = 1
    left_speed = max(min(based_speed + turn_rate, 1), 0)
    right_speed = max(min(based_speed - turn_rate, 1), 0)
    left_motor.value = left_speed
    right_motor.value = right_speed
    return error, turn_rate, left_speed, right_speed


def calibrate_magnetometer_with_rotation():
    """
    地磁気の校正
    """
    logging.info(
        "地磁気センサーのキャリブレーションを開始します。CanSatが自動で5秒旋回します..."
    )
    print(
        "地磁気センサーのキャリブレーションを開始します。CanSatが自動で5秒旋回します..."
    )
    x_min = y_min = z_min = 1000000
    x_max = y_max = z_max = -1000000
    start_time = time.time()
    rotation_speed = 0.5
    left_motor.value = rotation_speed
    right_motor.value = -rotation_speed
    try:
        while time.time() - start_time < 5:  # 5秒間キャリブレーション 短くした
            x, y, z = get_bmx055_mag_data()
            if x is not None and y is not None and z is not None:
                x_min = min(x_min, x)
                x_max = max(x_max, x)
                y_min = min(y_min, y)
                y_max = max(y_max, y)
                z_min = min(z_min, z)
                z_max = max(z_max, z)
            time.sleep(0.1)  # 少し待機してデータ取得を行う
    except Exception as e:
        logging.error(f"キャリブレーション中のエラー:{e}")
        print(f"キャリブレーション中のエラー: {e}")
    finally:
        left_motor.stop()
        right_motor.stop()
    x_offset = (x_min + x_max) / 2
    y_offset = (y_min + y_max) / 2
    z_offset = (z_min + z_max) / 2
    logging.info("キャリブレーション完了")
    print("キャリブレーション完了")
    logging.info(f"x_offset = {x_offset},y_offset = {y_offset},z_offset = {z_offset}")
    return (x_offset, y_offset, z_offset)


def gps_navigartion():
    """
    GPS走行処理
    """
    global integral, prev_error
    integral = 0
    prev_error = 0
    x_offset, y_offset, z_offset = calibrate_magnetometer_with_rotation()
    try:
        while True:
            current_lat, current_lon = get_gps()
            if current_lat is None or current_lon is None:
                continue

            distance_to_target = geodesic(
                (current_lat, current_lon), (TARGET_LAT, TARGET_LON)
            ).meters
            print(distance_to_target)
            if distance_to_target < 1:
                left_motor.stop()
                right_motor.stop()
                break

            x, y, z = get_bmx055_mag_data()
            if x is None or y is None:
                continue
            x -= x_offset
            y -= y_offset

            current_heading = calculate_heading(x, y)
            target_bearing = calculate_bearing(
                current_lat, current_lon, TARGET_LAT, TARGET_LON
            )

            error, turn_rate, left_speed, right_speed = control_motors(
                target_bearing, current_heading
            )
            # errorとturn＿rateをログに残したい
            logging.info(f"turn_rate = {turn_rate},error = {error}")
            control_motors(target_bearing, current_heading)
            time.sleep(0.1)
    except KeyboardInterrupt:
        logging.error("プログラムを終了します")
        print("プログラムを終了します")
        left_motor.stop()
        right_motor.stop()


def image_navigartion():
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
                if (ball_location[0] > minimum_area) and (
                    ball_location[0] < maximum_area
                ):
                    if ball_location[1] > (center_image_x + (image_width / 4)):
                        right_motor.forward(0.2)
                        left_motor.backward(0)
                        logging.info("右折")
                        print("右折")
                    elif ball_location[1] < (center_image_x - (image_width / 4)):
                        right_motor.backward(0.2)
                        left_motor.forward(0)
                        logging.info("左折")
                        print("左折")
                    else:
                        right_motor.forward(0.2)
                        left_motor.forward(0.2)
                        logging.info("前進")
                        print("前進")
                elif ball_location[0] < minimum_area:
                    right_motor.backward(0.2)
                    left_motor.forward(0)
                    logging.info("ターゲット未見地、捜索します")
                    print("ターゲット未検知, 捜索します")
                else:
                    right_motor.stop()
                    left_motor.stop()
                    logging.info("ターゲット到着、停止します")
                    print("ターゲット到着, 停止します")
                    break
            else:
                right_motor.backward(0.2)
                left_motor.forward(0)
                logging.info("タ-ゲットが見つかりません")
                print("ターゲットが見つかりません")
    except KeyboardInterrupt:
        logging.info("プログラムを終了します")
        print("プログラムを終了します")
        right_motor.stop()
        left_motor.stop()
    finally:
        left_motor.stop()
        right_motor.stop()


# background_image1の座標
# def get_image_bounds(image_path):
#     """衛星画像の境界座標を返す"""
#     return {
#         "left": 130.838004,
#         "right": 130.8440090,
#         "top": 33.8940315,
#         "bottom": 33.8902386,
#     }


def get_image_bounds(image_path):
    """衛星画像の境界座標を返す"""
    return {
        "left": 130.8384075,
        "right": 130.8427423,
        "top": 33.8912873,
        "bottom": 33.8891393,
    }


def create_map(coordinates, background_image_path):
    """GPSの軌跡を地図上にプロットする"""
    if len(coordinates) > 1:
        df = pd.DataFrame(coordinates, columns=["longitude", "latitude"])
        geometry = [Point(xy) for xy in zip(df["longitude"], df["latitude"])]
        gdf = gpd.GeoDataFrame(df, geometry=geometry, crs="EPSG:4326")
        line = LineString(gdf.geometry.tolist())
        gdf_line = gpd.GeoDataFrame(geometry=[line], crs="EPSG:4326")

        img = mpimg.imread(background_image_path)
        bounds = get_image_bounds(background_image_path)

        fig, ax = plt.subplots(figsize=(10, 6))
        ax.imshow(
            img,
            extent=[bounds["left"], bounds["right"], bounds["bottom"], bounds["top"]],
            aspect="equal",
        )
        gdf_line.plot(ax=ax, color="red", linewidth=2)

        start_point = coordinates[0]
        goal_point = coordinates[-1]
        ax.plot(
            start_point[0],
            start_point[1],
            marker="o",
            markersize=10,
            color="green",
            label="Start",
        )
        ax.plot(
            goal_point[0],
            goal_point[1],
            marker="o",
            markersize=10,
            color="red",
            label="Goal",
        )

        plt.legend()
        ax.set_xlim(bounds["left"], bounds["right"])
        ax.set_ylim(bounds["bottom"], bounds["top"])
        ax.set_xlabel("Longitude")
        ax.set_ylabel("Latitude")
        ax.xaxis.set_major_formatter(ScalarFormatter(useOffset=False))
        ax.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
        plt.title("GPS Route on Satellite Image")
        plt.savefig(
            "/home/Keito1091/cansat_route_with_background.png", dpi=300, bbox_inches="tight"
        )
        plt.close()


def separation():
    start_time = None
    stable_time = 5  # 静止時間
    while True:
        acc_x, acc_y, acc_z = get_bmx055_accel_data()
        acc = calculate_acc(acc_x, acc_y, acc_z)
        print(acc)
        if -0.5 < acc < 0.5:
            if start_time is None:
                start_time = time.time()
            elif time.time() - start_time >= stable_time:
                print("着地判定: CanSatが静止しました。")
                break
        else:
            start_time = None
        time.sleep(0.1)
    set_angle(5)  # 解除
    print("分離完了。走行開始")


def main():
    """
    落下→スイッチon→着地判定→分離→少し進む
    """
    while True:
        if detect_switch():
            break
    separation()
    time.sleep(1)
    right_motor.forward(1)
    left_motor.forward(1)
    time.sleep(5)
    print("start")
    time.sleep(1)
    gps_navigartion()
    logging.info("画像走行をはじめます")
    print("画像走行をはじめます")
    image_navigartion()
    if gps_coordinates:
        create_map(gps_coordinates, "/home/Keito1091/background_image2.png")
    left_motor.stop()
    right_motor.stop()
    logging.info("終了")
    print("終了")


if __name__ == "__main__":
    main()
