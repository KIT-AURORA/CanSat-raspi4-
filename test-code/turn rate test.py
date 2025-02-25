import serial
import pynmea2
import smbus
import time
import math
from geopy.distance import geodesic

# BMX055アドレス設定
MAG_ADDRESS = 0x13
MAG_REGISTER_ADDRESS = 0x42

# 目標地点の緯度・経度
TARGET_LAT = 35.681236  # 例: 東京駅の緯度
TARGET_LON = 139.767125  # 例: 東京駅の経度

# GPSの設定
serial_port = "/dev/serial0"
baud_rate = 9600


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
                        return latitude, longitude
            except pynmea2.ParseError:
                continue
            except Exception as e:
                print(f"GPSエラー: {e}")
                return None, None


def get_bmx055_mag_data():
    """
    BMX055地磁気センサーデータx,y,zを返す関数
    """
    i2c = smbus.SMBus(1)

    try:
        data = i2c.read_i2c_block_data(MAG_ADDRESS, MAG_REGISTER_ADDRESS, 6)
        x = (data[1] << 8) | data[0]
        y = (data[3] << 8) | data[2]
        z = (data[5] << 8) | data[4]
        x = x if x < 32768 else x - 65536
        y = y if y < 32768 else y - 65536
        z = z if z < 32768 else z - 65536
        return x, y, z
    except IOError as e:
        print(f"I2Cエラー: {e}")
        return None, None, None


def calculate_heading(x, y):
    """
    地磁気データから現在の方位を計算
    """
    initial_heading = math.atan2(y, x) * (180 / math.pi)
    compass_heading = (initial_heading - 90) % 360
    return compass_heading


def calculate_bearing(current_lat, current_lon, target_lat, target_lon):
    """
    現在位置から目標地点への方位を計算
    """
    diff_lon = math.radians(target_lon - current_lon)
    lat1 = math.radians(current_lat)
    lat2 = math.radians(target_lat)

    x = math.sin(diff_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (
        math.sin(lat1) * math.cos(lat2) * math.cos(diff_lon)
    )

    initial_bearing = math.atan2(y, x) * (180 / math.pi)
    compass_bearing = (initial_bearing) % 360
    return compass_bearing


def calculate_turn_rate(target_bearing, current_heading):
    """
    PID制御による旋回レートを計算して表示
    """
    Kp = 0.005
    Ki = 0
    Kd = 0

    error = target_bearing - current_heading
    error = (error + 180) % 360 - 180

    global integral, prev_error
    integral += error
    derivative = error - prev_error
    prev_error = error

    turn_rate = Kp * error + Ki * integral + Kd * derivative

    # -1から1の範囲に制限
    turn_rate = max(min(turn_rate, 1), -1)

    return turn_rate, error


def calibrate_magnetometer():
    print(
        "地磁気センサーのキャリブレーションを開始します。CanSatをゆっくり回転させてください..."
    )
    x_min = y_min = z_min = 1000000
    x_max = y_max = z_max = -1000000

    start_time = time.time()
    while time.time() - start_time < 15:  # 30秒間キャリブレーション
        x, y, z = get_bmx055_mag_data()
        if x is not None and y is not None and z is not None:
            x_min = min(x_min, x)
            x_max = max(x_max, x)
            y_min = min(y_min, y)
            y_max = max(y_max, y)
            z_min = min(z_min, z)
            z_max = max(z_max, z)
        time.sleep(0.1)

    x_offset = (x_min + x_max) / 2
    y_offset = (y_min + y_max) / 2
    z_offset = (z_min + z_max) / 2

    print("キャリブレーション完了")
    return x_offset, y_offset, z_offset


def main():
    """
    メイン処理
    """
    global integral, prev_error
    integral = 0
    prev_error = 0

    x_offset, y_offset, z_offset = calibrate_magnetometer()

    try:
        while True:
            current_lat, current_lon = get_gps()
            if current_lat is None or current_lon is None:
                print("GPS信号を待機中...")
                continue

            distance_to_target = geodesic(
                (current_lat, current_lon), (TARGET_LAT, TARGET_LON)
            ).meters

            if distance_to_target < 3:
                print("目標地点に到達しました！")
                break

            x, y, z = get_bmx055_mag_data()
            if x is None or y is None:
                print("地磁気センサーデータを待機中...")
                continue

            # オフセット補正
            x -= x_offset
            y -= y_offset

            current_heading = calculate_heading(x, y)
            target_bearing = calculate_bearing(
                current_lat, current_lon, TARGET_LAT, TARGET_LON
            )

            turn_rate, error = calculate_turn_rate(target_bearing, current_heading)

            # ナビゲーション情報の表示
            print("\n=== ナビゲーション情報 ===")
            print(f"現在の方位: {current_heading:.1f}°")
            print(f"目標の方位: {target_bearing:.1f}°")
            print(f"方位の誤差: {error:.1f}°")
            print(f"旋回レート: {turn_rate:.3f}")
            print(f"目標までの距離: {distance_to_target:.1f}m")
            print("=====================")

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("プログラムを終了します")


if __name__ == "__main__":
    main()
