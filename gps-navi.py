import math
import time
import pynmea2
import serial
import smbus
from geopy.distance import geodesic
from gpiozero import Motor

# BMX055アドレス設定
ACC_ADDRESS = 0x19
MAG_ADDRESS = 0x13
ACC_REGISTER_ADDRESS = 0x02
MAG_REGISTER_ADDRESS = 0x42

# # 現在の緯度・経度
# TARGET_LAT = 33.891901  # 緯度
# TARGET_LON = 130.839993  # 経度
# 北　目標の緯度・経度
# TARGET_LAT = 33.8902370  # 緯度
# TARGET_LON = 130.8404987  # 経度
# # 東　目標の緯度・経度
# TARGET_LAT = 33.891901  # 緯度
# TARGET_LON = 130.939993  # 経度
# # 南　目標の緯度・経度
# TARGET_LAT = 33.791901  # 緯度
# TARGET_LON = 130.839993  # 経度
# # 西　目標の緯度・経度
#TARGET_LAT = 33.891901  # 緯度
#TARGET_LON = 130.739993  # 経度
# 今回目標S棟前十字路
# TARGET_LAT = 33.8928172  # 緯度
# TARGET_LON = 130.8398221  # 経度
# ground
TARGET_LAT = 33.8910340  # 緯度
TARGET_LON = 130.8407471  # 経度

# モーター設定
# left_motor = Motor(forward=18, backward=12)
# right_motor = Motor(forward=13, backward=19)
left_motor = Motor(forward=12, backward=18)
right_motor = Motor(forward=13, backward=19)

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
        return x, y, z
    except IOError as e:
        print(f"I2Cエラー: {e}")
        return None, None, None


def calculate_heading(x, y):
    """
    引数は地磁気x,y.CanSatの向いている方位を示す。コンパスみたいな感じ
    ・math.atan2(y, x)
    得られた地磁気x,yの逆関数をとってその点の角度を求める。返り値は-πからπ
    ・math.atan2(y, x) * (180 / math.pi)
    ラジアン表記から度数表記に変更。（-180度から180度）
    ・compass_heading = (initial_heading - 90) % 360
    -180度から180度を0~360に
    北を0度、東を90度、南を180度、西を270度に調整
    %360 は負の値が出た時に調整するため
    式に用いた値は実際に9軸と角度をテストして決定した
    """
    initial_heading = math.atan2(y, x) * (180 / math.pi)
    compass_heading = (initial_heading - 90) % 360  # 北を0度に調整
    return compass_heading


def calculate_bearing(current_lat, current_lon, target_lat, target_lon):
    """
    CanSatと目的地の緯度経度が引数。
    返す値は、目標地点の方向
    球面座標方程式？みたいなものを用いて、自身を原点としたときの目的地の座標をx,yで求め
    calculate_heading同じ流れで角度を求める
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
    return compass_bearing


# モーター制御（PID制御を使用）
def control_motors(target_bearing, current_heading):
    """
    引数は目標地点の方位とCanSatの方位
    方位の誤差をもとにPID制御でモーターの値を返す
    Kp,Ki,Kd の適した値を実験により求める
    最初は小さく見積もって、かつKpのみでの制御から行うとよいかも
    """
    """
    編集すべき場所
    """
    Kp = 0.005  # 比例制御。パラメータ１
    Ki =0  # 積分制御。パラメータ２
    Kd =0  # 微分制御。パラメータ３

    error = target_bearing - current_heading  # 誤差の計算
    error = (error + 180) % 360 - 180
    """
    -360~360で表されていた誤差を-180から180の範囲に正規化
    350度の誤差は-10度の誤差に。これにより時計回り、反時計回りの制御が可能に
    """

    # 微分、積分の計算
    global integral, prev_error  # 関数の中で定義した変数を関数外で取り出せるように
    integral += error  # 積分項　これまでの誤差の蓄積。これにより長期間の誤差を補正
    derivative = error - prev_error  # 誤差の変化量。微分項。急激な変化に対応
    prev_error = error  # 現在の誤差を保存

    # モーターの回転速度の計算（PID制御）
    # 正の値で右旋回、負の値で左旋回。-1~1になるように調整したい
    turn_rate = Kp * error + Ki * integral + Kd * derivative

    # モーターの速度設定
    """
    turn_rate が正: left_speed が増え、right_speed が減少 → 右に旋回。
    turn_rate が負: left_speed が減り、right_speed が増加 → 左に旋回。
    max,minをもちいて-1~1になるように
    """
    based_speed = 1
    left_speed = max(min(based_speed + turn_rate, 1), 0)
    right_speed = max(min(based_speed - turn_rate, 1), 0)

    left_motor.value = left_speed
    right_motor.value = right_speed

    return error, turn_rate, left_speed, right_speed


def calibrate_magnetometer_with_rotation():
    """
    地磁気は周辺環境によって変わるため、値の校正を行う
    1回転させたとき、X,Yの奇跡である円の原点が0になるように調整
    X,Yの最大値、最小値を求め、その平均を原点からのズレoffsetとする
    """
    print(
        "地磁気センサーのキャリブレーションを開始します。CanSatが自動で5秒旋回します..."
    )
    x_min = y_min = z_min = 1000000
    x_max = y_max = z_max = -1000000
    start_time = time.time()
    # キャリブレーション中、モーターでCanSatを旋回させる
    rotation_speed = 0.5
    left_motor.value = rotation_speed
    right_motor.value = -rotation_speed
    try:
        while time.time() - start_time < 10:  # 10秒間キャリブレーション 短くした
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
        print(f"キャリブレーション中のエラー: {e}")
    finally:
        # キャリブレーション終了後にモーターを停止
        left_motor.stop()
        right_motor.stop()
    x_offset = (x_min + x_max) / 2
    y_offset = (y_min + y_max) / 2
    z_offset = (z_min + z_max) / 2
    print("キャリブレーション完了")
    return (x_offset, y_offset, z_offset)


def main():
    """
    作った関数を組み合わせたmain処理
    """
    global integral, prev_error
    integral = 0
    prev_error = 0

    x_offset, y_offset, z_offset = calibrate_magnetometer_with_rotation()

    try:
        while True:
            current_lat, current_lon = get_gps()
            if (
                current_lat is None or current_lon is None
            ):  # 緯度経度がとれなかったらループの先頭に戻る
                continue

            distance_to_target = geodesic(
                (current_lat, current_lon), (TARGET_LAT, TARGET_LON)
            ).meters  # 目標までの距離を求める。ライブラリを仕様
            if distance_to_target < 1:  # 3→1 テストのため変更:  # ゴール判定
                left_motor.stop()
                right_motor.stop()
                print("目標地点に到達しました！")
                break

            x, y, z = get_bmx055_mag_data()
            if x is None or y is None:  # 地磁気がとれなかったらループの先頭に戻る
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
            if left_speed - right_speed >= 0:  # 旋回方向を表示
                houkou = "右"
            else:
                houkou = "左"

            # print("\n=== ナビゲーション情報 ===")
            # print(f"現在の方位: {current_heading:.1f}°")
            # print(f"現在の緯度: {current_lat:.6f}")
            # print(f"現在の経度: {current_lon:.6f}")
            # print(f"目標の方位: {target_bearing:.1f}°")
            print(f"{error:.0f}")  # 方位の誤差:
            #print(f"旋回レート: {turn_rate:.3f}")
            # print(f"目標までの距離: {distance_to_target:.1f}m")
            print(f"左のスピード: {left_speed:.3f}")
            print(f"右のスピード: {right_speed:.3f}")
            # print(f"{houkou}に旋回")  # 旋回方向を表示
            # print("=====================")
            # control_motors(target_bearing, current_heading)  # モータ制御
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("プログラムを終了します")
        left_motor.stop()
        right_motor.stop()


if __name__ == "__main__":  # このプログラムが呼び出されたらmain()を実行
    main()
