import smbus
import time
import math


# BMX055アドレス設定
MAG_ADDRESS = 0x13
MAG_REGISTER_ADDRESS = 0x42

# BMX055地磁気センサーデータ取得
def get_bmx055_mag_data():
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


# CanSatが現在向いている方位角を計算
# 北が0度、東が90度、南が180度、西が270度で返す
def calculate_heading(x, y):
    heading = math.atan2(y, x) * (180 / math.pi)
    heading = (heading - 90) % 360  # 北を0度に調整
    return heading


# 地磁気センサーキャリブレーション
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
    # キャリブレーションを実行
    x_offset, y_offset, z_offset = calibrate_magnetometer()

    try:
        while True:
            # 地磁気センサーデータ取得
            x, y, z = get_bmx055_mag_data()
            if x is None or y is None:
                continue

            # キャリブレーション補正
            x -= x_offset
            y -= y_offset

            # 方位角を計算して表示
            current_heading = calculate_heading(x, y)
            print(f"現在の方位角: {current_heading:.2f}°")

            time.sleep(0.5)  # 0.5秒ごとに更新

    except KeyboardInterrupt:
        print("プログラムを終了します")


if __name__ == "__main__":
    main()
