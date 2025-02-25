import smbus
import time
import pigpio

servo_pin = 17
pi = pigpio.pi()  # pigpioを初期化

# BMX055アドレス設定
ACC_ADDRESS = 0x19
MAG_ADDRESS = 0x13
GYRO_ADDRESS = 0x69
ACC_REGISTER_ADDRESS = 0x02
MAG_REGISTER_ADDRESS = 0x42
GYRO_REGISTER_ADDRESS = 0x02


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
    assert 0 <= angle <= 180
    pulse_width = (angle / 180) * (2500 - 500) + 500
    pi.set_servo_pulsewidth(servo_pin, pulse_width)


def main():
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
    set_angle(5)
    time.sleep(5)

    print("分離完了。走行開始")


main()
