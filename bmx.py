import smbus
import time

# BMX055アドレス設定
ACC_ADDRESS = 0x19
MAG_ADDRESS = 0x13
ACC_REGISTER_ADDRESS = 0x02
MAG_REGISTER_ADDRESS = 0x42


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


while True:
    print(get_bmx055_mag_data())
    time.sleep(1)
