import serial
import pynmea2

# シリアルポートを設定
serial_port = '/dev/serial0'  # 実際のシリアルポートを指定
baud_rate = 9600

def read_gps_data(port, baudrate):
    with serial.Serial(port, baudrate, timeout=1) as ser:
        while True:
            try:
                nmea_sentence = ser.readline().decode('ascii', errors='ignore').strip()
                if nmea_sentence.startswith('$'):
                    msg = pynmea2.parse(nmea_sentence)
                    if isinstance(msg, pynmea2.types.talker.GGA):
                        latitude = msg.latitude
                        longitude = msg.longitude
                        print(f"緯度: {latitude}, 経度: {longitude}")
            except pynmea2.ParseError as e:
                print(f"解析エラー: {e}")
            except Exception as e:
                print(f"エラー: {e}")

# GPSデータの読み取りを開始
read_gps_data(serial_port, baud_rate)
