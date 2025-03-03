import serial
import pynmea2
import matplotlib.pyplot as plt
import geopandas as gpd
from shapely.geometry import Point, LineString
import pandas as pd
import matplotlib.image as mpimg
from matplotlib.ticker import ScalarFormatter
from pyproj import Transformer

# シリアルポート設定
serial_port = "/dev/serial0"  # GPSモジュールが接続されているシリアルポート
baud_rate = 9600
coordinates = []  # 座標リスト


# GPSデータを読み取る関数。
def read_gps_data(port, baudrate):
    with serial.Serial(port, baudrate, timeout=1) as ser:
        while True:
            try:
                nmea_sentence = ser.readline().decode("ascii", errors="ignore").strip()
                if nmea_sentence.startswith("$"):
                    msg = pynmea2.parse(nmea_sentence)
                    if isinstance(msg, pynmea2.types.talker.GGA):
                        latitude = msg.latitude
                        longitude = msg.longitude
                        print(f"緯度: {latitude}, 経度: {longitude}")
                        coordinates.append((longitude, latitude))
            except pynmea2.ParseError as e:
                print(f"解析エラー: {e}")
            except KeyboardInterrupt:
                print("終了します...")
                break
            except Exception as e:
                print(f"エラー: {e}")


def get_image_bounds(image_path):
    # 実際の値は、使用する衛星画像に応じて変更
    return {
        "left": 130.838004,  # 左端の経度
        "right": 130.8440090,  # 右端の経度
        "top": 33.8940315,  # 上端の緯度
        "bottom": 33.8902386,  # 下端の緯度
    }


# 地図を生成して保存する関数
def create_map(coordinates, background_image_path):
    if len(coordinates) > 1:
        df = pd.DataFrame(coordinates, columns=["longitude", "latitude"])
        geometry = [Point(xy) for xy in zip(df["longitude"], df["latitude"])]
        gdf = gpd.GeoDataFrame(df, geometry=geometry, crs="EPSG:4326")
        line = LineString(gdf.geometry.tolist())
        gdf_line = gpd.GeoDataFrame(geometry=[line], crs="EPSG:4326")
        # 衛星画像を読み込む
        img = mpimg.imread(background_image_path)
        # 衛星画像の境界を取得
        bounds = get_image_bounds(background_image_path)
        # 図を設定
        fig, ax = plt.subplots(figsize=(10, 6))
        # 衛星画像を表示
        ax.imshow(
            img,
            extent=[bounds["left"], bounds["right"], bounds["bottom"], bounds["top"]],
            aspect="equal",
        )
        # ルート（線）を赤色で描画
        gdf_line.plot(ax=ax, color="red", linewidth=2)
        # スタート地点とゴール地点をプロット
        start_point = coordinates[0]  # 最初の座標
        goal_point = coordinates[-1]  # 最後の座標
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
        # 軸ラベルを追加
        ax.set_xlabel("Longitude")
        ax.set_ylabel("Latitude")
        ax.xaxis.set_major_formatter(ScalarFormatter(useOffset=False))
        ax.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
        # タイトルを追加
        plt.title("GPS Route on Satellite Image")
        # 画像先を指定して保存
        plt.savefig(
            "/home/ozo/cansat_route_with_background.png", dpi=300, bbox_inches="tight"
        )
        plt.show()


# GPSデータの読み取り開始
read_gps_data(serial_port, baud_rate)

# 終了時に地図を作成して保存
create_map(coordinates, "/home/ozo/background_image.png")  # 衛星画像の保存先を指定
