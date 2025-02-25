import gpiozero
from pynput import keyboard
import signal
import sys

# ロボットの初期化
robot = gpiozero.Robot(left=(19, 13), right=(12, 18))


# 移動関数
def move_forward():
    robot.forward()


def move_backward():
    robot.backward()


def turn_left():
    robot.left()


def turn_right():
    robot.right()


def stop():
    robot.stop()


# キー入力に応じた動作
def on_press(key):
    try:
        if key.char == "w":
            move_forward()
        elif key.char == "s":
            move_backward()
        elif key.char == "a":
            turn_left()
        elif key.char == "d":
            turn_right()
    except AttributeError:
        pass


def on_release(key):
    stop()
    if key == keyboard.Key.esc:
        return False


# クリーンアップ関数
def cleanup():
    print("\nプログラムを終了します。")
    stop()  # モーターを停止
    robot.close()  # GPIOリソースを解放


# Ctrl+C のシグナルハンドラ
def signal_handler(sig, frame):
    cleanup()
    sys.exit(0)


# メイン処理
if __name__ == "__main__":
    # Ctrl+C のシグナルハンドラを設定
    signal.signal(signal.SIGINT, signal_handler)

    print(
        "ラジコン制御を開始します。終了するには 'Esc' キーまたは Ctrl+C を押してください。"
    )

    try:
        # キーボードリスナーの設定
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()
    except Exception as e:
        print(f"予期せぬエラーが発生しました: {e}")
    finally:
        cleanup()
