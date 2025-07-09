import subprocess
import time
import re
import sys
import queue
from vosk import Model, KaldiRecognizer
import sounddevice as sd
import json
# import pyttsx3

# input_device = r'DJI-MIC2-0C993D'
input_device = "Wireless Microphone RX"
input_device_similar = r'Monitor'

output_device = r"SoundCore 2"

q = queue.Queue()

def get_bluetooth_input_devices():
    """接続されているBluetoothオーディオデバイスを一覧表示し、サンプルレートも返す"""
    result = subprocess.run(['pactl', 'list', 'sources'], stdout=subprocess.PIPE)
    sources = result.stdout.decode('utf-8').split('\n')
    devices = []
    device_name = None
    sample_rate = None

    for i, line in enumerate(sources):
        if 'Description' in line and re.search(input_device, line) and not re.search(input_device_similar, line):
            device_name = sources[i - 1].split(":")[-1].strip()  # 上の行のNameを取得
            device_description = sources[i].split(":")[-1].strip()  # Descriptionを取得
            print(f"Found input device: {device_description}")
            
            # Sample Specificationを取得
            for j in range(i, len(sources)):
                if 'Sample Specification' in sources[j]:
                    sample_spec = sources[j].split(":")[-1].strip()
                    sample_rate_match = re.search(r'(\d+)Hz', sample_spec)
                    if sample_rate_match:
                        sample_rate = int(sample_rate_match.group(1))
                    break
            
            devices.append({'name': device_name, 'sample_rate': sample_rate})

    return devices

def get_bluetooth_output_devices():
    """接続されているBluetoothオーディオデバイスを一覧表示"""
    result = subprocess.run(['pactl', 'list', 'sinks'], stdout=subprocess.PIPE)
    sinks = result.stdout.decode('utf-8').split('\n')
    devices = []
    device_name = None

    for i, line in enumerate(sinks):
        if 'Description' in line and re.search(output_device, line):
            device_name = sinks[i - 1].split(":")[-1].strip()  # 上の行のNameを取得
            devices.append(device_name)
            device_description = sinks[i].split(":")[-1].strip()  # Descriptionを取得
            print(f"Found output device: {device_description}")

    return devices

def set_input_device(device_name):
    """指定されたBluetoothデバイスを録音デバイスとして設定"""
    try:
        subprocess.run(['pactl', 'set-default-source', device_name], check=True)
        print(f"Input device set to: {device_name}")
    except subprocess.CalledProcessError as e:
        print(f"Error setting input device: {e}")
        
def set_output_device(device_name):
    """指定されたBluetoothデバイスを再生デバイスとして設定"""
    try:
        subprocess.run(['pactl', 'set-default-sink', device_name], check=True)
        print(f"Output device set to: {device_name}")
    except subprocess.CalledProcessError as e:
        print(f"Error setting output device: {e}")

def check_and_set_device():
    """Bluetoothデバイスが接続されているかを確認し、入力デバイスを設定"""
    input_devices = get_bluetooth_input_devices()
    if not input_devices:
        print("No Bluetooth input devices found.")
        return None
    print(f"Input device: {input_devices[0]['name']}")
    print(f"Sample rate: {input_devices[0]['sample_rate']} Hz")
    print("-" * 80)
    set_input_device(input_devices[0]['name'])
    
    # output_devices = get_bluetooth_output_devices()
    # if not output_devices:
    #     print("No Bluetooth output devices found.")
    #     return
    # set_output_device(output_devices[0])

def int_or_str(text):
    """Helper function for argument parsing."""
    try:
        return int(text)
    except ValueError:
        return text

def callback(indata, frames, time, status):
    """音声データをリアルタイムで処理"""
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))
    
    
def voice_output(text, engine):
    # 3秒間チャレンジして無理だったらあきらめる
    count = 0
    while True:
        count += 1
        if count > 6:
            print("Failed to output voice")
            break
        try:
            engine.say(text)
            engine.runAndWait()
            break
        except Exception as e:
            print(f"Error occurred: {e}")
            time.sleep(0.5)
            continue

def recognize_speech():
    
    """音声をリアルタイムで認識し、終了条件を確認する"""
    model = Model("vosk-model-ja-0.22")  # voskモデルのパスを指定
    rec = KaldiRecognizer(model, 16000)


    # キューの作成
    # q = queue.Queue()

    # 音声入力を開始
    with sd.RawInputStream(callback=callback, channels=1, samplerate=16000, blocksize = 8000, device=None, dtype="int16",):
        while True:
            data = q.get()
            if rec.AcceptWaveform(data):
                result_json = rec.Result()  # 結果をJSON文字列として取得
                result = json.loads(result_json)  # JSON文字列を解析して辞書に変換
                current_text = result.get("text", "")
                    
                current_text_cleaned = re.sub(r"\s", "", current_text)
                if current_text_cleaned == "プログラム終了" or current_text_cleaned == "プログラム修了":
                    print("Program ended.")
                    break
                try:

                    # 結果から文字列を取得
                    if current_text_cleaned:
                        print(f"Recognized text: {current_text_cleaned}")
                    else:
                        print("No speech recognized.")


                except json.JSONDecodeError:
                    print("Error decoding JSON result.")

if __name__ == '__main__':
    check_and_set_device()  # Bluetoothデバイス設定
    time.sleep(3)
    # check_and_set_device()  # 再確認

    # 音声認識開始
    recognize_speech()
