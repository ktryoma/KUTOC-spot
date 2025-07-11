import argparse
import logging
import math
import signal
import sys
import threading
import os
import time
import queue
import json
import re
import heapq
import datetime
import multiprocessing
import subprocess

import cv2
import numpy as np

from commands_config import get_command_dict, get_keyboard_commands_dict

from multiprocessing import Process, Queue, Barrier, Value
from math import sqrt
from vosk import Model, KaldiRecognizer

from ultralytics import YOLO

from flask import Flask, jsonify, render_template, send_from_directory  # 既にimport済み

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.util import seconds_to_duration
from bosdyn.client import math_helpers, ResponseError, RpcError, create_standard_sdk
from bosdyn.client.math_helpers import Quat, Vec3
from bosdyn import geometry
from bosdyn.api import geometry_pb2 as geo, image_pb2, trajectory_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient, 
                                         blocking_stand, blocking_sit, 
                                         CommandFailedError, CommandTimedOutError)
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.frame_helpers import (GROUND_PLANE_FRAME_NAME, VISION_FRAME_NAME, 
                                         GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME,
                                         get_a_tform_b,
                                         get_vision_tform_body,
                                         get_odom_tform_body)

from bosdyn.client.robot_command import (CommandFailedError, CommandTimedOutError,
                                         RobotCommandBuilder, RobotCommandClient, 
                                         blocking_stand, blocking_sit, blocking_command)


from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_state import RobotStateClient

from bosdyn.client.recording import GraphNavRecordingServiceClient

import google.protobuf.timestamp_pb2
from google.protobuf import wrappers_pb2 as wrappers

import bosdyn.client.channel
from bosdyn.api.graph_nav import map_pb2, map_processing_pb2, recording_pb2, graph_nav_pb2, nav_pb2
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.map_processing import MapProcessingServiceClient
from bosdyn.client.power import PowerClient

# import Jetson.GPIO as GPIO

print("Imported all libraries")

"""Spotのスピード制御"""
SPEED_LIMIT = 1.2

LOGGER = bosdyn.client.util.get_logger()

SHUTDOWN_FLAG = Value('i', 0)

PROCESS_THREAD = None

PROCESSED_IMAGE_PATH = "image"


q = multiprocessing.Queue()

# SAMPLE_RATE = 16000
# BLOCK_SAZE = 8000
SAMPLE_RATE = 48000
BLOCK_SAZE = 24000

pin_out = 31

# DIRECT_DISTANCE(mm)で撮影したビブスの大きさBOX_HEIGHT(pixel)
# DIRECT_DISTANCE = 3210
# BOX_HEIGHT = 82.63327026367188
# BOX_HEIGHT = 89.59152221679688
REAL_BIBS_HEIGHT = 610
DIRECT_DISTANCE = 2500
BOX_HEIGHT = 65.13135711669922
# BOX_WIDTH = 170
# 撮影した時ビブスがBOX_WIDTH±5であれば止まる
BOX_WIDTH = 160

# 現実世界のビブスの大きさ65cm=650mm
# REAL_BIBS_HEIGHT = 650


# model = YOLO("best.pt")
model = YOLO("allbibs_epoch1000_batch16.pt")


"""入出力デバイスの設定"""
# 入力はヘッドセットで、出力は本体
# input_device = r'DJI-MIC2-0C993D'
# input_device = r"USB Composite Device Mono"
# input_device = r"DJI-MIC2-0C993D"
input_device = r"Wireless Microphone RX"
input_device_similar = r'Monitor'

output_device = r"SoundCore 2"

def get_bluetooth_input_devices():
    """接続されているBluetoothオーディオデバイスを一覧表示し、サンプルレートも返す"""
    result = subprocess.run(['pactl', 'list', 'sources'], stdout=subprocess.PIPE)
    sources = result.stdout.decode('utf-8').split('\n')
    devices = []
    device_name = None
    sample_rate = None

    for i, line in enumerate(sources):
        # if 'Description' in line and re.search(input_device, line) and not re.search(input_device_similar, line):
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
    # 更新: 端末のサンプルレートでグローバル変数SAMPLE_RATEを上書き
    global SAMPLE_RATE
    SAMPLE_RATE = input_devices[0]['sample_rate']
    print(f"Input device: {input_devices[0]['name']}")
    print(f"Sample rate: {SAMPLE_RATE} Hz")
    print("-" * 80)
    set_input_device(input_devices[0]['name'])
    # output_devices = get_bluetooth_output_devices()
    # if not output_devices:
    #     print("No Bluetooth output devices found.")
    #     return False
    # set_output_device(output_devices[0])
    return True

"""カメラ設定に関するプログラム"""
def gstreamer_pipeline(
    device="/dev/video0",
    capture_width=1920,
    capture_height=1280,
    framerate=30,
    format="UYVY",
):
    return (
        "v4l2src io-mode=0 device=%s do-timestamp=true ! "
        "video/x-raw, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1, format=(string)%s ! "
        "videoscale ! "
        "videoconvert ! "
        "appsink"
        % (
            device,
            capture_width,
            capture_height,
            framerate,
            format,
        )
    )


"""音声認識に使用する関数"""

# グローバル変数として音声認識プロセスを保持
audio_process = None

def audio_start():
    global audio_process
    print("aaaaaaaaaaaaaaaaa")
    audio_process = multiprocessing.Process(target=voice_recognition, daemon=True)
    audio_process.start()

def reset_audio_recognition():
    global audio_process
    print("Resetting audio recognition process...")
    if audio_process is not None and audio_process.is_alive():
        audio_process.terminate()
        audio_process.join()
    audio_start()
    print("Audio recognition process restarted.")

def voice_recognition():
    import sounddevice as sd
    
    def callback(indata, frames, time, status):
        # print("called")
        """This is called (from a separate thread) for each audio block."""
        if status:
            print(status, file=sys.stderr)
        q.put(bytes(indata))

    # sd.stop()
    # print(sd.query_devices())
    time.sleep(2)
    print("in this program is used sampling rate ", SAMPLE_RATE)
    BLOCK_SAZE = int(SAMPLE_RATE / 2)
    print("in this program is used block size ", BLOCK_SAZE)
    with sd.RawInputStream(samplerate=SAMPLE_RATE, blocksize=BLOCK_SAZE, device=None, channels=1, callback=callback, dtype="int16"):
        print("#" * 80)
        print("Spot and Vosk are ready to start. Please say the wake word.")
        print("#" * 80)
        while True:
            pass  # 音声処理はコールバックで行う
            time.sleep(0.1)

def int_or_str(text):
    """Helper function for argument parsing."""
    try:
        return int(text)
    except ValueError:
        return text



    
    
"""spotに使用する関数"""

class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client):
        # period is set to be about the same rate as detections on the CORE AI
        super(AsyncRobotState, self).__init__('robot_state', robot_state_client, LOGGER,
                                              period_sec=0.02)

    def _start_query(self):
        return self._client.get_robot_state_async()

def set_default_body_control():
    """Set default body control params to current body position"""
    footprint_R_body = geometry.EulerZXY()
    position = geo.Vec3(x=0.0, y=0.0, z=0.0)
    rotation = footprint_R_body.to_quaternion()
    pose = geo.SE3Pose(position=position, rotation=rotation)
    point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
    traj = trajectory_pb2.SE3Trajectory(points=[point])
    return spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)
    

# --- 統合したモビリティパラメータヘルパー ---
def create_mobility_params(vel_desired=0.5, ang_desired=0.25):
    speed_limit = geo.SE2VelocityLimit(
        max_vel=geo.SE2Velocity(
            linear=geo.Vec2(x=vel_desired, y=vel_desired),
            angular=ang_desired))
    body_control = set_default_body_control()  # 既存の set_default_body_control を利用
    return spot_command_pb2.MobilityParams(vel_limit=speed_limit, obstacle_params=None,
                                             body_control=body_control,
                                             locomotion_hint=spot_command_pb2.HINT_TROT)

def signal_handler(signal, frame):
    print('Interrupt caught, shutting down')
    SHUTDOWN_FLAG.value = 1


def _get_heading(xhat):
    zhat = [0.0, 0.0, 1.0]
    yhat = np.cross(zhat, xhat)
    mat = np.array([xhat, yhat, zhat]).transpose()
    return Quat.from_matrix(mat).to_yaw()



def calculate_angle_speed(angle):
    if angle <= math.pi/4 or angle > 3 * math.pi/4:
        angle_speed = 0.25
    elif angle <= math.pi/2:
        # Linear interpolation between 0.25 and 0.5 for angles between pi/4 and pi/2
        angle_speed = ((math.pi/2 - angle) / (math.pi/2 - math.pi/4)) * (1.0 - 0.25) + 0.25
    else:  # math.pi/2 < angle <= 3 * math_pi/4
        # Linear interpolation between 0.5 and 0.25 for angles between math.pi/2 and 3*math.pi/4
        angle_speed = ((3 * math.pi/4 - angle) / (3 * math.pi/4 - math.pi/2)) * (0.25 - 1.0) + 0.5
    return angle_speed



def move_to_position_cmd_make(robot, x, l, z, box_width, mobility_params=None):
    """Commands the robot to move to a specified x, y position with a specified yaw angle.
    
    Args:
        robot: spotのインスタンス
        x: spotから対象までの向いている方向の距離
        y: spotから対象までの横方向の距離
    """
    # speedの初期化
    speed = 0.0

    # 検出していないときは何もしない
    robot_state = robot.get_robot_state()
    if x == 0 and l == 0 and z == 0:
        print("The robot is close to the target")
        tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(z, x, 0, 
            robot_state.kinematic_state.transforms_snapshot, mobility_params)
        return tag_cmd

    # 距離をmmからmに変換
    x = x / 1000
    l = l / 1000
    z = z / 1000

    # 距離に応じたspeedの計算
    slope = 0.1 * SPEED_LIMIT
    speed = slope * z + 0.1
    if speed >= 2.0:
        speed = 2.0
    print("speed", speed)

    # 検出していないときは何もしない
    if box_width == 0:
        print("any object is not detected")
        speed = 0.0
        x, l, z, = 0, 0, 0

    elif z >=8.5:
        # 遠すぎるため追跡しない
        speed = 0.0
        x, l, z, = 0, 0, 0

    elif z >= 3.5:
        angle = math.atan2(x, z)
        # 角度を±π/8のときは誤差とみなして0にする
        if -math.pi/8 <= angle <= math.pi/8:
            angle = 0
        angle_speed = calculate_angle_speed(angle)
        # 対象の距離まで「進んでほしい位置」まで進むようにする
        z = z - 1.0

    elif z > 2.5:
        # 角度を変える
        # angle = math.atan2(z, x)
        # spot_angle = angle - (math.pi/2)
        # spot_angle = spot_angle * (-1)
        angle = math.atan2(x, z)
        
        # 角度を±π/16のときは誤差とみなして0にする
        if -math.pi/8 <= angle <= math.pi/8:
            angle = 0
        angle_speed = calculate_angle_speed(angle)
        print(f"Angle speed: {angle_speed}")
        # 対象の距離まで「進んでほしい位置」まで進むようにする
        z = z - 1.0

    elif (box_width <= BOX_WIDTH+5 and box_width >= BOX_WIDTH-5) or (z > 2.3 and z <= 2.5):
        # print("The robot is close to the target")
        speed = 0.0
        x, l, z, = 0, 0, 0

    elif z <= 2.3 or box_width >= BOX_WIDTH+5:
        angle_speed = 0.25
        print("The robot is far away from human")
        z = -1.3 + z


    # angle = math.atan2(z, x)
    # spot_angle = angle - (math.pi/2)
    # spot_angle = spot_angle * (-1)
    # angle_speed = calculate_angle_speed(angle)
    # print(f"Angle speed: {angle_speed}")

    # 速度と角速度の設定
    mobility_params = create_mobility_params(speed, angle_speed)

    print("z", z)
    print("x", x)
    print("angle", angle)

    tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(z, x, angle, 
    robot_state.kinematic_state.transforms_snapshot, mobility_params)

    return tag_cmd



def calc_distance(results, img):
    global DIRECT_DISTANCE, BOX_HEIGHT
    boxes = results[0].boxes.xyxy

    x1 = boxes[0][0].item()
    y1 = boxes[0][1].item()
    x2 = boxes[0][2].item()
    y2 = boxes[0][3].item()

    print("x1, y1", x1, y1)
    print("x2, y2", x2, y2)

    # 画像の中心点
    center_x = img.shape[1] / 2
    # print("center_x", center_x)
    center_y = img.shape[0] / 2

    # BBの中心点
    bb_center_x = x1 + ((x2 - x1) / 2)
    bb_center_y = y1 + ((y2 - y1) / 2)

    # BBの高さ = BOX_HEIGHT
    box_height = y2 - y1
    print(f"Box height: {box_height}")

    # BBの横幅
    box_width = x2 - x1
    print(f"Box width: {box_width}")

    # 1ピクセルあたりの距離
    distance_per_pixel = REAL_BIBS_HEIGHT / box_height
    print("1ピクセル当たりの距離mm", distance_per_pixel)

    # 画像の中心点からの距離
    distance_x = center_x - bb_center_x
    print("中心点からの距離", distance_x)

    # 画像の中心点からの現実世界の距離
    x = distance_x * distance_per_pixel
    print(f"Distance_x: {x} mm")

    # カメラから対象までの直線距離
    l = DIRECT_DISTANCE * (BOX_HEIGHT / box_height)
    # print(f"Distance_l: {l} mm")

    # カメラから対象までの縦方向の距離
    z = math.sqrt(l ** 2 - x ** 2)
    # print(f"Distance_z: {z} mm")


    return x, l, z, box_width, box_height

def process_image_thread(robot_state_client, robot_command_client):
    global SHUTDOWN_FLAG
    last_capture_time = 0  # 2秒に1回更新用のタイムスタンプ
    try:
        video_capture = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        update_robot_state(tracking=True)
        if video_capture.isOpened():
            print("Video stream is opened")
            while not SHUTDOWN_FLAG.value:
                # ...existing code...
                try:
                    ret_val, frame = video_capture.read()
                    if ret_val:
                        resized_frame = cv2.resize(frame, (640, 360))
                        results = model(resized_frame, classes=[0], conf=0.8, device=0)
                        if results and results[0].boxes.xyxy.shape[0] > 0:
                            current_time = time.time()
                            # 2秒経過している場合のみ画像を保存
                            if current_time - last_capture_time >= 2.0:
                                timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S%f")[:-4]
                                cv2.imwrite(os.path.join(PROCESSED_IMAGE_PATH, f"detected_{timestamp}.jpg"), resized_frame)
                                last_capture_time = current_time
                            # ...既存の認識検出、コマンド送信処理...
                            for box in results[0].boxes.xyxy:
                                x, l, z, box_width, box_height = calc_distance(results, resized_frame)
                                print(f"Detected object at: x={x}, l={l}, z={z}")
                                left_top_x = int(box[0])
                                right_bottom_x = int(box[2])
                                
                                # 画像の幅
                                img_width = resized_frame.shape[1]
                                
                                robot_state = robot_state_client.get_robot_state()
                                # 左にバウンディングボックスが寄っているとき
                                if left_top_x < 90:
                                    print("The object is on the left side")
                                    
                                    mobility_params = create_mobility_params(0.75, 0.6)
                                    tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(0, 0, math.cos(z/l),
                                        robot_state.kinematic_state.transforms_snapshot, mobility_params)
                                    end_time = 0.3
                                    robot_command_client.robot_command(lease=None, command=tag_cmd,
                                                                    end_time_secs=time.time() + end_time)
                                    # time.sleep(1.0)
                                    
                                # 右にバウンディングボックスが寄っているとき 
                                elif right_bottom_x > img_width - 90:
                                    print("The object is on the right side")
                
                                    mobility_params = create_mobility_params(0.75, 0.6)
                                    tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(0, 0, -math.cos(z/l),
                                        robot_state.kinematic_state.transforms_snapshot, mobility_params)
                                    end_time = 0.3
                                    robot_command_client.robot_command(lease=None, command=tag_cmd,
                                                                    end_time_secs=time.time() + end_time)
                                    # time.sleep(1.0)
                                
                                else:
                                    tag_cmd = move_to_position_cmd_make(robot_state_client, x, l, z, box_width)
                                
                                
                                # tag_cmd = move_to_position_cmd_make(robot_state_client, 0, 0, 0, box_width)
                                cv2.rectangle(resized_frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 0, 255), 2)
                            
                                text1 = f"x={x}"
                                text2 = f"z={z}"
                                # text3 = f"height={boxheight}"
                                # text4 = f"width={box_width}"
                                cv2.putText(resized_frame, text1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                                cv2.putText(resized_frame, text2, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                                # cv2.putText(resize_img, text3, (10, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                                # cv2.putText(resize_img, text4, (10, 330), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                                
                                # Save the image
                                cv2.imwrite(os.path.join(PROCESSED_IMAGE_PATH, f"detected_{timestamp}.jpg"), resized_frame)
                                time.sleep(0.1)
                                
                            end_time = 0.7
                            if tag_cmd is not None:
                                # print("check")
                                print('executing command')
                                robot_command_client.robot_command(lease=None, command=tag_cmd,
                                                                    end_time_secs=time.time() + end_time)
                            else:
                                print('You may mistake any inputs')
                                
                        else:
                            print("No object detected")
                            tag_cmd = move_to_position_cmd_make(robot_state_client, 0, 0, 0, 0)
                            end_time = 2.0
                            if tag_cmd is not None:
                                # print("check")
                                print('executing command')
                                robot_command_client.robot_command(lease=None, command=tag_cmd,
                                                                    end_time_secs=time.time() + end_time)
                                
                        time.sleep(0.1)
                except Exception as e:
                    print(f"Error: {e}")
                    continue
        else:
            print("Failed to open video stream")
    except Exception as e:
        print(f"Error: {e}")
        log_event(f"エラー: {e}")
    finally:
        cv2.destroyAllWindows()
    print("Success to finish process_image_thread")
    return True

robot_state_display = "待機状態"
log_messages = []

app = Flask(__name__)

# サーバー起動中ずっと保持するログリスト
log_messages = []
# ロボットの現在の状態を保持するグローバル変数（初期状態）
current_state = "待機状態"

# 不要なコンソール出力を減らすため、werkzeugのログレベルをWARNINGに設定
log = logging.getLogger('werkzeug')
log.setLevel(logging.WARNING)

# 追加: 最新キャプチャ画像のURLを取得するヘルパー関数
def get_latest_capture_url():
    import glob
    image_files = glob.glob(os.path.join(PROCESSED_IMAGE_PATH, "detected_*.jpg"))
    if image_files:
        latest = max(image_files, key=os.path.getmtime)
        filename = os.path.basename(latest)
        return '/captures/' + filename
    return ""

# 追加: キャプチャ画像を配信するエンドポイント
@app.route("/captures/<path:filename>")
def send_capture(filename):
    return send_from_directory(PROCESSED_IMAGE_PATH, filename)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/update")
def update():
    global log_messages, current_state, tracking_state
    # 追跡中の場合は最新キャプチャ、そうでなければ黒い画像を返す
    capture_url = get_latest_capture_url() if tracking_state else "/captures/black.jpg"
    return jsonify(state=current_state, logs=log_messages, capture=capture_url)

def log_event(message):
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log_line = f"{timestamp} : {message}"
    print(log_line)
    log_messages.append(log_line)

# 状態管理用のグローバル変数
tracking_state = False          # 追跡状態
map_recording_state = False       # 地図作成状態
going_save_state = False          # 保存場所に向かう状態
returning_state = False           # 戻って状態
initializing_state = False        # 地図初期値に向かう状態

def update_robot_state(*, tracking=False, map_recording=False, going_save=False, returning=False, initializing=False):
    """
    各状態のフラグを引数として受け取り、複数の状態がTrueのときは全ての状態名を連結して表示します。
    すべてFalseの場合は「待機状態」として更新します。
    """
    global current_state, tracking_state, map_recording_state, going_save_state, returning_state, initializing_state
    # 各状態の更新
    tracking_state = tracking
    map_recording_state = map_recording
    going_save_state = going_save
    returning_state = returning
    initializing_state = initializing

    # 各状態がTrueの場合、その状態名をリストに追加
    state_list = []
    if tracking_state:
        state_list.append("追跡状態")
    if map_recording_state:
        state_list.append("地図作成状態")
    if going_save_state:
        state_list.append("保存場所に向かう状態")
    if returning_state:
        state_list.append("戻って状態")
    if initializing_state:
        state_list.append("地図初期値に向かう状態")
    # いずれもTrueでなければ「待機状態」とする
    if not state_list:
        state_list.append("待機状態")
    new_state = " / ".join(state_list)
    current_state = new_state
    log_event(f"状態更新: {new_state}")

class CommandProcess():
    
    def __init__(self, robot, robot_state_client, robot_command_client):

        self.robot_state_client = robot_state_client
        self.robot_command_client = robot_command_client
        
        self.thread_running = False
        
        self.allowed_command = {"プログラム終了", "プログラム修了", "止まって"}
        
        self.recording_map = False
        self.initialize = False
        
        self.before_unique_position = None
        self.before_unique_position_id = None
        
        self._is_spot_go_front = False
        self._is_spot_go_back = False
        
    def _reset_audio(self):
        log_event("音声認識をリセットします")
        reset_audio_recognition()
        return False  # プログラム終了にはつながらない
                
        
    def _spot_sit_down(self):
        try:
            if self.recording_map == True:
                print("地図作成中には座れません")
                print("-" * 80)
                return
            
            log_event("命令受理: 座る")
            print("座ります")
            # engine.say("sit down")
            # engine.runAndWait()
            # voice_output("sit down")
            blocking_sit(self.robot_command_client)
            print("-" * 80)
        except Exception as e:
            print(f"Error occurred: {e}")
            return
        
        finally:
            self.thread_running = False
        
    def _spot_stand_up(self):
        try:
            if self.recording_map == True:
                print("地図作成中には立ち上がれません")
                print("-" * 80)
                return
            
            log_event("命令受理: 立つ")
            print("立ち上がります")
            # engine.say("stand up")
            # engine.runAndWait()
            # voice_output("stand up")
            blocking_stand(self.robot_command_client)
            print("-" * 80)
        except Exception as e:
            print(f"Error occurred: {e}")
            return
        
        finally:
            self.thread_running = False
            
        
    def _spot_detect_and_follow(self):
        global PROCESS_THREAD, SHUTDOWN_FLAG
        try:
            if PROCESS_THREAD is None or not PROCESS_THREAD.is_alive():
                log_event("命令受理: 追跡開始")
                print("追跡開始")
                SHUTDOWN_FLAG.value = 0
                PROCESS_THREAD = threading.Thread(target=process_image_thread, args=(self.robot_state_client, self.robot_command_client), daemon=True)
                PROCESS_THREAD.start()
                PROCESS_THREAD.join()
                update_robot_state()
            else:
                print("既に追跡中です")
        except Exception as e:
            log_event(f"エラー: {e}")
            print(f"Error occurred: {e}")
            return
        finally:
            self.thread_running = False
            
    def _spot_stop_follow(self):
        global PROCESS_THREAD
        if PROCESS_THREAD is not None and PROCESS_THREAD.is_alive():
            SHUTDOWN_FLAG.value = 1
            log_event("命令受理: 追跡停止")
            print("追跡終了")
        # ...existing処理...
        log_event("命令完了: 追跡停止完了")
        print("-" * 80)
            
        
    def _spot_go_to_front(self):
        global PROCESS_THREAD
        # engine.say("go to front")
        # engine.runAndWait()
        # voice_output("go to front")
        if PROCESS_THREAD is not None and PROCESS_THREAD.is_alive():
            SHUTDOWN_FLAG.value = 1
        print("前進します")
        log_event("命令受理: 前進")
        if self._is_spot_go_front == False:
            # まだ前進していないとき
            self._is_spot_go_front = True
            self.start_time = time.time()
            front_thread = threading.Thread(target=self._move_front_thread, daemon=True)
            front_thread.start()
            front_thread.join()
        # if self._is_spot_go_front == True:
        #     # 前進中のとき
        #     # 進んでを2回言っても止まるようにはなってる
        #     self._is_spot_go_front = False
        #     return
        self.thread_running = False
            
    def _spot_move_front_thread(self):
        while self._is_spot_go_front:
            robot_state = self.robot_state_client.get_robot_state()
            mobility_params = create_mobility_params(0.3, 0.5)
            tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(1, 0, 0,
                                    robot_state.kinematic_state.transforms_snapshot, mobility_params)
            end_time = 0.5
            if tag_cmd is not None:
                print('executing command')
                self.robot_command_client.robot_command(lease=None, command=tag_cmd,
                                                        end_time_secs=time.time() + end_time)
                
            walk_end_time = time.time()
            if walk_end_time - self.start_time >= 15.0:
                self._is_spot_go_front = False
                return
            
    def _spot_go_to_back(self):
        global PROCESS_THREAD
        # engine.say("go to back")
        # engine.runAndWait()
        # voice_output("go to back")
        if PROCESS_THREAD is not None and PROCESS_THREAD.is_alive():
            SHUTDOWN_FLAG.value = 1
        print("後退します")
        log_event("命令受理: 後退")
        if self._is_spot_go_back == False:
            # まだ後退していないとき
            self._is_spot_go_back = True
            self.back_start_time = time.time()
            back_thread = threading.Thread(target=self._move_back_thread, daemon=True)
            back_thread.start()
            back_thread.join()
        # if self._is_spot_go_back == True:
        #     # 後退中のとき
        #     # 進んでを2回言っても止まるようにはなってる
        #     self._is_spot_go_back = False
        #     return
        
        self.thread_running = False
        
    def _spot_move_back_thread(self):
        while self._is_spot_go_back:
            robot_state = self.robot_state_client.get_robot_state()
            mobility_params = create_mobility_params(0.1, 0.5)
            tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(-1, 0, 0,
                                    robot_state.kinematic_state.transforms_snapshot, mobility_params)
            end_time = 0.5
            if tag_cmd is not None:
                print('executing command')
                self.robot_command_client.robot_command(lease=None, command=tag_cmd,
                                                        end_time_secs=time.time() + end_time)
                
            walk_end_time = time.time()
            if walk_end_time - self.back_start_time >= 10.0:
                self._is_spot_go_back = False
                return
            
    def _spot_go_front_little(self):
        try:
            log_event("命令受理: 少し前進")
            robot_state = self.robot_state_client.get_robot_state()
            mobility_params = create_mobility_params(0.8, 0.5)
            tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(0.5, 0, 0,
                                    robot_state.kinematic_state.transforms_snapshot, mobility_params)
            end_time = 2.0
            if tag_cmd is not None:
                print('executing command')
                self.robot_command_client.robot_command(lease=None, command=tag_cmd,
                                                        end_time_secs=time.time() + end_time)
                
        except Exception as e:
            print(e)
            log_event(f"エラー: {e}")
        
        finally:
            self.thread_running = False

        return
    
    def _spot_go_back_little(self):
        try:
            log_event("命令受理: 少し後退")
            robot_state = self.robot_state_client.get_robot_state()
            mobility_params = create_mobility_params(0.8, 0.5)
            tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(-0.5, 0, 0,
                                    robot_state.kinematic_state.transforms_snapshot, mobility_params)
            end_time = 2.0
            if tag_cmd is not None:
                print('executing command')
                self.robot_command_client.robot_command(lease=None, command=tag_cmd,
                                                        end_time_secs=time.time() + end_time)
                
        except Exception as e:
            log_event(f"エラー: {e}")
            print(e)
            
        finally:
            self.thread_running = False

        return

    
    def _spot_go_right_little(self):
        try:
            log_event("命令受理: 少し右に移動")
            robot_state = self.robot_state_client.get_robot_state()
            mobility_params = create_mobility_params(0.8, 0.5)
            tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(0, -0.5, 0,
                                    robot_state.kinematic_state.transforms_snapshot, mobility_params)
            end_time = 2.0
            if tag_cmd is not None:
                print('executing command')
                self.robot_command_client.robot_command(lease=None, command=tag_cmd,
                                                        end_time_secs=time.time() + end_time)
                
        except Exception as e:
            log_event(f"エラー: {e}")
            print(e)
            
        finally:
            self.thread_running = False

        return
    
    def _spot_go_left_little(self):
        try:
            log_event("命令受理: 少し左に移動")
            robot_state = self.robot_state_client.get_robot_state()
            mobility_params = create_mobility_params(0.8, 0.5)
            tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(0, 0.5, 0,
                                    robot_state.kinematic_state.transforms_snapshot, mobility_params)
            end_time = 2.0
            if tag_cmd is not None:
                print('executing command')
                self.robot_command_client.robot_command(lease=None, command=tag_cmd,
                                                        end_time_secs=time.time() + end_time)
                
        except Exception as e:
            log_event(f"エラー: {e}")
            print(e)
            
        finally:
            self.thread_running = False

        return
    
    def _spot_pose_bow(self):
        
        log_event("命令受理: お辞儀")
        robot_state = self.robot_state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
        # Specify a trajectory to shift the body forward followed by looking down, then return to nominal.
        # Define times (in seconds) for each point in the trajectory.
        t1 = 0.5
        t2 = 2.0
        t3 = 6.0
        
        degree = 20
        w = math.cos(math.radians(degree/2))
        y = math.sin(math.radians(degree/2))

        # Specify the poses as transformations to the cached flat_body pose.
        flat_body_T_pose1 = math_helpers.SE3Pose(x=0, y=0, z=0, rot=math_helpers.Quat())
        flat_body_T_pose2 = math_helpers.SE3Pose(
            x=0.0, y=0, z=0, rot=math_helpers.Quat(w=w, x=0, y=y, z=0))
        flat_body_T_pose3 = math_helpers.SE3Pose(x=0.0, y=0, z=0, rot=math_helpers.Quat())

        # Build the points in the trajectory.
        traj_point1 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body * flat_body_T_pose1).to_proto(),
            time_since_reference=seconds_to_duration(t1))
        traj_point2 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body * flat_body_T_pose2).to_proto(),
            time_since_reference=seconds_to_duration(t2))
        traj_point3 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body * flat_body_T_pose3).to_proto(),
            time_since_reference=seconds_to_duration(t3))

        # Build the trajectory proto by combining the points.
        traj = trajectory_pb2.SE3Trajectory(points=[traj_point1, traj_point2, traj_point3])

        # Build a custom mobility params to specify absolute body control.
        body_control = spot_command_pb2.BodyControlParams(
            body_pose=spot_command_pb2.BodyControlParams.BodyPose(root_frame_name=ODOM_FRAME_NAME,
                                                                  base_offset_rt_root=traj))
        blocking_stand(self.robot_command_client, timeout_sec=10,
                       params=spot_command_pb2.MobilityParams(body_control=body_control))
        
        time.sleep(3.5)
        self.thread_running = False
        
    def _spot_rotate(self):
        log_event("命令受理: 回転")
        robot_state = self.robot_state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
        # Specify a trajectory to shift the body forward followed by looking down, then return to nominal.
        # Define times (in seconds) for each point in the trajectory.
        mobility_params = create_mobility_params(0.75, 0.6)
        tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(0, 0, math.pi,
            robot_state.kinematic_state.transforms_snapshot, mobility_params)
        end_time = 20.0
        self.robot_command_client.robot_command(lease=None, command=tag_cmd,
                                        end_time_secs=time.time() + end_time)
        
        
    def _exit_program(self):
        global PROCESS_THREAD, SHUTDOWN_FLAG
        print("プログラムを終了します")
        if PROCESS_THREAD is not None and PROCESS_THREAD.is_alive():
            SHUTDOWN_FLAG.value = 1
            
        if self.recording_map == True:
            self.rcl._stop_recording()
            self.recording_map = False
            
        if self.gcl._is_finished == False:
            self.gcl._is_finished = True            
        
        return True
    
# キーボード入力をキューに追加する関数
def capture_keyboard_input(keyboard_queue):
    while True:
        key = input("Press a key: ")
        keyboard_queue.put(key)
        time.sleep(0.1)
            
            
def keyboard_input_thread(keyboard_queue, cp):
    while True:
        if not keyboard_queue.empty():
            key = keyboard_queue.get()
            if key == "q":
                if cp._exit_program():
                    return True  # プログラム終了のキーを受け付けた際に True を返す
                            
            if key in cp.keyboard_command_dict:
                command_func = cp.keyboard_command_dict[key]
                if command_func():
                    return True  # コマンド関数が True を返した場合に True を返す
                
        time.sleep(0.1)
                
def audio_recog(cp, rec):
    spot_detect_time = None

    while True:
        if not q.empty():
            data = q.get()
            if rec.AcceptWaveform(data):
                result_json = rec.Result()  # 結果をJSON文字列として取得
                result = json.loads(result_json)  # JSON文字列を解析して辞書に変換
                current_text = result.get("text", "")
                    
                current_text_cleaned = re.sub(r"\s", "", current_text)
                if current_text_cleaned == "プログラム終了" or current_text_cleaned == "プログラム修了":
                    if cp._exit_program():
                        break
                if current_text_cleaned.startswith("スポット"):
                    spot_detect_time = time.time()
                    command = current_text_cleaned.replace("スポット", "").strip()
                            
                    if command in cp.command_dict:
                        command_func = cp.command_dict[command]
                        if command_func:                       
                            print(f"コマンド: {command} を実行します")
                            # cp._execute_command(command)
                            current_thread = threading.Thread(target=command_func, daemon=True)
                            current_thread.start()
                        else:
                            print(f"コマンド: {command} は実行できません")
                            log_event(f"コマンド: {command} は実行できません")
                    else:
                        print(f"無効なコマンド: {command}")
                        log_event(f"無効なコマンド: {command}")
                        continue
                            
                else:
                    if current_text_cleaned:
                        print(f"コマンドが無視されました: '{current_text_cleaned}' は 'スポット' で始まる必要があります。")
                        log_event(f"コマンドが無視されました: '{current_text_cleaned}' は 'スポット' で始まる必要があります。")
                    else:
                        print("コマンドが無視されました: 空の音声が認識されました。")
                    continue
                
        time.sleep(0.1)
                
def main():
    global SHUTDOWN_FLAG

    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    # bosdyn.client.util.add_payload_credentials_arguments(parser)
    options = parser.parse_args()
    signal.signal(signal.SIGINT, signal_handler)
    print("start program")
    try:
        sdk = bosdyn.client.create_standard_sdk('SpotFollowClient')
        robot = sdk.create_robot(options.hostname)

        recording_sdk = bosdyn.client.create_standard_sdk('SpotRecordingClient')
        print("before authtication")
        # if options.payload_credentials_file:
        #     robot.authenticate_from_payload_credentials(
        #         *bosdyn.client.util.get_guid_and_secret(options))
        # else:
        #     bosdyn.client.util.authenticate(robot)
        bosdyn.client.util.authenticate(robot)
        print("Authetication finish")
        # Time sync is necessary so that time-based filter requests can be converted
        robot.time_sync.wait_for_sync()
        print("time sync")

        # Verify the robot is not estopped and that an external application has registered and holds
        # an estop endpoint.
        assert not robot.is_estopped(), 'Robot is estopped. Please use an external E-Stop client,' \
                                        ' such as the estop SDK example, to configure E-Stop.'

        print("estop")
        robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        lease_client = robot.ensure_client(LeaseClient.default_service_name)

        print("create client")

        robot_state_task = AsyncRobotState(robot_state_client)

        print('Detect and follow client connected.')

        lease = lease_client.take()
        lease_keep = LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True)
        # Power on the robot and stand it up
        resp = robot.power_on()

        try:
            blocking_stand(robot_command_client)
        except CommandFailedError as exc:
            print(f'Error ({exc}) occurred while trying to stand. Check robot surroundings.')
            return False
        except CommandTimedOutError as exc:
            print(f'Stand command timed out: {exc}')
            return False
        print('Robot powered on and standing.')
        params_set = create_mobility_params()
        

        """音声認識系の処理"""
        print("sound setting")
        model = Model("vosk-model-ja-0.22")
        
        
        """入出力デバイスの設定"""
        print("audio setting")
        sound_check = check_and_set_device()
        if sound_check == None:
            print("音声デバイスの設定に失敗しました")
            return
        
        cp = CommandProcess(robot, robot_state_client, robot_command_client)
        
        
        # 修正: 固定の16000ではなく、動的に取得したSAMPLE_RATEを使用
        print(SAMPLE_RATE)
        rec = KaldiRecognizer(model, SAMPLE_RATE)
        
        # 生成済みでない場合、黒い画像（640x360）を作成して保存する
        os.makedirs(PROCESSED_IMAGE_PATH, exist_ok=True)
        black_image_path = os.path.join(PROCESSED_IMAGE_PATH, "black.jpg")
        if not os.path.exists(black_image_path):
            import numpy as np
            # 画像サイズは 360x640 (高さ×幅)
            black_img = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.imwrite(black_image_path, black_img)
        
        app_thread = threading.Thread(
            target=app.run, 
            args=("0.0.0.0", 5000, False),
            daemon=True
        )
        app_thread.start()
        
        
        print("keyboad setting")
        keyboard_queue = queue.Queue()
        key_capture_thread = threading.Thread(target=capture_keyboard_input, args=(keyboard_queue,), daemon=True)
        key_capture_thread.start()
        
        key_input_thread = threading.Thread(target=keyboard_input_thread, args=(keyboard_queue, cp), daemon=True)
        key_input_thread.start()
        
        audio_thread = threading.Thread(target=audio_recog, args=(cp, rec), daemon=True)
        audio_thread.start()


        print("audio start")

        # sd.stop()
        audio_start()
        
        # cp._spot_pose_bow()            

        while True:
            if not key_input_thread.is_alive():
                print("key input thread is dead")
                break
                
            if not audio_thread.is_alive():
                print("audio thread is dead")
                break

            time.sleep(0.1)

        print("the program will exit normally")
        log_event("program will be finished")

    except Exception as exc:  # pylint: disable=broad-except
        LOGGER.error('Spot jetson threw an exception: %s', exc)
        return False

if __name__ == "__main__":
    main()
