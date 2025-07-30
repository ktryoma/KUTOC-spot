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

from multiprocessing import Process, Queue, Barrier, Value
from math import sqrt

import cv2
import numpy as np

from vosk import Model, KaldiRecognizer
from ultralytics import YOLO
from flask import Flask, jsonify, render_template, send_from_directory, request

import src.graph_nav_util as graph_nav_util
import src.dijkstra as dijkstra
from src.commands_config import (get_command_dict, get_keyboard_commands_dict,
                                 get_web_commands_dict)
from src.graph_nav import GraphNavInterface, RecordingInterface
from src.tracking import Tracking
from src.web import WebServer

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
from bosdyn.client.docking import DockingClient, blocking_dock_robot, blocking_undock, get_dock_id, blocking_go_to_prep_pose
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

import Jetson.GPIO as GPIO

print("Imported all libraries")
"""Spotのスピード制御"""
SPEED_LIMIT = 1.2

LOGGER = bosdyn.client.util.get_logger()

SHUTDOWN_FLAG = Value('i', 0)

PROCESS_THREAD = None
LEASE = None
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



class CommandProcess():
    
    def __init__(self, robot, client_meta_data, robot_state_client, robot_command_client, web_server):

        self.robot = robot
        self.web_server = web_server
        
        self.rcl = RecordingInterface(robot, os.getcwd(), client_meta_data, False)
        self.gcl = GraphNavInterface(robot, "downloaded_graph", False)

        self.tracking = Tracking(self.web_server.log_event, self.web_server.update_robot_state, model, create_mobility_params,
                                 SPEED_LIMIT, BOX_WIDTH, BOX_HEIGHT, DIRECT_DISTANCE,
                                 PROCESSED_IMAGE_PATH, SHUTDOWN_FLAG, REAL_BIBS_HEIGHT)

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
        
        
        self.spot_moving_command = {
            "ついてきて", "着いてきて",
            "スタート", "戻れ", "戻って", "ナビゲート", "保存場所", "トラック",
        }
        
        self.command_dict = get_command_dict(self)
        
        self.keyboard_command_dict = get_keyboard_commands_dict(self)
        
        
    def _execute_command(self, command):
        # if self.thread_running and command not in the allowed_command:
        #     print("他のコマンドを実行中です")
        #     return
        
        if command in self.allowed_command:
            self.thread_running = False
            self.command_dict[command]()
            return
        
        if command in self.command_dict:
            self.thread_running = True
            self.current_thread = threading.Thread(target=self.command_dict[command], daemon=True)
            self.current_thread.start()
            return
        
    def _reset_audio(self):
        self.web_server.log_event("音声認識をリセットします")
        reset_audio_recognition()
        return False  # プログラム終了にはつながらない
            
    def _make_map(self):
        try:
            if self.recording_map == True:
                self.web_server.log_event("地図作成中: 既に地図作成中です")
                print("地図作成中です")
                print("-" * 80)
                return
            else:
                self.web_server.log_event("命令受理: 地図作成開始")
                print("地図を作成開始します")
                self.rcl._start_recording()
                self.recording_map = True
                self.web_server.update_robot_state(map_recording=True)
                print("-" * 80)
        except Exception as e:
            self.web_server.log_event(f"エラー: {e}")
            self.web_server.update_robot_state(map_recording=False)
            print(f"Error occurred: {e}")
            return
        
    def _take_odm_current_and_create_waypoint(self):
        
        initial_robot_state = self.robot_state_client.get_robot_state()
        initial_pose = get_a_tform_b(initial_robot_state.kinematic_state.transforms_snapshot,
                                 ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME).position
        
        prev_x, prev_y, prev_z = initial_pose.x, initial_pose.y, initial_pose.z
        
        # self.waypoints = []
        while self.recording_map:
            try:
                current_robot_state = self.robot_state_client.get_robot_state()
                current_pose = get_a_tform_b(current_robot_state.kinematic_state.transforms_snapshot,
                                    ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME).position
                    
                curr_x, curr_y, curr_z = current_pose.x, current_pose.y, current_pose.z
                    
                distance = math.sqrt((curr_x - prev_x) ** 2 + (curr_y - prev_y) ** 2 + (curr_z - prev_z) ** 2)
                    
                if distance >= 1.0:
                    prev_x, prev_y, prev_z = curr_x, curr_y, curr_z
                    self.rcl._create_default_waypoint_odm()

            except Exception as e:
                print(f"Error occurred while creating waypoints: {e}")
                
    def _stop_making_map_downloading(self):
        global PROCESS_THREAD
        if PROCESS_THREAD is not None and PROCESS_THREAD.is_alive():
            SHUTDOWN_FLAG.value = 1
            print("追跡終了")
            time.sleep(2.0)
            
        self.web_server.log_event("命令受理: 地図作成終了")
        if self.recording_map == True:
            try:
                print("地図作成を終了します")
                self.rcl._stop_recording()
                self.web_server.log_event("地図作成終了: 地図作成を終了しました")
                print("recording stop")
                time.sleep(10.0)
                self.recording_map = False
                close_fiducial_loops = True
                close_odometry_loops = True
                self.rcl._auto_close_loops(close_fiducial_loops, close_odometry_loops)
                self.web_server.log_event("地図作成終了: ループクローズを実行しました")
                time.sleep(5)
                print("-" * 80)
                self.rcl._download_full_graph()
                self.gcl._list_graph_waypoint_and_edge_ids()
                self.web_server.log_event("地図作成終了: 地図をアップロードしました")
                print("-" * 80)
            except Exception as e:
                self.web_server.log_event(f"エラー: {e}")
                print(f"Error occurred: {e}")
                return
            finally:
                self.thread_running = False
                self.web_server.update_robot_state()
                
    
    def _close_map(self):
        global PROCESS_THREAD
        if PROCESS_THREAD is not None and PROCESS_THREAD.is_alive():
            SHUTDOWN_FLAG.value = 1
            print("追跡終了")
            time.sleep(2.0)
            
        self.web_server.log_event("命令受理: 地図作成終了")
        try:
            self.recording_map = False
            close_fiducial_loops = True
            close_odometry_loops = True
            self.rcl._auto_close_loops(close_fiducial_loops, close_odometry_loops)
            time.sleep(20)
            self.web_server.log_event("地図作成終了: ループクローズを実行しました")
        except Exception as e:
            self.web_server.log_event(f"エラー: {e}")
            print(f"Error occurred: {e}")
            return

    def _save_map(self):
        global PROCESS_THREAD
        if PROCESS_THREAD is not None and PROCESS_THREAD.is_alive():
            SHUTDOWN_FLAG.value = 1
            print("追跡終了")
            time.sleep(2.0)
            
        try:
            close_fiducial_loops = True
            close_odometry_loops = True
            self.rcl._auto_close_loops(close_fiducial_loops, close_odometry_loops)
            time.sleep(20)
            self.web_server.log_event("地図作成終了: ループクローズを実行しました")
            self.web_server.log_event("命令受理: 地図保存")
            print("地図を保存します")
            self.rcl._download_full_graph()
            self.gcl._list_graph_waypoint_and_edge_ids()
            self.web_server.log_event("地図保存: 地図をダウンロードしました")
            print("-" * 80)
            
        except Exception as e:
            self.web_server.log_event(f"エラー: {e}")
            print(f"Error occurred: {e}")
            return
        finally:
            # self.thread_running = False
            self.web_server.update_robot_state()
            
    def _map_download(self):
        self.web_server.log_event("命令受理: 地図ダウンロード")
        try:
            if self.recording_map == True:
                print("地図作成中はDLできません")
                print("-" * 80)
                return
            else:
                print("地図をダウンロードします")
                # engine.say("downloading the map made")
                # engine.runAndWait()
                # voice_output("downloading the map made")
                
                time.sleep(2.0)

                close_fiducial_loops = True
                close_odometry_loops = True
                self.rcl._auto_close_loops(close_fiducial_loops, close_odometry_loops)
                self.web_server.log_event("地図ダウンロード: ループクローズを実行しました")

                # engine.runAndWait()
                # print("thread")
                # マップのクロージング
                
                # print("地図をアップロードします")
                # engine.say("uploading the map to host PC")
                # # engine.runAndWait()
                self.gcl._upload_graph_and_snapshots()
                # print("地図を作成順に並び替えます")
                self.gcl._list_graph_waypoint_and_edge_ids()
                self.web_server.log_event("地図ダウンロード: 地図をアップロードしました")
                # print("-" * 80)
                
                # engine.say("complete downloading the map")
                # engine.runAndWait()
                # voice_output("complete downloading the map")
                # self._spot_pose_bow()
                print("-" * 80)
                
        except Exception as e:
            print(f"Error occurred: {e}")
            self.web_server.log_event(f"エラー: {e}")
            return
        
        finally:
            self.thread_running = False
        

    def _initialize(self):
        # engine.say("initialize localize position")
        # engine.runAndWait()
        # voice_output("initialize localize position")
        self.web_server.log_event("命令受理: 初期化")
        time.sleep(1.5)
        try:
            self.gcl._set_initial_localization_fiducial()
            waypoint = self.gcl._get_localization_id()
            print("waypoint", waypoint)
            self.web_server.log_event("QRコードでの初期化に成功しました")
        except:
            try:
                self.gcl._set_initial_localization_waypoint()
            except:
                print("QRコードでの初期化に失敗しました")
                print("waypointでの初期化に失敗しました")
                self.web_server.log_event("QRコードでの初期化に失敗しました")
                self.web_server.log_event("waypointでの初期化に失敗しました")
                print("-" * 80)
                return
        # self.initialize = True
        # print("地図をダウンロードします")
        # engine.say("initialize position")
        # engine.runAndWait()
        
        finally:
            self.thread_running = False
        
    def _sit_down(self):
        try:
            if self.recording_map == True:
                print("地図作成中には座れません")
                print("-" * 80)
                return
            
            self.web_server.log_event("命令受理: 座る")
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
        
    def _stand_up(self):
        try:
            if self.recording_map == True:
                print("地図作成中には立ち上がれません")
                print("-" * 80)
                return

            self.web_server.log_event("命令受理: 立つ")
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
            
            
    def _map_upload_and_sort(self):
        self.web_server.log_event("命令受理: 地図アップロード")
        try:
            if self.recording_map == True:
                print("地図作成中はアップロードできません")
                print("-" * 80)
                return
            else:
                # engine.say("uploading the map to host PC")
                # engine.runAndWait()
                # voice?_output("uploading the map from host PC")
                print("地図の最適化")   
                close_fiducial_loops = True
                close_odometry_loops = True
                self.rcl._auto_close_loops(close_fiducial_loops, close_odometry_loops)
                self.web_server.log_event("地図アップロード: ループクローズを実行しました")

                print("地図をアップロードします")
                self.gcl._upload_graph_and_snapshots()
                print("地図を作成順に並び替えます")
                self.gcl._list_graph_waypoint_and_edge_ids()

                self.web_server.log_event("地図アップロード: 地図をアップロードしました")

                # voice?_output("complete uploading the map")
                print("-" * 80)
                
                # self._spot_pose_bow()
        except Exception as e:
            print(f"Error occurred: {e}")
            self.web_server.log_event(f"エラー: {e}")
            return
        
        finally:
            self.thread_running = False
        
        
        
    def _detect_and_follow(self):
        global PROCESS_THREAD, SHUTDOWN_FLAG
        try:
            if PROCESS_THREAD is None or not PROCESS_THREAD.is_alive():
                self.web_server.log_event("命令受理: 追跡開始")
                print("追跡開始")
                # SHUTDOWN_FLAG.value = 0
                self.tracking.SHUTDOWN_FLAG.value = 0
                PROCESS_THREAD = threading.Thread(target=self.tracking.process_image_thread, args=(self.robot_state_client, self.robot_command_client), daemon=True)
                PROCESS_THREAD.start()
                PROCESS_THREAD.join()
                self.web_server.update_robot_state()
            else:
                print("既に追跡中です")
        except Exception as e:
            self.web_server.log_event(f"エラー: {e}")
            print(f"Error occurred: {e}")
            return
        finally:
            self.thread_running = False
            
    def _stop_follow(self):
        global PROCESS_THREAD
        if PROCESS_THREAD is not None and PROCESS_THREAD.is_alive():
            SHUTDOWN_FLAG.value = 1
            self.web_server.log_event("命令受理: 追跡停止")
            print("追跡終了")
        # ...existing処理...
        self.web_server.log_event("命令完了: 追跡停止完了")
        print("-" * 80)
            
                
            
    def _create_waypoint(self):
        global PROCESS_THREAD
        if self.recording_map == True:
            self.rcl._create_default_waypoint()
            self.web_server.log_event("命令受理: ウェイポイント作成")
            # engine.say("create waypoint")
            # engine.runAndWait()
            # voice_output("create waypoint")
        else:
            print("地図作成中ではありません")
            # return

        print("-" * 80)

        self.thread_running = False
        
        
    def _navigate_to_first_position(self):
        global PROCESS_THREAD
        self.web_server.log_event("命令受理: 最初の位置に戻る")
        try:
            if self.recording_map == True:
                print("地図作成中は戻れません")
                print("-" * 80)
                return
            
            elif self.gcl._waypoint_all_id == None:
                print("マップの最適化を行ってください")
                print("-" * 80)
                return
            
            elif PROCESS_THREAD is not None and PROCESS_THREAD.is_alive():
                # engine.say("It's running followings")
                # engine.runAndWait()
                # voice_output("It's running followings")
                time.sleep(1.0)
                return
            
            else:
                print("最初の位置に戻ります")
                # engine.say("navigate to first position")
                # engine.runAndWait()
                # voice_output("navigate to first position")
                # 初期化されていない場合（プログラム起動時）には初期化を行うようにする
                if self.initialize == False:
                    try:
                        self.gcl._set_initial_localization_fiducial()
                    except:
                        try:
                            self.gcl._set_initial_localization_waypoint()
                        except:
                            print("QRコードでの初期化に失敗しました")
                            print("waypointでの初期化に失敗しました")
                            print("-" * 80)
                            return
                    self.initialize = True
                position = self.gcl._waypoint_all_id[0]
                print(f"First waypoint id: {position}")
                self.web_server.log_event(f"最初の位置に戻ります: {position}")
                self.web_server.update_robot_state(initializing=True)
                self.gcl._navigate_first_waypoint(position)
                self.web_server.update_robot_state()
                # 最初の地点に戻るたびに初期化を行う
                if self.initialize == True:
                    try:
                        self.gcl._set_initial_localization_fiducial()
                    except:
                        print("初期化に失敗しました")
                        print("-" * 80)
                        return
                print("-" * 80)
        except Exception as e:
            print(f"Error occurred: {e}")
            self.web_server.log_event(f"エラー: {e}")
            return
        
        finally:
            self.thread_running = False
            
            
    def _navigate_to_last_position(self):
        global PROCESS_THREAD
        # while True:
        #     try:
        #         engine.say("go back previous position")
        #         engine.runAndWait()
        #         break  # 成功したらループを終了
        #     except Exception as e:
        #         print(f"Error occurred: {e}")
        #         time.sleep(0.5)  # 少し待ってから再試行
        # voice_output("go back previous position")

        self.web_server.log_event("命令受理: 最後の位置に戻る")
        try:
            self.gcl._set_initial_localization_fiducial()
        except Exception as e:
            print(f"Error occurred: {e}")
        
        try:
            current_waypoint_id, current_waypoint_name, current  = self.gcl._get_localication_id()
            # self.before_unique_position_id = current_waypoint_id
            # self.before_unique_position = current
            if current_waypoint_id == None or current_waypoint_name == None or current == None:
                return

            # 向かう先のwaypointを取得
            if self.before_unique_position_id == None or self.before_unique_position == None:
                return
            
            print(f"Current waypoint id: {current_waypoint_id}, Goal waypoint id: {self.before_unique_position_id}")
            
            if current == self.before_unique_position:
                print("同じwaypointです")
                self.web_server.log_event("同じwaypointにつき、移動しません")
                return
            
            print("目的地に向かいます")

            self.web_server.log_event("")
            self.web_server.update_robot_state(returning=True)
            self.gcl._navigate_route_to_user_waypoint(current, current_waypoint_id, self.before_unique_position, self.before_unique_position_id)
            self.web_server.update_robot_state()
        except Exception as e:
            print(f"Error occurred: {e}")
            self.web_server.log_event(f"エラー: {e}")
            return
        
        finally:
            self.thread_running = False
            
    def _navigate_route_all(self):
        global PROCESS_THREAD
        try:
            if self.recording_map == True:
                print("地図作成中にその操作はできません")
                print("-" * 80)
                return
            else:
                try:
                    self.gcl._set_initial_localization_fiducial()
                except:
                    try:
                        self.gcl._set_initial_localization_waypoint()
                    except:
                        print("QRコードでの初期化に失敗しました")
                        print("waypointでの初期化に失敗しました")
                        print("-" * 80)
                        return
            
                print("ルートを全て歩きます")
                # threadで作成することで、他の操作を行えるようにする
                # autowalk_thread = threading.Thread(target=self.gcl._navigate_route, daemon=True)
                # autowalk_thread.start()
                self.gcl._navigate_route()
                
        except Exception as e:
            print(f"Error occurred: {e}")
            return
        
        finally:
            self.thread_running = False
            
            
    def _navigate_to_unique_position_thinning(self):
        global PROCESS_THREAD
        
        if PROCESS_THREAD is not None and PROCESS_THREAD.is_alive():
            # engine.say("It's running followings")
            # engine.runAndWait()
            # voice_output("It's running followings")
            time.sleep(1.0)
            return
        time.sleep(1.0)

        self.web_server.log_event("命令受理: 保存場所に戻る")
        try:
            self.gcl._set_initial_localization_fiducial()
        except Exception as e:
            print(f"Error occurred: {e}")
        try:
            current_waypoint_id, current_waypoint_name, current  = self.gcl._get_localication_id()
            self.before_unique_position_id = current_waypoint_id
            self.before_unique_position = current
            if current_waypoint_id == None or current_waypoint_name == None or current == None:
                return

            # 向かう先のwaypointを取得
            goal_waypoint_name, goal_waypoint_id, goal = self.gcl._get_user_waypoint()
            if goal_waypoint_id == None or goal_waypoint_name == None or goal == None:
                return
            
            print(f"Current waypoint id: {current_waypoint_id}, Goal waypoint id: {goal_waypoint_id}")
            
            if current == goal:
                print("同じwaypointです")
                self.web_server.log_event("同じwaypointにつき、移動しません")
                return
            
            print("目的地に向かいます")
            

            self.web_server.update_robot_state(going_save=True)
            self.gcl._navigate_route_to_user_waypoint(current, current_waypoint_id, goal, goal_waypoint_id)
            self.web_server.update_robot_state()

            self._spot_thinning_pose()
            
            time.sleep(1.0)
            
            # 目的地到着
            # self._spot_pose_bow()
            
            # self.gcl._navigate_route_to_user_waypoint(current, current_waypoint_id, goal, goal_waypoint_id)
        except Exception as e:
            # engine.say("I can't navigate to the place")
            # engine.runAndWait()
            # voice_output("I can't navigate to the place")
            print(f"Error occurred: {e}")
            self.web_server.log_event(f"エラー: {e}")
            return
        
        finally:
            self.thread_running = False
            
    def _navigate_to_unique_position(self):
        global PROCESS_THREAD
        
        if PROCESS_THREAD is not None and PROCESS_THREAD.is_alive():
            # engine.say("It's running followings")
            # engine.runAndWait()
            # voice_output("It's running followings")
            time.sleep(1.0)
            return
        time.sleep(1.0)

        self.web_server.log_event("命令受理: 保存場所に戻る")
        try:
            self.gcl._set_initial_localization_fiducial()
        except Exception as e:
            print(f"Error occurred: {e}")
        try:
            current_waypoint_id, current_waypoint_name, current  = self.gcl._get_localication_id()
            self.before_unique_position_id = current_waypoint_id
            self.before_unique_position = current
            if current_waypoint_id == None or current_waypoint_name == None or current == None:
                return

            # 向かう先のwaypointを取得
            goal_waypoint_name, goal_waypoint_id, goal = self.gcl._get_user_waypoint()
            if goal_waypoint_id == None or goal_waypoint_name == None or goal == None:
                return
            
            print(f"Current waypoint id: {current_waypoint_id}, Goal waypoint id: {goal_waypoint_id}")
            
            if current == goal:
                print("同じwaypointです")
                self.web_server.log_event("同じwaypointにつき、移動しません")
                return
            
            print("目的地に向かいます")


            self.web_server.update_robot_state(going_save=True)
            self.gcl._navigate_route_to_user_waypoint(current, current_waypoint_id, goal, goal_waypoint_id)
            self.web_server.update_robot_state()

            # self._spot_bow_pose()
            
            time.sleep(1.0)
            
            # 目的地到着
            # self._spot_pose_bow()
            
            # self.gcl._navigate_route_to_user_waypoint(current, current_waypoint_id, goal, goal_waypoint_id)
        except Exception as e:
            # engine.say("I can't navigate to the place")
            # engine.runAndWait()
            # voice_output("I can't navigate to the place")
            print(f"Error occurred: {e}")
            self.web_server.log_event(f"エラー: {e}")
            return
        
        finally:
            self.thread_running = False
            
    def _spot_thinning(self):
        self.web_server.log_event("命令受理: 摘果命令受理")

        self._navigate_to_unique_position_thinning()
        time.sleep(1.0)
        self._navigate_to_last_position()

    def _spot_docking(self):
        self.web_server.log_event("命令受理: ドッキング")
        dock_id = 520  # ドッキングIDを指定
        blocking_dock_robot(self.robot, dock_id)
        time.sleep(1.0)

    def _spot_undocking(self):
        self.web_server.log_event("命令受理: ドッキング解除")
        resp = self.robot.power_on()
        dock_id = get_dock_id(self.robot)
        if dock_id is None:
            print("ドッキングされていません")
            self.web_server.log_event("ドッキングされていません")
        blocking_undock(self.robot)

    def _spot_find_dock(self):
        self.web_server.log_event("命令受理: ドックを探す")
        # self._navigate_to_unique_position()
        self._navigate_to_first_position()
        time.sleep(.5)
        dock_id = 520
        blocking_go_to_prep_pose(self.robot, dock_id)
        time.sleep(.5)
        blocking_dock_robot(self.robot, dock_id)
        print("docking success")
        self.web_server.log_event("ドッキング成功")
        
    def _spot_memory_truck(self):
        # self.web_server.log_event("命令受理: トラックの位置を記憶")
        # self.rcl._start_recording()
        self._make_map()
        time.sleep(.5)
        self.rcl._create_default_waypoint()
        time.sleep(.5)
        # self.rcl._stop_recording()
        self._stop_making_map_downloading()
        
    def _spot_finish_dock(self):
        self.web_server.log_event("命令受理: ドック解除し、舞台に戻る")
        resp = self.robot.power_on()
        dock_id = get_dock_id(self.robot)
        if dock_id is None:
            print("ドッキングされていません")
            self.web_server.log_event("ドッキングされていません")
        blocking_undock(self.robot)
        time.sleep(1.0)
        self._navigate_to_unique_position()

    def _spot_squat_pose(self):

        self.web_server.log_event("命令受理: スクワット")
        robot_state = self.robot_state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
        # Specify a trajectory to shift the body forward followed by looking down, then return to nominal.
        # Define times (in seconds) for each point in the trajectory.
        t1 = 0.5
        t2 = 2.0
        t3 = 3.0

        # Specify the poses as transformations to the cached flat_body pose.
        flat_body_T_pose1 = math_helpers.SE3Pose(x=0, y=0, z=0, rot=math_helpers.Quat())
        flat_body_T_pose2 = math_helpers.SE3Pose(
            x=0.0, y=0, z=-0.3, rot=math_helpers.Quat())
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
        
    def _spot_squat(self):

        self.web_server.log_event("命令受理: しゃがむ")
        robot_state = self.robot_state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                         ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
        # Specify a trajectory to shift the body forward followed by looking down, then return to nominal.
        # Define times (in seconds) for each point in the trajectory.
        t1 = 0.5
        t2 = 2.0

        # Specify the poses as transformations to the cached flat_body pose.
        flat_body_T_pose1 = math_helpers.SE3Pose(x=0, y=0, z=0, rot=math_helpers.Quat())
        flat_body_T_pose2 = math_helpers.SE3Pose(
            x=0.0, y=0, z=-0.5, rot=math_helpers.Quat())

        # Build the points in the trajectory.
        traj_point1 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body * flat_body_T_pose1).to_proto(),
            time_since_reference=seconds_to_duration(t1))
        traj_point2 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body * flat_body_T_pose2).to_proto(),
            time_since_reference=seconds_to_duration(t2))

        # Build the trajectory proto by combining the points.
        traj = trajectory_pb2.SE3Trajectory(points=[traj_point1, traj_point2])

        # Build a custom mobility params to specify absolute body control.
        body_control = spot_command_pb2.BodyControlParams(
            body_pose=spot_command_pb2.BodyControlParams.BodyPose(root_frame_name=ODOM_FRAME_NAME,
                                                                  base_offset_rt_root=traj))
        blocking_stand(self.robot_command_client, timeout_sec=5,
                       params=spot_command_pb2.MobilityParams(body_control=body_control))
        
        time.sleep(2.5)
        self.thread_running = False
        
        
    # def _spot_rotate(self):

    #     self.web_server.log_event("命令受理: 回転")
    #     robot_state = self.robot_state_client.get_robot_state()
    #     mobility_params = create_mobility_params(0.3, 0.5)
    #     tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(0, 0, 2* math.pi,
    #                             robot_state.kinematic_state.transforms_snapshot, mobility_params)
    #     end_time = 2.0
    #     if tag_cmd is not None:
    #         print('executing command')
    #         self.robot_command_client.robot_command(lease=None, command=tag_cmd,
    #                                                 end_time_secs=time.time() + end_time)

    def _spot_rotate(self):
        self.web_server.log_event("命令受理: 回転")
        
        rotation_steps = 2
        rotation_per_step = math.pi  # 90度
        
        for i in range(rotation_steps):
            robot_state = self.robot_state_client.get_robot_state()
            mobility_params = create_mobility_params(0.3, 0.5)
            tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(
                0, 0, rotation_per_step,
                robot_state.kinematic_state.transforms_snapshot, mobility_params)

            end_time = 6.3
            if tag_cmd is not None:
                print(f'executing rotation step {i+1}/{rotation_steps}')
                self.robot_command_client.robot_command(lease=None, command=tag_cmd,
                                                    end_time_secs=time.time() + end_time)
                time.sleep(end_time + 0.5)  # 各ステップの完了を待つ
                
    def _spot_rotate_90(self):
        self.web_server.log_event("命令受理: 回転")
        robot_state = self.robot_state_client.get_robot_state()
        mobility_params = create_mobility_params(0.3, 0.5)
        tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(0, 0, math.pi/2,
                                robot_state.kinematic_state.transforms_snapshot, mobility_params)
        end_time = 3.5
        if tag_cmd is not None:
            print('executing command')
            self.robot_command_client.robot_command(lease=None, command=tag_cmd,
                                                    end_time_secs=time.time() + end_time)
            
    def _go_to_front(self):
        global PROCESS_THREAD
        # engine.say("go to front")
        # engine.runAndWait()
        # voice_output("go to front")
        if PROCESS_THREAD is not None and PROCESS_THREAD.is_alive():
            SHUTDOWN_FLAG.value = 1
        print("前進します")
        self.web_server.log_event("命令受理: 前進")
        if self._is_spot_go_front == False:
            # まだ前進していないとき
            self._is_spot_go_front = True
            self.start_time = time.time()
            front_thread = threading.Thread(target=self._move_front_thread, daemon=True)
            front_thread.start()
            front_thread.join()
        if self._is_spot_go_front == True:
            # 前進中のとき
            # 進んでを2回言っても止まるようにはなってる
            self._is_spot_go_front = False
            return
        self.thread_running = False
            
    def _move_front_thread(self):
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
            if walk_end_time - self.start_time >= 10.0:
                self._is_spot_go_front = False
                return
            
    def _go_to_back(self):
        global PROCESS_THREAD
        # engine.say("go to back")
        # engine.runAndWait()
        # voice_output("go to back")
        if PROCESS_THREAD is not None and PROCESS_THREAD.is_alive():
            SHUTDOWN_FLAG.value = 1
        print("後退します")
        self.web_server.log_event("命令受理: 後退")
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
        
    def _move_back_thread(self):
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
            
    def _go_front_little(self):
        try:
            self.web_server.log_event("命令受理: 少し前進")
            robot_state = self.robot_state_client.get_robot_state()
            mobility_params = create_mobility_params(0.8, 0.5)
            tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(1.0, 0, 0,
                                    robot_state.kinematic_state.transforms_snapshot, mobility_params)
            end_time = 2.0
            if tag_cmd is not None:
                print('executing command')
                self.robot_command_client.robot_command(lease=None, command=tag_cmd,
                                                        end_time_secs=time.time() + end_time)
                
        except Exception as e:
            print(e)
            self.web_server.log_event(f"エラー: {e}")

        finally:
            self.thread_running = False

        return
    
    def _go_back_little(self):
        try:
            self.web_server.log_event("命令受理: 少し後退")
            robot_state = self.robot_state_client.get_robot_state()
            mobility_params = create_mobility_params(0.8, 0.5)
            tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(-1.0, 0, 0,
                                    robot_state.kinematic_state.transforms_snapshot, mobility_params)
            end_time = 2.0
            if tag_cmd is not None:
                print('executing command')
                self.robot_command_client.robot_command(lease=None, command=tag_cmd,
                                                        end_time_secs=time.time() + end_time)
                
        except Exception as e:
            self.web_server.log_event(f"エラー: {e}")
            print(e)
            
        finally:
            self.thread_running = False

        return

    
    def _go_right_little(self):
        try:
            self.web_server.log_event("命令受理: 少し右に移動")
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
            self.web_server.log_event(f"エラー: {e}")
            print(e)
            
        finally:
            self.thread_running = False

        return
    
    def _go_left_little(self):
        try:
            self.web_server.log_event("命令受理: 少し左に移動")
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
            self.web_server.log_event(f"エラー: {e}")
            print(e)
            
        finally:
            self.thread_running = False

        return
    
    def _spot_pose_bow(self):

        self.web_server.log_event("命令受理: お辞儀")
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
        
    def _spot_bow_pose(self):
        """Command the robot to bow."""
            
        robot_state = self.robot_state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                            ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
        # Specify a trajectory to shift the body forward followed by looking down, then return to nominal.
        # Define times (in seconds) for each point in the trajectory.
        """
        t1 -> t2: おじぎを始める
        """
        t1 = 0.0
        t2 = 0.5

        
        # おじぎの角度
        degree = 25
        w = math.cos(math.radians(degree/2))
        y = math.sin(math.radians(degree/2))

        # Specify the poses as transformations to the cached flat_body pose.
        flat_body_T_pose1 = math_helpers.SE3Pose(x=0, y=0, z=0, rot=math_helpers.Quat())
        flat_body_T_pose2 = math_helpers.SE3Pose(
            x=0.0, y=0, z=0, rot=math_helpers.Quat(w=w, x=0, y=y, z=0))

        # Build the points in the trajectory.
        traj_point1 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body * flat_body_T_pose1).to_proto(),
            time_since_reference=seconds_to_duration(t1))
        traj_point2 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body * flat_body_T_pose2).to_proto(),
            time_since_reference=seconds_to_duration(t2))
        # Build the trajectory proto by combining the points.
        traj = trajectory_pb2.SE3Trajectory(points=[traj_point1, traj_point2])

        # Build a custom mobility params to specify absolute body control.
        body_control = spot_command_pb2.BodyControlParams(
            body_pose=spot_command_pb2.BodyControlParams.BodyPose(root_frame_name=ODOM_FRAME_NAME,
                                                                    base_offset_rt_root=traj))
        blocking_stand(self.robot_command_client, timeout_sec=3,
                        params=spot_command_pb2.MobilityParams(body_control=body_control))
        
        time.sleep(0.6)
        GPIO.output(pin_out,1)
        """
        # 入力を待つ
        while True:
            print("continue to come back? -> y")
            if input() == "y":
                break
            time.sleep(0.1)
        """
        time.sleep(3.0)

        GPIO.output(pin_out,0)
        t3 = 0.0
        t4 = 0.5
        flat_body_T_pose3 = math_helpers.SE3Pose(
            x=0.0, y=0, z=0, rot=math_helpers.Quat(w=w, x=0, y=y, z=0))
        flat_body_T_pose4 = math_helpers.SE3Pose(x=0, y=0, z=0, rot=math_helpers.Quat())
        
        traj_point3 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body * flat_body_T_pose3).to_proto(),
            time_since_reference=seconds_to_duration(t3))
        traj_point4 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body * flat_body_T_pose4).to_proto(),
            time_since_reference=seconds_to_duration(t4))
        
        traj = trajectory_pb2.SE3Trajectory(points=[traj_point3, traj_point4])
        body_control = spot_command_pb2.BodyControlParams(
            body_pose=spot_command_pb2.BodyControlParams.BodyPose(root_frame_name=ODOM_FRAME_NAME,
                                                                    base_offset_rt_root=traj))
        blocking_stand(self.robot_command_client, timeout_sec=3,
                        params=spot_command_pb2.MobilityParams(body_control=body_control))
        
        time.sleep(3.0)
        
    def _spot_performance_bow(self):
        self.web_server.log_event("命令受理: パフォーマンス")
        end_time = 2.0 
    
        footprint_R_body = bosdyn.geometry.EulerZXY(yaw=0.4, roll=0.0, pitch=0.0)
        cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
        self.robot_command_client.robot_command(cmd)
        time.sleep(1.0)
        
        footprint_R_body = geometry.EulerZXY(yaw=0.4, roll=0.0, pitch=-0.3)
        cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
        self.robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + end_time)
        time.sleep(end_time)
        
        footprint_R_body = geometry.EulerZXY(yaw=0.4, roll=0.0, pitch=0.3)
        cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
        self.robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + end_time)
        time.sleep(end_time)
        
        footprint_R_body = geometry.EulerZXY(yaw=-0.4, roll=0.0, pitch=0.0)
        cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
        self.robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + end_time)
        time.sleep(1.0)
        
        footprint_R_body = geometry.EulerZXY(yaw=-0.4, roll=0.0, pitch=-0.3)
        cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
        self.robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + end_time)
        time.sleep(end_time)
        
        footprint_R_body = geometry.EulerZXY(yaw=-0.4, roll=0.0, pitch=0.3)
        cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
        self.robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + end_time)
        time.sleep(end_time)
        
        footprint_R_body = geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=0.0)
        cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
        self.robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + end_time)
        time.sleep(1.0)
        
        footprint_R_body = geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=-0.3)
        cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
        self.robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + end_time)
        time.sleep(end_time)
        
        footprint_R_body = geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=0.3)
        cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
        self.robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + 6.0)
        time.sleep(6.0)
        
        footprint_R_body = geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=0.0)
        cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
        self.robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + end_time)
        time.sleep(1.0)
    
    def _spot_thinning_pose(self):
        """Command the robot to thin."""
        robot_state = self.robot_state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                            ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
        # Specify a trajectory to shift the body forward followed by looking down, then return to nominal.
        # Define times (in seconds) for each point in the trajectory.
        """
        t1 -> t2: おじぎを始める
        """
        t1 = 0.0
        t2 = 0.5

        
        # おじぎの角度
        degree = 25
        w = math.cos(math.radians(degree/2))
        y = math.sin(math.radians(degree/2))

        # Specify the poses as transformations to the cached flat_body pose.
        flat_body_T_pose1 = math_helpers.SE3Pose(x=0, y=0, z=0, rot=math_helpers.Quat())
        flat_body_T_pose2 = math_helpers.SE3Pose(
            x=0.0, y=0, z=0, rot=math_helpers.Quat(w=w, x=0, y=y, z=0))

        # Build the points in the trajectory.
        traj_point1 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body * flat_body_T_pose1).to_proto(),
            time_since_reference=seconds_to_duration(t1))
        traj_point2 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body * flat_body_T_pose2).to_proto(),
            time_since_reference=seconds_to_duration(t2))
        # Build the trajectory proto by combining the points.
        traj = trajectory_pb2.SE3Trajectory(points=[traj_point1, traj_point2])

        # Build a custom mobility params to specify absolute body control.
        body_control = spot_command_pb2.BodyControlParams(
            body_pose=spot_command_pb2.BodyControlParams.BodyPose(root_frame_name=ODOM_FRAME_NAME,
                                                                    base_offset_rt_root=traj))
        blocking_stand(self.robot_command_client, timeout_sec=3,
                        params=spot_command_pb2.MobilityParams(body_control=body_control))
        
        time.sleep(0.6)
        GPIO.output(pin_out,1)
        """
        # 入力を待つ
        while True:
            print("continue to come back? -> y")
            if input() == "y":
                break
            time.sleep(0.1)
        """
        time.sleep(3.0)

        GPIO.output(pin_out,0)
        t3 = 0.0
        t4 = 0.5
        flat_body_T_pose3 = math_helpers.SE3Pose(
            x=0.0, y=0, z=0, rot=math_helpers.Quat(w=w, x=0, y=y, z=0))
        flat_body_T_pose4 = math_helpers.SE3Pose(x=0, y=0, z=0, rot=math_helpers.Quat())
        
        traj_point3 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body * flat_body_T_pose3).to_proto(),
            time_since_reference=seconds_to_duration(t3))
        traj_point4 = trajectory_pb2.SE3TrajectoryPoint(
            pose=(odom_T_flat_body * flat_body_T_pose4).to_proto(),
            time_since_reference=seconds_to_duration(t4))
        
        traj = trajectory_pb2.SE3Trajectory(points=[traj_point3, traj_point4])
        body_control = spot_command_pb2.BodyControlParams(
            body_pose=spot_command_pb2.BodyControlParams.BodyPose(root_frame_name=ODOM_FRAME_NAME,
                                                                    base_offset_rt_root=traj))
        blocking_stand(self.robot_command_client, timeout_sec=3,
                        params=spot_command_pb2.MobilityParams(body_control=body_control))
        
        time.sleep(3.0)
        
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

def audio_recog(cp, rec, web_server):
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
                            web_server.log_event(f"コマンド: {command} は実行できません")
                    else:
                        print(f"無効なコマンド: {command}")
                        web_server.log_event(f"無効なコマンド: {command}")
                        continue
                            
                else:
                    if current_text_cleaned:
                        print(f"コマンドが無視されました: '{current_text_cleaned}' は 'スポット' で始まる必要があります。")
                        web_server.log_event(f"コマンドが無視されました: '{current_text_cleaned}' は 'スポット' で始まる必要があります。")
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
        # LEASE = lease_client

        print("create client")

        robot_state_task = AsyncRobotState(robot_state_client)

        print('Detect and follow client connected.')

        lease = lease_client.take()
        lease_keep = LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True)
        # Power on the robot and stand it up
        dock_id = get_dock_id(robot)
        print(f'Dock ID: {dock_id}')
        if dock_id is None:
            print("no docking")
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
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pin_out, GPIO.OUT, initial=GPIO.LOW)

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
        # sd.RawInputStream(samplerate=16000, blocksize=8000, device=None, channels=1, callback=callback)

        """graphnavに関する部分"""
        print("GraphNav setting")
        client_metadata = GraphNavRecordingServiceClient.make_client_metadata(
            session_name="", client_username="", client_id='RecordingClient',
            client_type='Python SDK')
        recording_command_line = RecordingInterface(robot, os.getcwd(), client_metadata,
                                                False)
        
        # graphnav_command_line = GraphNavInterface(robot, "downloaded_graph", False)
        
        # time.sleep(2)
        
        web_server = WebServer()

        cp = CommandProcess(robot, client_metadata, robot_state_client, robot_command_client, web_server)


        web_server.set_cp(cp)
        web_server.set_lease(lease_client)
        app_thread = threading.Thread(
            target=web_server.app.run, 
            args=("0.0.0.0", 5000, False),
            daemon=True
        )
        app_thread.start()

        # 修正: 固定の16000ではなく、動的に取得したSAMPLE_RATEを使用
        print(SAMPLE_RATE)
        rec = KaldiRecognizer(model, SAMPLE_RATE)
        
        print("keyboad setting")
        keyboard_queue = queue.Queue()
        key_capture_thread = threading.Thread(target=capture_keyboard_input, args=(keyboard_queue,), daemon=True)
        key_capture_thread.start()
        
        key_input_thread = threading.Thread(target=keyboard_input_thread, args=(keyboard_queue, cp), daemon=True)
        key_input_thread.start()
        
        audio_thread = threading.Thread(target=audio_recog, args=(cp, rec, web_server), daemon=True)
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
        web_server.log_event("program will be finished")

    except Exception as exc:  # pylint: disable=broad-except
        LOGGER.error('Spot jetson threw an exception: %s', exc)
        return False
    
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
