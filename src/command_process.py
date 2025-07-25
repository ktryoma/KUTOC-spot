import math
import threading
import os
import time


import numpy as np

import src.graph_nav_util as graph_nav_util
import src.dijkstra as dijkstra
from src.commands_config import get_command_dict, get_keyboard_commands_dict
from src.graph_nav import GraphNavInterface, RecordingInterface

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

import Jetson.GPIO as GPIO

class CommandProcess():
    
    def __init__(self, robot, client_meta_data, robot_state_client, robot_command_client):
        
        
        """
        rcl ... RecordingInterfaceのインスタンス
        gcl ... GraphNavInterfaceのインスタンス
        
        """
        
        self.rcl = RecordingInterface(robot, os.getcwd(), client_meta_data, False)
        self.gcl = GraphNavInterface(robot, "downloaded_graph", False)

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
        log_event("音声認識をリセットします")
        reset_audio_recognition()
        return False  # プログラム終了にはつながらない
            
    def _make_map(self):
        try:
            if self.recording_map == True:
                log_event("地図作成中: 既に地図作成中です")
                print("地図作成中です")
                print("-" * 80)
                return
            else:
                log_event("命令受理: 地図作成開始")
                print("地図を作成開始します")
                self.rcl._start_recording()
                self.recording_map = True
                update_robot_state(map_recording=True)
                print("-" * 80)
        except Exception as e:
            log_event(f"エラー: {e}")
            update_robot_state(map_recording=False)
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
            
        log_event("命令受理: 地図作成終了")
        if self.recording_map == True:
            try:
                print("地図作成を終了します")
                self.rcl._stop_recording()
                log_event("地図作成終了: 地図作成を終了しました")
                print("recording stop")
                time.sleep(10.0)
                self.recording_map = False
                close_fiducial_loops = True
                close_odometry_loops = True
                self.rcl._auto_close_loops(close_fiducial_loops, close_odometry_loops)
                log_event("地図作成終了: ループクローズを実行しました")
                time.sleep(5)
                print("-" * 80)
                self.rcl._download_full_graph()
                self.gcl._list_graph_waypoint_and_edge_ids()
                log_event("地図作成終了: 地図をアップロードしました")
                print("-" * 80)
            except Exception as e:
                log_event(f"エラー: {e}")
                print(f"Error occurred: {e}")
                return
            finally:
                self.thread_running = False
                update_robot_state()
                
    
    def _close_map(self):
        global PROCESS_THREAD
        if PROCESS_THREAD is not None and PROCESS_THREAD.is_alive():
            SHUTDOWN_FLAG.value = 1
            print("追跡終了")
            time.sleep(2.0)
            
        log_event("命令受理: 地図作成終了")
        try:
            self.recording_map = False
            close_fiducial_loops = True
            close_odometry_loops = True
            self.rcl._auto_close_loops(close_fiducial_loops, close_odometry_loops)
            time.sleep(20)
            log_event("地図作成終了: ループクローズを実行しました")
        except Exception as e:
            log_event(f"エラー: {e}")
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
            log_event("地図作成終了: ループクローズを実行しました")
            log_event("命令受理: 地図保存")
            print("地図を保存します")
            self.rcl._download_full_graph()
            self.gcl._list_graph_waypoint_and_edge_ids()
            log_event("地図保存: 地図をダウンロードしました")
            print("-" * 80)
            
        except Exception as e:
            log_event(f"エラー: {e}")
            print(f"Error occurred: {e}")
            return
        finally:
            # self.thread_running = False
            update_robot_state()
            
    def _map_download(self):
        log_event("命令受理: 地図ダウンロード")
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
                log_event("地図ダウンロード: ループクローズを実行しました")

                # engine.runAndWait()
                # print("thread")
                # マップのクロージング
                
                # print("地図をアップロードします")
                # engine.say("uploading the map to host PC")
                # # engine.runAndWait()
                self.gcl._upload_graph_and_snapshots()
                # print("地図を作成順に並び替えます")
                self.gcl._list_graph_waypoint_and_edge_ids()
                log_event("地図ダウンロード: 地図をアップロードしました")
                # print("-" * 80)
                
                # engine.say("complete downloading the map")
                # engine.runAndWait()
                # voice_output("complete downloading the map")
                # self._spot_pose_bow()
                print("-" * 80)
                
        except Exception as e:
            print(f"Error occurred: {e}")
            log_event(f"エラー: {e}")
            return
        
        finally:
            self.thread_running = False
        

    def _initialize(self):
        # engine.say("initialize localize position")
        # engine.runAndWait()
        # voice_output("initialize localize position")
        log_event("命令受理: 初期化")
        time.sleep(1.5)
        try:
            self.gcl._set_initial_localization_fiducial()
            waypoint = self.gcl._get_localization_id()
            print("waypoint", waypoint)
            log_event("QRコードでの初期化に成功しました")
        except:
            try:
                self.gcl._set_initial_localization_waypoint()
            except:
                print("QRコードでの初期化に失敗しました")
                print("waypointでの初期化に失敗しました")
                log_event("QRコードでの初期化に失敗しました")
                log_event("waypointでの初期化に失敗しました")
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
        
    def _stand_up(self):
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
            
            
    def _map_upload_and_sort(self):
        log_event("命令受理: 地図アップロード")
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
                log_event("地図アップロード: ループクローズを実行しました")
                
                print("地図をアップロードします")
                self.gcl._upload_graph_and_snapshots()
                print("地図を作成順に並び替えます")
                self.gcl._list_graph_waypoint_and_edge_ids()
                
                log_event("地図アップロード: 地図をアップロードしました")
                
                # voice?_output("complete uploading the map")
                print("-" * 80)
                
                # self._spot_pose_bow()
        except Exception as e:
            print(f"Error occurred: {e}")
            log_event(f"エラー: {e}")
            return
        
        finally:
            self.thread_running = False
        
        
        
    def _detect_and_follow(self):
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
            
    def _stop_follow(self):
        global PROCESS_THREAD
        if PROCESS_THREAD is not None and PROCESS_THREAD.is_alive():
            SHUTDOWN_FLAG.value = 1
            log_event("命令受理: 追跡停止")
            print("追跡終了")
        # ...existing処理...
        log_event("命令完了: 追跡停止完了")
        print("-" * 80)
            
                
            
    def _create_waypoint(self):
        global PROCESS_THREAD
        if self.recording_map == True:
            self.rcl._create_default_waypoint()
            log_event("命令受理: ウェイポイント作成")
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
        log_event("命令受理: 最初の位置に戻る")
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
                log_event(f"最初の位置に戻ります: {position}")
                update_robot_state(initializing=True)
                self.gcl._navigate_first_waypoint(position)
                update_robot_state()
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
            log_event(f"エラー: {e}")
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
        
        log_event("命令受理: 最後の位置に戻る")
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
                log_event("同じwaypointにつき、移動しません")
                return
            
            print("目的地に向かいます")
            
            log_event("")
            update_robot_state(returning=True)
            self.gcl._navigate_route_to_user_waypoint(current, current_waypoint_id, self.before_unique_position, self.before_unique_position_id)
            update_robot_state()
        except Exception as e:
            print(f"Error occurred: {e}")
            log_event(f"エラー: {e}")
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
        
        log_event("命令受理: 保存場所に戻る")
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
                log_event("同じwaypointにつき、移動しません")
                return
            
            print("目的地に向かいます")
            
            
            update_robot_state(going_save=True)
            self.gcl._navigate_route_to_user_waypoint(current, current_waypoint_id, goal, goal_waypoint_id)
            update_robot_state()
            
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
            log_event(f"エラー: {e}")
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
        
        log_event("命令受理: 保存場所に戻る")
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
                log_event("同じwaypointにつき、移動しません")
                return
            
            print("目的地に向かいます")
            
            
            update_robot_state(going_save=True)
            self.gcl._navigate_route_to_user_waypoint(current, current_waypoint_id, goal, goal_waypoint_id)
            update_robot_state()
            
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
            log_event(f"エラー: {e}")
            return
        
        finally:
            self.thread_running = False
            
    def _spot_thinning(self):
        log_event("命令受理: 摘果命令受理")

        self._navigate_to_unique_position_thinning()
        time.sleep(1.0)
        self._navigate_to_last_position()
            
    def _spot_squat_pose(self):
        
        log_event("命令受理: しゃがむ")
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
        
    def _go_to_front(self):
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
            if walk_end_time - self.start_time >= 15.0:
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
    
    def _go_back_little(self):
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

    
    def _go_right_little(self):
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
    
    def _go_left_little(self):
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