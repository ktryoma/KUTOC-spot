import os
import time
import math
from math import sqrt

import src.graph_nav_util as graph_nav_util
import src.dijkstra as dijkstra

import bosdyn.client
from bosdyn.client import math_helpers, ResponseError
from bosdyn.api import geometry_pb2 as geo
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2

from bosdyn.client.frame_helpers import (GROUND_PLANE_FRAME_NAME, VISION_FRAME_NAME, 
                                         get_odom_tform_body)

from bosdyn.client.robot_command import (CommandFailedError, CommandTimedOutError,
                                         RobotCommandBuilder, RobotCommandClient, 
                                         blocking_stand, blocking_sit, blocking_command)

from bosdyn.client.robot_state import RobotStateClient

from bosdyn.client.recording import GraphNavRecordingServiceClient
from google.protobuf import wrappers_pb2 as wrappers

from bosdyn.api.graph_nav import map_pb2, map_processing_pb2, recording_pb2, graph_nav_pb2, nav_pb2
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.map_processing import MapProcessingServiceClient
from bosdyn.client.power import PowerClient

class RecordingInterface(object):
    """Recording service command line interface."""

    def __init__(self, robot, download_filepath, client_metadata, use_gps=False, 
                 speed_limit=1.2):
        # Keep the robot instance and it's ID.
        self._robot = robot

        self.use_gps = use_gps

        # Force trigger timesync.
        self._robot.time_sync.wait_for_sync()

        # Filepath for the location to put the downloaded graph and snapshots.
        self._download_filepath = os.path.join(download_filepath, 'downloaded_graph')

        # Set up the recording service client.
        self._recording_client = self._robot.ensure_client(
            GraphNavRecordingServiceClient.default_service_name)

        # Create the recording environment.
        self._recording_environment = GraphNavRecordingServiceClient.make_recording_environment(
            waypoint_env=GraphNavRecordingServiceClient.make_waypoint_environment(
                client_metadata=client_metadata))

        # Set up the graph nav service client.
        self._graph_nav_client = robot.ensure_client(GraphNavClient.default_service_name)

        self._map_processing_client = robot.ensure_client(
            MapProcessingServiceClient.default_service_name)

        # Store the most recent knowledge of the state of the robot based on rpc calls.
        self._current_graph = None
        self._current_edges = dict()  #maps to_waypoint to list(from_waypoint)
        self._current_waypoint_snapshots = dict()  # maps id to waypoint snapshot
        self._current_edge_snapshots = dict()  # maps id to edge snapshot
        self._current_annotation_name_to_wp_id = dict()
        
        # いくつでもwaypointを作成できるようにカウントする
        self._count = 1

        self._speed_limit = speed_limit
        
        self._create_waypoint_name = ["pointA", "pointB", "pointC"]


    def should_we_start_recording(self):
        # Before starting to record, check the state of the GraphNav system.
        graph = self._graph_nav_client.download_graph()
        if graph is not None:
            # Check that the graph has waypoints. If it does, then we need to be localized to the graph
            # before starting to record
            if len(graph.waypoints) > 0:
                localization_state = self._graph_nav_client.get_localization_state()
                if not localization_state.localization.waypoint_id:
                    # Not localized to anything in the map. The best option is to clear the graph or
                    # attempt to localize to the current map.
                    # Returning false since the GraphNav system is not in the state it should be to
                    # begin recording.
                    return False
        # If there is no graph or there exists a graph that we are localized to, then it is fine to
        # start recording, so we return True.
        return True

    def _clear_map(self, *args):
        """Clear the state of the map on the robot, removing all waypoints and edges."""
        return self._graph_nav_client.clear_graph()

    def _start_recording(self, *args):
        """Start recording a map."""
        should_start_recording = self.should_we_start_recording()
        if not should_start_recording:
            print('The system is not in the proper state to start recording.'
                  'Try using the graph_nav_command_line to either clear the map or'
                  'attempt to localize to the map.')
            return
        try:
            status = self._recording_client.start_recording(
                recording_environment=self._recording_environment)
            print('Successfully started recording a map.')
        except Exception as err:
            print(f'Start recording failed: {err}')

    def _stop_recording(self, *args):
        """Stop or pause recording a map."""
        first_iter = True
        while True:
            try:
                status = self._recording_client.stop_recording()
                print('Successfully stopped recording a map.')
                break
            except bosdyn.client.recording.NotReadyYetError as err:
                # It is possible that we are not finished recording yet due to
                # background processing. Try again every 1 second.
                if first_iter:
                    print('Cleaning up recording...')
                first_iter = False
                time.sleep(1.0)
                continue
            except Exception as err:
                print(f'Stop recording failed: {err}')
                break

    def _get_recording_status(self, *args):
        """Get the recording service's status."""
        status = self._recording_client.get_record_status()
        if status.is_recording:
            print('The recording service is on.')
        else:
            print('The recording service is off.')

    def _create_default_waypoint(self, *args):
        """Create a default waypoint at the robot's current location."""
        resp = self._recording_client.create_waypoint(waypoint_name='created_user_waypoint' + str(self._count))
        if resp.status == recording_pb2.CreateWaypointResponse.STATUS_OK:
            print('Successfully created a waypoint.')
        else:
            print('Could not create a waypoint.')
            
    def _create_default_waypoint_odm(self, *args):
        """オドメトリを使ってwaypointを作成する"""
        resp = self._recording_client.create_waypoint(waypoint_name='odm_waypoint' + str(self._count))
        if resp.status == recording_pb2.CreateWaypointResponse.STATUS_OK:
            print('Successfully created a waypoint.')
            self._count += 1
        else:
            print('Could not create a waypoint.')

    def _download_full_graph(self, *args):
        """Download the graph and snapshots from the robot."""
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            print('Failed to download the graph.')
            return
        self._write_full_graph(graph)
        print(
            f'Graph downloaded with {len(graph.waypoints)} waypoints and {len(graph.edges)} edges')
        # Download the waypoint and edge snapshots.
        self._download_and_write_waypoint_snapshots(graph.waypoints)
        self._download_and_write_edge_snapshots(graph.edges)

    def _write_full_graph(self, graph):
        """Download the graph from robot to the specified, local filepath location."""
        graph_bytes = graph.SerializeToString()
        self._write_bytes(self._download_filepath, 'graph', graph_bytes)

    def _download_and_write_waypoint_snapshots(self, waypoints):
        """Download the waypoint snapshots from robot to the specified, local filepath location."""
        num_waypoint_snapshots_downloaded = 0
        for waypoint in waypoints:
            if len(waypoint.snapshot_id) == 0:
                continue
            try:
                waypoint_snapshot = self._graph_nav_client.download_waypoint_snapshot(
                    waypoint.snapshot_id)
            except Exception:
                # Failure in downloading waypoint snapshot. Continue to next snapshot.
                print(f'Failed to download waypoint snapshot: {waypoint.snapshot_id}')
                continue
            self._write_bytes(os.path.join(self._download_filepath, 'waypoint_snapshots'),
                              str(waypoint.snapshot_id), waypoint_snapshot.SerializeToString())
            num_waypoint_snapshots_downloaded += 1
            print(
                f'Downloaded {num_waypoint_snapshots_downloaded} of the total {len(waypoints)} waypoint snapshots.'
            )

    def _download_and_write_edge_snapshots(self, edges):
        """Download the edge snapshots from robot to the specified, local filepath location."""
        num_edge_snapshots_downloaded = 0
        num_to_download = 0
        for edge in edges:
            if len(edge.snapshot_id) == 0:
                continue
            num_to_download += 1
            try:
                edge_snapshot = self._graph_nav_client.download_edge_snapshot(edge.snapshot_id)
            except Exception:
                # Failure in downloading edge snapshot. Continue to next snapshot.
                print(f'Failed to download edge snapshot: {edge.snapshot_id}')
                continue
            self._write_bytes(os.path.join(self._download_filepath, 'edge_snapshots'),
                              str(edge.snapshot_id), edge_snapshot.SerializeToString())
            num_edge_snapshots_downloaded += 1
            print(
                f'Downloaded {num_edge_snapshots_downloaded} of the total {num_to_download} edge snapshots.'
            )

    def _write_bytes(self, filepath, filename, data):
        """Write data to a file."""
        os.makedirs(filepath, exist_ok=True)
        with open(os.path.join(filepath, filename), 'wb+') as f:
            f.write(data)
            f.close()

    def _update_graph_waypoint_and_edge_ids(self, do_print=False):
        # Download current graph
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            print('Empty graph.')
            return
        self._current_graph = graph

        localization_id = self._graph_nav_client.get_localization_state().localization.waypoint_id

        # Update and print waypoints and edges
        self._current_annotation_name_to_wp_id, self._current_edges = graph_nav_util.update_waypoints_and_edges(
            graph, localization_id, do_print)

    def _list_graph_waypoint_and_edge_ids(self, *args):
        """List the waypoint ids and edge ids of the graph currently on the robot."""
        self._update_graph_waypoint_and_edge_ids(do_print=True)

    def _create_new_edge(self, *args):
        """Create new edge between existing waypoints in map."""

        if len(args[0]) != 2:
            print('ERROR: Specify the two waypoints to connect (short code or annotation).')
            return

        self._update_graph_waypoint_and_edge_ids(do_print=False)

        from_id = graph_nav_util.find_unique_waypoint_id(args[0][0], self._current_graph,
                                                         self._current_annotation_name_to_wp_id)
        to_id = graph_nav_util.find_unique_waypoint_id(args[0][1], self._current_graph,
                                                       self._current_annotation_name_to_wp_id)

        print(f'Creating edge from {from_id} to {to_id}.')

        from_wp = self._get_waypoint(from_id)
        if from_wp is None:
            return

        to_wp = self._get_waypoint(to_id)
        if to_wp is None:
            return

        # Get edge transform based on kinematic odometry
        edge_transform = self._get_transform(from_wp, to_wp)

        # Define new edge
        new_edge = map_pb2.Edge()
        new_edge.id.from_waypoint = from_id
        new_edge.id.to_waypoint = to_id
        new_edge.from_tform_to.CopyFrom(edge_transform)

        print(f'edge transform = {new_edge.from_tform_to}')

        # Send request to add edge to map
        self._recording_client.create_edge(edge=new_edge)
        
    def _auto_close_loops(self, close_fiducial_loops, close_odometry_loops, *args):
        """Automatically find and close all loops in the graph."""
        response = self._map_processing_client.process_topology(
            params=map_processing_pb2.ProcessTopologyRequest.Params(
                do_fiducial_loop_closure=wrappers.BoolValue(value=close_fiducial_loops),
                do_odometry_loop_closure=wrappers.BoolValue(value=close_odometry_loops)),
            modify_map_on_server=True)
        print(f'Created {len(response.new_subgraph.edges)} new edge(s).')
        # log_event(f'Created {len(response.new_subgraph.edges)} new edge(s).')
    

class GraphNavInterface(object):
    """GraphNav service command line interface."""

    def __init__(self, robot, upload_path, use_gps=False):
        self._robot = robot
        self.use_gps = use_gps

        # Force trigger timesync.
        self._robot.time_sync.wait_for_sync()

        # Create robot state and command clients.
        self._robot_command_client = self._robot.ensure_client(
            RobotCommandClient.default_service_name)
        self._robot_state_client = self._robot.ensure_client(RobotStateClient.default_service_name)

        # Create the client for the Graph Nav main service.
        self._graph_nav_client = self._robot.ensure_client(GraphNavClient.default_service_name)

        # Create a power client for the robot.
        self._power_client = self._robot.ensure_client(PowerClient.default_service_name)

        # Boolean indicating the robot's power state.
        power_state = self._robot_state_client.get_robot_state().power_state
        self._started_powered_on = (power_state.motor_power_state == power_state.STATE_ON)
        self._powered_on = self._started_powered_on

        # Number of attempts to wait before trying to re-power on.
        self._max_attempts_to_wait = 50

        # Store the most recent knowledge of the state of the robot based on rpc calls.
        self._current_graph = None
        self._current_edges = dict()  #maps to_waypoint to list(from_waypoint)
        self._current_waypoint_snapshots = dict()  # maps id to waypoint snapshot
        self._current_edge_snapshots = dict()  # maps id to edge snapshot
        self._current_annotation_name_to_wp_id = dict()
        
        
        # waypointをidを格納する配列
        self._waypoint_all_id = []
        self._waypoint_all_name = []
        
        # 自動運転を制御するためのブール変数
        self._is_finished = True
        
        # マップはspotにアップロードされたか
        self._is_uploaded = False
        
        self._is_autodrive_cancel = False

        # Filepath for uploading a saved graph's and snapshots too.
        if upload_path[-1] == '/':
            self._upload_filepath = upload_path[:-1]
        else:
            self._upload_filepath = upload_path

        if self.use_gps:
            self._command_dictionary['g'] = self._navigate_to_gps_coords
            
    

    def _get_localization_state(self, *args):
        """Get the current localization and state of the robot."""
        state = self._graph_nav_client.get_localization_state(request_gps_state=self.use_gps)
        print(f'Got localization: \n{state.localization}')
        odom_tform_body = get_odom_tform_body(state.robot_kinematics.transforms_snapshot)
        print(f'Got robot state in kinematic odometry frame: \n{odom_tform_body}')
        if self.use_gps:
            print(f'GPS info:\n{state.gps}')

    def _set_initial_localization_fiducial(self, *args):
        """Trigger localization when near a fiducial."""
        robot_state = self._robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot).to_proto()
        # Create an empty instance for initial localization since we are asking it to localize
        # based on the nearest fiducial.
        localization = nav_pb2.Localization()
        self._graph_nav_client.set_localization(initial_guess_localization=localization,
                                                ko_tform_body=current_odom_tform_body)


    def _set_initial_localization_waypoint(self, *args):
        """Trigger localization to a waypoint."""
        # Take the first argument as the localization waypoint.
        if len(args) < 1:
            # If no waypoint id is given as input, then return without initializing.
            print('No waypoint specified to initialize to.')
            return
        destination_waypoint = graph_nav_util.find_unique_waypoint_id(
            args[0][0], self._current_graph, self._current_annotation_name_to_wp_id)
        if not destination_waypoint:
            # Failed to find the unique waypoint id.
            return

        robot_state = self._robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot).to_proto()
        # Create an initial localization to the specified waypoint as the identity.
        localization = nav_pb2.Localization()
        localization.waypoint_id = destination_waypoint
        localization.waypoint_tform_body.rotation.w = 1.0
        self._graph_nav_client.set_localization(
            initial_guess_localization=localization,
            # It's hard to get the pose perfect, search +/-20 deg and +/-20cm (0.2m).
            max_distance=0.2,
            max_yaw=20.0 * math.pi / 180.0,
            fiducial_init=graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NO_FIDUCIAL,
            ko_tform_body=current_odom_tform_body)
        
    def _get_localication_id(self):
        """現在、経路のどこにいるか（一番近くのwaypointを見つける）"""

        # Download current graph
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            print('Empty graph.')
            return None, None, None
        self._current_graph = graph

        localization_id = self._graph_nav_client.get_localization_state().localization.waypoint_id
        
        for i in range(len(self._waypoint_all_id)):
            if self._waypoint_all_id[i] == localization_id:
                # print(f"現在のwaypointは{self._waypoint_all_name[i]}です")
                localization_name = self._waypoint_all_name[i]
                break
        
        print("現在のwaypoint_idは", localization_id)
        
        return localization_id, localization_name, i

    def _list_graph_waypoint_and_edge_ids(self, *args):
        """List the waypoint ids and edge ids of the graph currently on the robot."""

        # Download current graph
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            print('Empty graph.')
            return
        self._current_graph = graph

        localization_id = self._graph_nav_client.get_localization_state().localization.waypoint_id

        # Update and print waypoints and edges
        # self._current_annotation_name_to_wp_id, self._current_edges = graph_nav_util.update_waypoints_and_edges(
        #     graph, localization_id)
        
        # 作成順に並び替える（みつき作）
        self._current_annotation_name_to_wp_id, self._current_edges, self._waypoint_all_id, self._waypoint_all_name = graph_nav_util.update_waypoints_and_edges(
            graph, localization_id)
        
        print("waypointをidを出力します")
        for i in range(len(self._waypoint_all_id)):
            print(self._waypoint_all_id[i])
            
        print("waypointを名前を出力します")
        for i in range(len(self._waypoint_all_name)):
            print(self._waypoint_all_name[i])
            
        self._is_uploaded = True
        
    
    def _get_user_waypoint(self):
        
        # 作成したウェイポイントを探す
        # 作成していないと見つからない
        # 一番最後に作成したwaypointを探すようにしているはず
        if self._is_uploaded == False:
            print("マップがアップロードされていません")
            return None, None, None
        max_id = -1  # 最大の数値部分を保持する変数
        max_waypoint = None  # 最大のウェイポイント名を保持する変数
        max_index = -1  # 最大のウェイポイントのインデックスを保持する変数

        for i in range(len(self._waypoint_all_id)):
            if "created_user_waypoint" in self._waypoint_all_name[i]:
                # ウェイポイント名から数字部分を抽出
                number_part = self._waypoint_all_name[i][len("created_user_waypoint"):]

                try:
                    number_value = int(number_part)  # 数値に変換
                    if number_value > max_id:  # 最大値を更新
                        max_id = number_value
                        max_waypoint = self._waypoint_all_name[i]
                        max_index = i
                except ValueError:
                    # 数値変換に失敗した場合は無視
                    continue

        if max_waypoint is not None:
            print(f"{max_waypoint}: {self._waypoint_all_id[max_index]}")
            return max_waypoint, self._waypoint_all_id[max_index], max_index

        print("ユーザが作成したウェイポイントが見つかりません")
        return None, None, None
            

    def _upload_graph_and_snapshots(self, *args):
        """Upload the graph and snapshots to the robot."""
        print('Loading the graph from disk into local storage...')
        with open(self._upload_filepath + '/graph', 'rb') as graph_file:
            # Load the graph from disk.
            data = graph_file.read()
            self._current_graph = map_pb2.Graph()
            self._current_graph.ParseFromString(data)
            print(
                f'Loaded graph has {len(self._current_graph.waypoints)} waypoints and {self._current_graph.edges} edges'
            )
        for waypoint in self._current_graph.waypoints:
            # Load the waypoint snapshots from disk.
            with open(f'{self._upload_filepath}/waypoint_snapshots/{waypoint.snapshot_id}',
                      'rb') as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                self._current_waypoint_snapshots[waypoint_snapshot.id] = waypoint_snapshot
        for edge in self._current_graph.edges:
            if len(edge.snapshot_id) == 0:
                continue
            # Load the edge snapshots from disk.
            with open(f'{self._upload_filepath}/edge_snapshots/{edge.snapshot_id}',
                      'rb') as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                edge_snapshot.ParseFromString(snapshot_file.read())
                self._current_edge_snapshots[edge_snapshot.id] = edge_snapshot
        # Upload the graph to the robot.
        print('Uploading the graph and snapshots to the robot...')
        true_if_empty = not len(self._current_graph.anchoring.anchors)
        response = self._graph_nav_client.upload_graph(graph=self._current_graph,
                                                       generate_new_anchoring=true_if_empty)
        # Upload the snapshots to the robot.
        for snapshot_id in response.unknown_waypoint_snapshot_ids:
            waypoint_snapshot = self._current_waypoint_snapshots[snapshot_id]
            self._graph_nav_client.upload_waypoint_snapshot(waypoint_snapshot)
            print(f'Uploaded {waypoint_snapshot.id}')
        for snapshot_id in response.unknown_edge_snapshot_ids:
            edge_snapshot = self._current_edge_snapshots[snapshot_id]
            self._graph_nav_client.upload_edge_snapshot(edge_snapshot)
            print(f'Uploaded {edge_snapshot.id}')

        # The upload is complete! Check that the robot is localized to the graph,
        # and if it is not, prompt the user to localize the robot before attempting
        # any navigation commands.
        localization_state = self._graph_nav_client.get_localization_state()
        if not localization_state.localization.waypoint_id:
            # The robot is not localized to the newly uploaded graph.
            print('\n')
            print(
                'Upload complete! The robot is currently not localized to the map; please localize'
                ' the robot using commands (2) or (3) before attempting a navigation command.')



    def _navigate_to(self, *args):
        """Navigate to a specific waypoint."""
        # Take the first argument as the destination waypoint.
        if len(args) < 1:
            # If no waypoint id is given as input, then return without requesting navigation.
            print('No waypoint provided as a destination for navigate to.')
            return

        destination_waypoint = graph_nav_util.find_unique_waypoint_id(
            args[0][0], self._current_graph, self._current_annotation_name_to_wp_id)
        if not destination_waypoint:
            # Failed to find the appropriate unique waypoint id for the navigation command.
            return
        if not self.toggle_power(should_power_on=True):
            print('Failed to power on the robot, and cannot complete navigate to request.')
            return

        nav_to_cmd_id = None
        # Navigate to the destination waypoint.
        self._is_finished = False
        while not self._is_finished:
            # Issue the navigation command about twice a second such that it is easy to terminate the
            # navigation command (with estop or killing the program).
            try:
                nav_to_cmd_id = self._graph_nav_client.navigate_to(destination_waypoint, 1.0,
                                                                   command_id=nav_to_cmd_id)
            except ResponseError as e:
                print(f'Error while navigating {e}')
                break
            time.sleep(.5)  # Sleep for half a second to allow for command execution.
            # Poll the robot for feedback to determine if the navigation command is complete. Then sit
            # the robot down once it is finished.
            self._is_finished = self._check_success(nav_to_cmd_id)

        # Power off the robot if appropriate.
        if self._powered_on and not self._started_powered_on:
            # Sit the robot down + power off after the navigation command is complete.
            self.toggle_power(should_power_on=False)
            
    def _navigate_first_waypoint(self, *args):
        # 移動先のwaypointを指定する
        destination_waypoint = args[0]
        print("destination_waypoint", destination_waypoint)
        
        if not destination_waypoint:
            print("waypointが記録されていない可能性があります")
            return
        
        nav_to_cmd_id = None
        # Navigate to the destination waypoint.
        self._is_finished = False
        while not self._is_finished:
            # Issue the navigation command about twice a second such that it is easy to terminate the
            # navigation command (with estop or killing the program).
            try:
                nav_to_cmd_id = self._graph_nav_client.navigate_to(destination_waypoint, 1.0,
                                                                   command_id=nav_to_cmd_id)
            except ResponseError as e:
                print(f'Error while navigating {e}')
                break
            time.sleep(.5)  # Sleep for half a second to allow for command execution.
            # Poll the robot for feedback to determine if the navigation command is complete. Then sit
            # the robot down once it is finished.
            self._is_finished = self._check_success(nav_to_cmd_id)
        print("waypointに到着しました")
        # log_event("waypointに到着しました")
        # engine.say("arrived at the waypoint")
        # engine.runAndWait()
        # voice_output("arrived at the waypoint")
        print("-" * 80)

    def _navigate_route(self, *args):

        speed_x = 0.6 * self._speed_limit
        speed_y = 1.0 * self._speed_limit
        speed_angle = 0.5 * self._speed_limit

        # 速度の設定
        speed_limit = geo.SE2VelocityLimit(
        max_vel=geo.SE2Velocity(
        linear=geo.Vec2(x=speed_x, y=speed_y),
        angular=speed_angle))
        print("確認1")
        
        params = self._graph_nav_client.generate_travel_params(
        max_distance = 0.5, max_yaw = 0.5, velocity_limit = speed_limit)

        print("確認2")
        
        waypoint_ids = self._waypoint_all_id
        for i in range(len(waypoint_ids)):
            print("before waypoint_ids[i]", waypoint_ids[i])
            waypoint_ids[i] = graph_nav_util.find_unique_waypoint_id(
                waypoint_ids[i], self._current_graph, self._current_annotation_name_to_wp_id)
            if not waypoint_ids[i]:
                # Failed to find the unique waypoint id.
                return
            print("after waypoint_ids[i]", waypoint_ids[i])
            
        edge_ids_list = []
        
        print("確認3")
        for i in range(len(waypoint_ids) - 1):
            start_wp = waypoint_ids[i]
            end_wp = waypoint_ids[i + 1]
            edge_id = self._match_edge(self._current_edges, start_wp, end_wp)
            if edge_id is not None:
                edge_ids_list.append(edge_id)
            else:
                all_edges_found = False
                print(f'Failed to find an edge between waypoints: {start_wp} and {end_wp}')
                print(
                    'List the graph\'s waypoints and edges to ensure pairs of waypoints has an edge.'
                )
                break
            
            
        route = self._graph_nav_client.build_route(waypoint_ids, edge_ids_list)
        self._is_finished = False
        previous_waypoint = None
        print("確認4")
        while not self._is_finished:
            try:
                nav_route_command_id = self._graph_nav_client.navigate_route(
                    route, cmd_duration=1.0, travel_params = params)

                # 終点に到達したかの確認
                self._is_finished = self._check_success(nav_route_command_id)
                
                if (self._is_finished):
                    print("終点に到達しました")
                    # engine.say("arrived at the end point")
                    # engine.runAndWait()
                    # voice_output("arrived at the end point")
                    print("-" * 80)
                    break
                    
                
            except ResponseError as e:
                break

            
            
    def _navigate_route_to_user_waypoint(self, current, current_waypoint_id, goal, goal_waypoint_id):
        # ユーザーが設定したwaypointに移動する
        
        # self.rcl._download_full_graph()
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            print('Empty graph.')
            return
        self._current_graph = graph
        
        minrootgraph = dijkstra.MinRootGraph()
        
        # 入力データの作成
        graph_waypoints = []
        graph_edges = []
        
        print("確認1")
        for waipoint in range(len(self._current_graph.waypoints)):
            graph_dict = {
                "id": self._current_graph.waypoints[waipoint].id,
                "position": self._current_graph.waypoints[waipoint].waypoint_tform_ko.position,
                "name": self._current_graph.waypoints[waipoint].annotations.name
            }
            graph_waypoints.append(graph_dict)
            
        print("確認2")
            
        for edge in range(len(self._current_graph.edges)):
            value = self._current_graph.edges[edge].annotations.cost
            x = self._current_graph.edges[edge].from_tform_to.position.x
            y = self._current_graph.edges[edge].from_tform_to.position.y
            z = self._current_graph.edges[edge].from_tform_to.position.z
            
            position = sqrt(x**2 + y**2 + z**2)
            
            if hasattr(value, "value"):
                cost = float(value.value)
            else:
                cost = float(value)
            graph_dict = {
                "from_waypoint": self._current_graph.edges[edge].id.from_waypoint,
                "to_waypoint": self._current_graph.edges[edge].id.to_waypoint,
                "position": position,
            }
            print(graph_dict)
            graph_edges.append(graph_dict)

        print("確認3")
            
        for wayopint in range(len(graph_waypoints)):
            minrootgraph.add_waypoint(self._current_graph.waypoints[wayopint].id)
            
        for edge in range(len(self._current_graph.edges)):
            minrootgraph.add_edge(self._current_graph.edges[edge].id.from_waypoint, self._current_graph.edges[edge].id.to_waypoint, self._current_graph.edges[edge].annotations.cost)

        start_wp_id = current_waypoint_id
        goal_wp_id = goal_waypoint_id
        
        distances, path = minrootgraph.dijkstra(start_wp_id)
        
        print("確認4")
        def get_path(paths, start, end):
            path = []
            while end != start:
                path.append(end)
                end = paths[end]
            path.append(start)
            return path[::-1]
        
        shortest_path = get_path(path, start_wp_id, goal_wp_id)
        
        waypoint_route = [] 
        print("確認5")
        for i in range(len(shortest_path)):
            shortest_path[i] = graph_nav_util.find_unique_waypoint_id(
                shortest_path[i], self._current_graph, self._current_annotation_name_to_wp_id)
            if not shortest_path[i]:
                # Failed to find the unique waypoint id.
                return
            waypoint_route.append(shortest_path[i])
            
        edge_ids_list = []
        for i in range(len(waypoint_route) - 1):
            start_wp = waypoint_route[i]
            end_wp = waypoint_route[i + 1]
            edge_id = self._match_edge(self._current_edges, start_wp, end_wp)
            if edge_id is not None:
                edge_ids_list.append(edge_id)
            else:
                all_edges_found = False
                print(f'Failed to find an edge between waypoints: {start_wp} and {end_wp}')
                print(
                    'List the graph\'s waypoints and edges to ensure pairs of waypoints has an edge.'
                )
                break
        print("確認6")

        speed_x = 0.6 * self._speed_limit
        speed_y = 1.0 * self._speed_limit
        speed_angle = 0.5 * self._speed_limit

        speed_limit = geo.SE2VelocityLimit(
        max_vel=geo.SE2Velocity(
        linear=geo.Vec2(x=speed_x, y=speed_y),
        angular=speed_angle))
        
        params = self._graph_nav_client.generate_travel_params(
        max_distance = 0.5, max_yaw = 0.5, velocity_limit = speed_limit)

            
        route = self._graph_nav_client.build_route(waypoint_route, edge_ids_list)
        self._is_finished = False
        self._is_autodrive_cancel = False
        
        while not self._is_finished:
            try:
                nav_route_command_id = self._graph_nav_client.navigate_route(
                    route, cmd_duration=1.4, travel_params = params)

                # 終点に到達したかの確認
                self._is_finished = self._check_success(nav_route_command_id)
                if (self._is_finished):
                    # engine.say("arrive at the goal")
                    # engine.runAndWait()
                    print("arrive at the goal")
                    # log_event("到着しました")
                    print("-" * 80)
                    # voice_output("arrive at the goal")
                    
                    try:
                        self._set_initial_localization_fiducial()
                    except Exception as e:
                        print(f'failed initialize {e}')
                    print("-" * 80)
                    
                    break
                
                # 自動運転がキャンセルされたときの処理
                if (self._is_autodrive_cancel):
                    print("自動運転をキャンセルしました")
                    # engine.say("cancel auto drive")
                    # engine.runAndWait()
                    print("-" * 80)
                    try:
                        self._set_initial_localization_fiducial()
                    except Exception as e:
                        print(f'failed initialize {e}')
                    print("-" * 80)
                    
                    self._is_finished = True
                    
                    break
                
            except ResponseError as e:
                self._is_finished = True
                print("-" * 80)
                break
        
            
            
    def _check_success(self, command_id=-1):
        """Use a navigation command id to get feedback from the robot and sit when command succeeds."""
        if command_id == -1:
            # No command, so we have no status to check.
            return False
        status = self._graph_nav_client.navigation_feedback(command_id)
        if status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
            # Successfully completed the navigation commands!
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            print('Robot got lost when navigating the route, the robot will now sit down.')
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            print('Robot got stuck when navigating the route, the robot will now sit down.')
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED:
            print('Robot is impaired.')
            return True
        else:
            # Navigation command is not complete yet.
            return False
        
            
        
            
    def _match_edge(self, current_edges, waypoint1, waypoint2):
        """Find an edge in the graph that is between two waypoint ids."""
        # Return the correct edge id as soon as it's found.
        for edge_to_id in current_edges:
            for edge_from_id in current_edges[edge_to_id]:
                if (waypoint1 == edge_to_id) and (waypoint2 == edge_from_id):
                    # This edge matches the pair of waypoints! Add it the edge list and continue.
                    return map_pb2.Edge.Id(from_waypoint=waypoint2, to_waypoint=waypoint1)
                elif (waypoint2 == edge_to_id) and (waypoint1 == edge_from_id):
                    # This edge matches the pair of waypoints! Add it the edge list and continue.
                    return map_pb2.Edge.Id(from_waypoint=waypoint1, to_waypoint=waypoint2)
        return None

    def _clear_graph(self, *args):
        """Clear the state of the map on the robot, removing all waypoints and edges."""
        return self._graph_nav_client.clear_graph()