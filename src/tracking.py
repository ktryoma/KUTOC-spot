import math
import time
import datetime
import os
import cv2

from bosdyn.client.robot_command import RobotCommandBuilder

class Tracking:
    def __init__(self, log_event, update_robot_state, model, create_mobility_params
                 , SPEED_LIMIT, BOX_WIDTH, BOX_HEIGHT, DIRECT_DISTANCE,
                 PROCESSED_IMAGE_PATH, SHUTDOWN_FLAG, REAL_BIBS_HEIGHT):
        self.log_event = log_event
        self.update_robot_state = update_robot_state
        self.model = model
        self.create_mobility_params = create_mobility_params
        self.SPEED_LIMIT = SPEED_LIMIT
        self.BOX_WIDTH = BOX_WIDTH
        self.BOX_HEIGHT = BOX_HEIGHT
        self.DIRECT_DISTANCE = DIRECT_DISTANCE
        self.PROCESSED_IMAGE_PATH = PROCESSED_IMAGE_PATH
        if not os.path.exists(self.PROCESSED_IMAGE_PATH):
            os.makedirs(self.PROCESSED_IMAGE_PATH)
        self.SHUTDOWN_FLAG = SHUTDOWN_FLAG  # This should be set to True to stop the thread
        self.REAL_BIBS_HEIGHT = REAL_BIBS_HEIGHT  # mm, height of the bib in real world

    def gstreamer_pipeline(
        self,
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

    def calculate_angle_speed(self, angle):
        if angle <= math.pi/4 or angle > 3 * math.pi/4:
            angle_speed = 0.25
        elif angle <= math.pi/2:
            # Linear interpolation between 0.25 and 0.5 for angles between pi/4 and pi/2
            angle_speed = ((math.pi/2 - angle) / (math.pi/2 - math.pi/4)) * (1.0 - 0.25) + 0.25
        else:  # math.pi/2 < angle <= 3 * math_pi/4
            # Linear interpolation between 0.5 and 0.25 for angles between math.pi/2 and 3*math.pi/4
            angle_speed = ((3 * math.pi/4 - angle) / (3 * math.pi/4 - math.pi/2)) * (0.25 - 1.0) + 0.5
        return angle_speed


    def move_to_position_cmd_make(self, robot, x, l, z, box_width, mobility_params=None):
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
        slope = 0.1 * self.SPEED_LIMIT
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
            angle_speed = self.calculate_angle_speed(angle)
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
            angle_speed = self.calculate_angle_speed(angle)
            print(f"Angle speed: {angle_speed}")
            # 対象の距離まで「進んでほしい位置」まで進むようにする
            z = z - 1.0

        elif (box_width <= self.BOX_WIDTH+5 and box_width >= self.BOX_WIDTH-5) or (z > 2.3 and z <= 2.5):
            # print("The robot is close to the target")
            speed = 0.0
            x, l, z, = 0, 0, 0

        elif z <= 2.3 or box_width >= self.BOX_WIDTH+5:
            angle_speed = 0.25
            print("The robot is far away from human")
            z = -1.3 + z


        # angle = math.atan2(z, x)
        # spot_angle = angle - (math.pi/2)
        # spot_angle = spot_angle * (-1)
        # angle_speed = calculate_angle_speed(angle)
        # print(f"Angle speed: {angle_speed}")

        # 速度と角速度の設定
        mobility_params = self.create_mobility_params(speed, angle_speed)

        print("z", z)
        print("x", x)
        print("angle", angle)

        tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(z, x, angle, 
        robot_state.kinematic_state.transforms_snapshot, mobility_params)

        return tag_cmd



    def calc_distance(self, results, img):
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
        distance_per_pixel = self.REAL_BIBS_HEIGHT / box_height
        print("1ピクセル当たりの距離mm", distance_per_pixel)

        # 画像の中心点からの距離
        distance_x = center_x - bb_center_x
        print("中心点からの距離", distance_x)

        # 画像の中心点からの現実世界の距離
        x = distance_x * distance_per_pixel
        print(f"Distance_x: {x} mm")

        # カメラから対象までの直線距離
        l = self.DIRECT_DISTANCE * (self.BOX_HEIGHT / box_height)
        # print(f"Distance_l: {l} mm")

        # カメラから対象までの縦方向の距離
        z = math.sqrt(l ** 2 - x ** 2)
        # print(f"Distance_z: {z} mm")


        return x, l, z, box_width, box_height

    def process_image_thread(self, robot_state_client, robot_command_client):
        last_capture_time = 0  # 2秒に1回更新用のタイムスタンプ
        try:
            video_capture = cv2.VideoCapture(self.gstreamer_pipeline(), cv2.CAP_GSTREAMER)
            self.update_robot_state(tracking=True)
            if video_capture.isOpened():
                print("Video stream is opened")
                while not self.SHUTDOWN_FLAG.value:
                    # ...existing code...
                    try:
                        ret_val, frame = video_capture.read()
                        if ret_val:
                            resized_frame = cv2.resize(frame, (640, 360))
                            results = self.model(resized_frame, classes=[0], conf=0.8, device=0)
                            if results and results[0].boxes.xyxy.shape[0] > 0:
                                current_time = time.time()
                                # 2秒経過している場合のみ画像を保存
                                if current_time - last_capture_time >= 2.0:
                                    timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S%f")[:-4]
                                    cv2.imwrite(os.path.join(self.PROCESSED_IMAGE_PATH, f"detected_{timestamp}.jpg"), resized_frame)
                                    last_capture_time = current_time
                                # ...既存の認識検出、コマンド送信処理...
                                for box in results[0].boxes.xyxy:
                                    x, l, z, box_width, box_height = self.calc_distance(results, resized_frame)
                                    print(f"Detected object at: x={x}, l={l}, z={z}")
                                    left_top_x = int(box[0])
                                    right_bottom_x = int(box[2])
                                    
                                    # 画像の幅
                                    img_width = resized_frame.shape[1]
                                    
                                    robot_state = robot_state_client.get_robot_state()
                                    # 左にバウンディングボックスが寄っているとき
                                    if left_top_x < 90:
                                        print("The object is on the left side")

                                        mobility_params = self.create_mobility_params(0.75, 0.6)
                                        tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(0, 0, math.cos(z/l),
                                            robot_state.kinematic_state.transforms_snapshot, mobility_params)
                                        end_time = 0.3
                                        robot_command_client.robot_command(lease=None, command=tag_cmd,
                                                                        end_time_secs=time.time() + end_time)
                                        # time.sleep(1.0)
                                        
                                    # 右にバウンディングボックスが寄っているとき 
                                    elif right_bottom_x > img_width - 90:
                                        print("The object is on the right side")

                                        mobility_params = self.create_mobility_params(0.75, 0.6)
                                        tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(0, 0, -math.cos(z/l),
                                            robot_state.kinematic_state.transforms_snapshot, mobility_params)
                                        end_time = 0.3
                                        robot_command_client.robot_command(lease=None, command=tag_cmd,
                                                                        end_time_secs=time.time() + end_time)
                                        # time.sleep(1.0)
                                    
                                    else:
                                        tag_cmd = self.move_to_position_cmd_make(robot_state_client, x, l, z, box_width)

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
                                    cv2.imwrite(os.path.join(self.PROCESSED_IMAGE_PATH, f"detected_{timestamp}.jpg"), resized_frame)
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
                                tag_cmd = self.move_to_position_cmd_make(robot_state_client, 0, 0, 0, 0)
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
            self.log_event(f"エラー: {e}")
        finally:
            cv2.destroyAllWindows()
        print("Success to finish process_image_thread")
        return True
