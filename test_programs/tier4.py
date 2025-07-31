# MIT License
# Copyright (c) 2019-2022 JetsonHacks

# Using a CSI camera (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit using OpenCV
# Drivers for the camera and OpenCV are included in the base image
import os
import datetime
import cv2
from ultralytics import YOLO
import time
import math

REAL_BIBS_HEIGHT = 610
DIRECT_DISTANCE = 2500
BOX_HEIGHT = 65.13135711669922
PROCESSED_IMAGE_PATH = "calc"

# model = YOLO("best.pt")
# model = YOLO(r"pt_list\0204_epoch200_batch8.pt")
# model = YOLO(r"redbibs_0222.pt")
model = YOLO(r"../allbibs_epoch1000_batch16.pt")


""" 
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080 displayd in a 1/4 size window
"""

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

def show_camera():
    window_title = "tier4"

    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    # print(gstreamer_pipeline(flip_method=0))
    video_capture = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    print("Camera is opened")
    if video_capture.isOpened():
        try:
            # window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            while True:
                ret_val, frame = video_capture.read()
                # Check to see if the user closed the window
                # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
                # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    save_image(frame, directory="0310_orig")
                    resize_frame = cv2.resize(frame, (640, 360))
                    results = model(resize_frame, classes=[0], conf=0.8, device=0)
                    annoted_frame = results[0].plot()
                    # cv2.imshow(window_title, annoted_frame)
                    save_image(annoted_frame, directory="0310")
                else:
                    break 
                keyCode = cv2.waitKey(10) & 0xFF
                # Stop the program on the ESC key or 'q'
                if keyCode == 27 or keyCode == ord('q'):
                    break
                
        except Exception as e:
            print(f"Error: {e}")
        finally:
            video_capture.release()
            cv2.destroyAllWindows()
    else:
        print("Error: Unable to open camera")
        


def inference_bibs():
    try:
        video_capture = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if video_capture.isOpened():
            while True:
                ret_val, frame = video_capture.read()
                if ret_val:
                    save_image(frame, directory="0310_orig")
                    resize_frame = cv2.resize(frame, (640, 360))
                    resize_frame = cv2.rotate(resize_frame, cv2.ROTATE_180)
                    results = model(resize_frame, classes=[0], conf=0.8, device=0)
                    anno = results[0].plot()
                    save_image(anno, directory="0310")
        else:
            print("Error: Unable to open camera")
    except KeyboardInterrupt:
        print("プログラムを終了します")
    finally:
        video_capture.release()
        
def calc_bibs():
    time.sleep(5)
    video_capture = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    # 初期化: 検出したビブスの累計値と検出回数
    total_x = total_l = total_z = total_box_width = total_box_height = 0
    detection_count = 0
    if video_capture.isOpened():
        try:
            while True:
                ret_val, frame = video_capture.read()
                if ret_val:
                    resized_frame = cv2.resize(frame, (640, 360))
                    results = model(resized_frame, classes=[0], conf=0.8, device=0)
                    if results and results[0].boxes.xyxy.shape[0] > 0:
                        timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S%f")[:-4]
                        for box in results[0].boxes.xyxy:
                            x, l, z, box_width, box_height = calc_distance(results, resized_frame)
                            print(f"Detected object at: x={x}, l={l}, z={z}")
                            left_top_x = int(box[0])
                            right_bottom_x = int(box[2])
                            
                            # 画像の幅
                            img_width = resized_frame.shape[1]
                            # 左にバウンディングボックスが寄っているとき
                            if left_top_x < 90:
                                print("The object is on the left side")
                                # time.sleep(1.0)
                                
                            # 右にバウンディングボックスが寄っているとき 
                            elif right_bottom_x > img_width - 90:
                                print("The object is on the right side")
                                # time.sleep(1.0)
                            
                            
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
                            
                            # 累計値を更新
                            total_x += x
                            total_l += l
                            total_z += z
                            total_box_width += box_width
                            total_box_height += box_height
                            detection_count += 1
                            if detection_count >= 100:
                                # 平均値を計算して出力
                                avg_x = total_x / detection_count
                                avg_l = total_l / detection_count
                                avg_z = total_z / detection_count
                                avg_box_width = total_box_width / detection_count
                                avg_box_height = total_box_height / detection_count
                                print(f"Averages over {detection_count} detections:")
                                print(f"avg_x: {avg_x} mm, avg_l: {avg_l} mm, avg_z: {avg_z} mm")
                                print(f"avg_box_width: {avg_box_width} px, avg_box_height: {avg_box_height} px")
                                raise KeyboardInterrupt
                    else:
                        print("No object detected")
                            
                    time.sleep(0.1)
                    
        except KeyboardInterrupt:
            print("プログラムを終了します")
        finally:
            video_capture.release()
            
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

def save_image(frame, directory="tier4"):
    if not os.path.exists(directory):
        os.makedirs(directory)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S.%f")
    filename = os.path.join(directory, f"image_{timestamp}.jpg")
    cv2.imwrite(filename, frame)
    print(f"Image saved to {filename}")



if __name__ == "__main__":
    # show_camera()
    inference_bibs()
    # calc_bibs()

