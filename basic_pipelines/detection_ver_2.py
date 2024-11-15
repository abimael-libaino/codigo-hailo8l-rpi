import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import argparse
import numpy as np
import setproctitle
import time
from pymavlink import mavutil
import cv2
import json
import hailo
from hailo_rpi_common import (
    get_default_parser,
    QUEUE,
    get_caps_from_pad,
    get_numpy_from_buffer,
    GStreamerApp,
    app_callback_class,
)
from time import sleep
import sys

current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
  
sys.path.append(parent_directory)

from siyi_sdk import SIYISDK

master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
master.wait_heartbeat()
print("Conectado ao sistema:", master.target_system, ", componente:", master.target_component)

def test():
    cam = SIYISDK(server_ip="192.168.144.25", port=37260)
    if not cam.connect():
        print("No connection ")
        exit(1)

    cam.setGimbalRotation(0, -25)  # de -90 ate 25 graus.
    cam.disconnect()

if __name__ == "__main__":
    test()

class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.new_variable = 42
        self.frame_count = 0
        self.detections_data = []  # List to hold detection data for each frame
        self.gps_coordinates = (0, 0, 0)  # To hold latitude, longitude, altitude
    
    def gpsmavlink(self):
        msg = master.recv_match(blocking=False)
        if msg and msg.get_type() == 'GLOBAL_POSITION_INT':
            lat = msg.lat / 1E7  # Converte para graus
            lon = msg.lon / 1E7  # Converte para graus
            alt = msg.alt / 1000.0  # Converte para metros
            self.gps_coordinates = (lat, lon, alt)
            print(f"Coordenadas GPS - Latitude: {lat}, Longitude: {lon}, Altitude: {alt} m")
        return self.gps_coordinates

    def increment(self):
        self.frame_count += 1

    def get_count(self):
        return self.frame_count
    
    def add_detection_data(self, frame_number, detection_count, coordinates):
        self.detections_data.append({
            'frame': frame_number,
            'number_of_people': detection_count,
            'coordinates': coordinates
        })

    def save_to_json(self, filename="detections.json"):
        with open(filename, 'w') as f:
            json.dump(self.detections_data, f, indent=4)

def app_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    user_data.increment()
    frame_number = user_data.get_count()
    user_data.gpsmavlink()  # Atualiza as coordenadas GPS
    coordinates = user_data.gps_coordinates

    format, width, height = get_caps_from_pad(pad)

    frame = None
    if user_data.use_frame and format is not None and width is not None and height is not None:
        frame = get_numpy_from_buffer(buffer, format, width, height)

    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    detection_count = 0
    person_detected = False
    for detection in detections:
        label = detection.get_label()
        confidence = detection.get_confidence()
        if label == "person":
            detection_count += 1
            person_detected = True

    if user_data.use_frame:
        cv2.putText(frame, f"Detections: {detection_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"GPS: {coordinates[0]:.5f}, {coordinates[1]:.5f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        user_data.set_frame(frame)

        if person_detected:
            image_dir = "imagens"
            os.makedirs(image_dir, exist_ok=True)
            image_path = os.path.join(image_dir, f"frame_{frame_number}.jpg")
            cv2.imwrite(image_path, frame)

    user_data.add_detection_data(frame_number, detection_count, coordinates)
    print(f"Frame: {frame_number}, Detections: {detection_count}, Coordinates: {coordinates}")

    user_data.save_to_json()

    return Gst.PadProbeReturn.OK

lass GStreamerDetectionApp(GStreamerApp):
    def __init__(self, args, user_data):
        super().__init__(args, user_data)
        self.saving = args.save
        self.batch_size = 2
        self.network_width = 640
        self.network_height = 640
        self.network_format = "RGB"
        nms_score_threshold = 0.3 
        nms_iou_threshold = 0.45
        
        new_postprocess_path = os.path.join(self.current_path, '../resources/libyolo_hailortpp_post.so')
        if os.path.exists(new_postprocess_path):
            self.default_postprocess_so = new_postprocess_path
        else:
            self.default_postprocess_so = os.path.join(self.postprocess_dir, 'libyolo_hailortpp_post.so')

        if args.hef_path is not None:
            self.hef_path = args.hef_path
        elif args.network == "yolov6n":
            self.hef_path = os.path.join(self.current_path, '../resources/yolov6n.hef')
        elif args.network == "yolov8s":
            self.hef_path = os.path.join(self.current_path, '../resources/yolov8s_h8l.hef')
        elif args.network == "yolox_s_leaky":
            self.hef_path = os.path.join(self.current_path, '../resources/yolox_s_leaky_h8l_mz.hef')
        else:
            assert False, "Invalid network type"

        if args.labels_json is not None:
            self.labels_config = f' config-path={args.labels_json} '
            if not os.path.exists(new_postprocess_path):
                print("New postprocess so file is missing. It is required to support custom labels. Check documentation for more information.")
                exit(1)
        else:
            self.labels_config = ''

        self.app_callback = app_callback
    
        self.thresholds_str = (
            f"nms-score-threshold={nms_score_threshold} "
            f"nms-iou-threshold={nms_iou_threshold} "
            f"output-format-type=HAILO_FORMAT_TYPE_FLOAT32"
        )

        setproctitle.setproctitle("Hailo Detection App")

        self.create_pipeline()

    def get_pipeline_string(self):
        if self.source_type == "rpi":
            source_element = (
                "libcamerasrc name=src_0 auto-focus-mode=2 ! "
                f"video/x-raw, format={self.network_format}, width=1536, height=864 ! "
                + QUEUE("queue_src_scale")
                + "videoscale ! "
                f"video/x-raw, format={self.network_format}, width={self.network_width}, height={self.network_height}, framerate=30/1 ! "
            )
        elif self.source_type == "usb":
            source_element = (
                f"v4l2src device={self.video_source} name=src_0 ! "
                "video/x-raw, width=640, height=480, framerate=30/1 ! "
            )
        elif self.source_type == "rtsp":
            source_element = (
                f"rtspsrc location={self.video_source} name=src_0 message-forward=true ! "
                + "rtph265depay ! "
                + "queue name=hailo_preprocess_q_0 leaky=no max-size-buffers=10 max-size-bytes=0 max-size-time=0 ! "
                + "decodebin ! queue leaky=downstream max-size-buffers=5 max-size-bytes=0 max-size-time=0 ! "
                "video/x-raw, format=I420 ! "
            )
        else:
            source_element = (
                f"filesrc location={self.video_source} name=src_0 ! "
                + QUEUE("queue_dec264")
                + "qtdemux ! h264parse ! avdec_h264 max-threads=2 ! "
                "video/x-raw, format=I420 ! "
            )
        source_element += QUEUE("queue_scale")
        source_element += "videoscale n-threads=2 ! "
        source_element += QUEUE("queue_src_convert")
        source_element += "videoconvert n-threads=3 name=src_convert qos=false ! "
        source_element += f"video/x-raw, format={self.network_format}, width={self.network_width}, height={self.network_height}, pixel-aspect-ratio=1/1 ! "

        pipeline_string = (
            "hailomuxer name=hmux "
            + source_element
            + "tee name=t ! "
            + QUEUE("bypass_queue", max_size_buffers=20)
            + "hmux.sink_0 "
            + "t. ! "
            + QUEUE("queue_hailonet")
            + "videoconvert n-threads=3 ! "
            f"hailonet hef-path={self.hef_path} batch-size={self.batch_size} {self.thresholds_str} force-writable=true ! "
            + QUEUE("queue_hailofilter")
            + f"hailofilter so-path={self.default_postprocess_so} {self.labels_config} qos=false ! "
            + QUEUE("queue_hmuc")
            + "hmux.sink_1 "
            + "hmux. ! "
            + QUEUE("queue_hailo_python")
            + QUEUE("queue_user_callback")
            + "identity name=identity_callback ! "
            + QUEUE("queue_hailooverlay")
            + "hailooverlay ! "
            + QUEUE("queue_videoconvert")
            + "videoconvert n-threads=3 qos=false ! "
        )

        if self.saving is None:
            pipeline_string += (
                QUEUE("queue_hailo_display")
                + f"fpsdisplaysink video-sink={self.video_sink} name=hailo_display sync={self.sync} text-overlay={self.options_menu.show_fps} signal-fps-measurements=true "
            )
        else:
            pipeline_string += (
                f"{QUEUE('queue_save')} "
                "videoconvert ! "
                "x264enc bitrate=6000 speed-preset=ultrafast tune=zerolatency ! "
                "matroskamux ! "
                f"filesink location={self.saving} "
            )
        print(pipeline_string)
        return pipeline_string

if __name__ == "__main__":
    user_data = user_app_callback_class()
    parser = get_default_parser()
    parser.add_argument("--save", type=str, default=None, help="Directory to save video to file (default: not saving)")
    parser.add_argument(
        "--network",
        default="yolov8s",
        choices=['yolov6n', 'yolov8s', 'yolox_s_leaky'],
        help="Which Network to use, default is yolov6n",
    )
    parser.add_argument(
        "--hef-path",
        default=None,
        help="Path to HEF file",
    )
    parser.add_argument(
        "--labels-json",
        default=None,
        help="Path to custom labels JSON file",
    )
    args = parser.parse_args()
    app = GStreamerDetectionApp(args, user_data)
    app.run()


