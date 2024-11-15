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
import sys
from hailo_rpi_common import (
    get_default_parser,
    QUEUE,
    get_caps_from_pad,
    get_numpy_from_buffer,
    GStreamerApp,
    app_callback_class,
)
#from time import sleep
#import sys

#current = os.path.dirname(os.path.realpath(__file__))
#parent_directory = os.path.dirname(current)
  
#sys.path.append(parent_directory)

#from siyi_sdk import SIYISDK
#master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
#master.wait_heartbeat()
#print("Conectado ao sistema:", master.target_system, ", componente:", master.target_component)
#def test():
#    cam = SIYISDK(server_ip="192.168.144.25", port=37260)
#    if not cam.connect():
#        print("No connection ")
#        exit(1)

#    cam.setGimbalRotation(0, -25) # de -90 ate 25 graus.

 #   cam.disconnect()

#if __name__ == "__main__":
#    test()

class MAVLinkHandler:
    def __init__(self, port='/dev/ttyAMA0', baud=115200):
        self.port = port
        self.baud = baud
        self.vehicle = None
        self.connected = False
        self.last_position = None
        self.position_received = False
        
    def connect(self):
        """Establishes connection with the vehicle via MAVLink"""
        try:
            print(f"Connecting to vehicle on {self.port}...")
            self.vehicle = mavutil.mavlink_connection(self.port, baud=self.baud)
            self.wait_heartbeat()
            self.connected = True
            return self.wait_first_position()
        except Exception as e:
            print(f"MAVLink connection failed: {str(e)}")
            self.connected = False
            return False
            
    def wait_heartbeat(self):
        """Waits for the first heartbeat from the vehicle"""
        try:
            print("Waiting for heartbeat...")
            self.vehicle.wait_heartbeat(timeout=10)
            print(f"Heartbeat received from system: {self.vehicle.target_system}, "
                  f"component: {self.vehicle.target_component}")
        except Exception as e:
            print(f"Error waiting for heartbeat: {e}")
            raise

    def wait_first_position(self, timeout=30):
        """
        Waits for the first position message from the vehicle
        
        Args:
            timeout (int): Maximum time to wait in seconds
            
        Returns:
            bool: True if position received, False if timeout
        """
        print("Waiting for first position message...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            msg = self.vehicle.recv_match(
                type='GLOBAL_POSITION_INT',
                blocking=True,
                timeout=1.0
            )
            
            if msg is not None:
                lat = msg.lat / 1E7
                lon = msg.lon / 1E7
                alt = msg.relative_alt / 1000.0
                
                print(f"Initial position received:")
                print(f"Latitude: {lat:.7f}°")
                print(f"Longitude: {lon:.7f}°")
                print(f"Relative Altitude: {alt:.2f}m")
                
                self.last_position = (lat, lon, alt)
                self.position_received = True
                return True
                
        print(f"Timeout after {timeout} seconds waiting for position")
        return False
            
    def get_position(self):
        """Gets the current global position from the vehicle"""
        if not self.connected or self.vehicle is None:
            return None, None, None
            
        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            lat = msg.lat / 1E7
            lon = msg.lon / 1E7
            alt = msg.relative_alt / 1000.0
            self.last_position = (lat, lon, alt)
            return lat, lon, alt
        return self.last_position if self.last_position else (None, None, None)

class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.frame_count = 0
        self.detections_data = []
        self.current_frame = None
        self.last_save_time = time.time()
        self.save_interval = 1.0
        self.mavlink = MAVLinkHandler()
        
        print("Initializing MAVLink connection and waiting for first position...")
        if not self.mavlink.connect():
            print("Failed to get initial position. Exiting...")
            sys.exit(1)
        print("System ready - starting processing...")
    
    def get_gps_data(self):
        """Wrapper for getting GPS data from MAVLink"""
        return self.mavlink.get_position()
    
    def increment(self):
        self.frame_count += 1

    def get_count(self):
        return self.frame_count
    
    def set_frame(self, frame):
        self.current_frame = frame
    
    def add_detection_data(self, frame_number, detection_count):
        lat, lon, alt = self.get_gps_data()
        detection_data = {
            'frame': frame_number,
            'number_of_people': detection_count,
            'gps': {
                'latitude': lat,
                'longitude': lon,
                'altitude': alt
            },
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
        }
        self.detections_data.append(detection_data)
        
        # Print detection with coordinates
        print(f"Detection at Frame {frame_number}:")
        print(f"  People detected: {detection_count}")
        print(f"  Position: Lat: {lat:.7f}°, Lon: {lon:.7f}°, Alt: {alt:.2f}m")
        print(f"  Time: {detection_data['timestamp']}\n")

    def save_to_json(self, filename="detections.json"):
        with open(filename, 'w') as f:
            json.dump(self.detections_data, f, indent=4)

def app_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    user_data.increment()
    frame_number = user_data.get_count()

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

    if user_data.use_frame and frame is not None:
        # Get current GPS position
        lat, lon, alt = user_data.get_gps_data()
        
        # Add MAVLink connection status and GPS data
        mavlink_status = "MAVLink: Connected" if user_data.mavlink.connected else "MAVLink: Disconnected"
        status_color = (0, 255, 0) if user_data.mavlink.connected else (0, 0, 255)
        
        # Convert frame for OpenCV operations
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # Add detection count
        cv2.putText(frame, f"Detections: {detection_count}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Add MAVLink status
        cv2.putText(frame, mavlink_status, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)
        
        # Add GPS coordinates if available
        if lat is not None and lon is not None:
            gps_text = f"GPS: {lat:.6f}, {lon:.6f}, {alt:.1f}m"
            cv2.putText(frame, gps_text, (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
        
        user_data.set_frame(frame)

        # Save frame if person detected and enough time has passed
        current_time = time.time()
        if person_detected and (current_time - user_data.last_save_time) >= user_data.save_interval:
            image_dir = "detected_people"
            os.makedirs(image_dir, exist_ok=True)
            
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            image_path = os.path.join(image_dir, f"detection_{timestamp}_frame_{frame_number}.jpg")
            
            cv2.imwrite(image_path, frame)
            user_data.last_save_time = current_time
            print(f"Saved detection image: {image_path}")

    if person_detected:
        user_data.add_detection_data(frame_number, detection_count)
        user_data.save_to_json()

    return Gst.PadProbeReturn.OK

# Rest of the GStreamerDetectionApp class remains the same...

class GStreamerDetectionApp(GStreamerApp):
    def __init__(self, args, user_data):
        super().__init__(args, user_data)
        self.saving = args.save
        self.batch_size = 2
        self.network_width = 640
        self.network_height = 640
        self.network_format = "RGB"
        self.nms_score_threshold = 0.3 
        self.nms_iou_threshold = 0.45
        
        new_postprocess_path = os.path.join(self.current_path, '../resources/libyolo_hailortpp_post.so')
        self.default_postprocess_so = (new_postprocess_path 
                                     if os.path.exists(new_postprocess_path) 
                                     else os.path.join(self.postprocess_dir, 'libyolo_hailortpp_post.so'))

        self.hef_path = args.hef_path if args.hef_path is not None else {
            "yolov6n": os.path.join(self.current_path, '../resources/yolov6n.hef'),
            "yolov8s": os.path.join(self.current_path, '../resources/yolov8s_h8l.hef'),
            "yolox_s_leaky": os.path.join(self.current_path, '../resources/yolox_s_leaky_h8l_mz.hef')
        }.get(args.network)

        if self.hef_path is None:
            raise ValueError("Invalid network type")

        if args.labels_json is not None:
            if not os.path.exists(new_postprocess_path):
                raise RuntimeError("New postprocess so file is missing. It is required to support custom labels. Check documentation for more information.")
            self.labels_config = f' config-path={args.labels_json} '
        else:
            self.labels_config = ''

        self.app_callback = app_callback
    
        self.thresholds_str = (
            f"nms-score-threshold={self.nms_score_threshold} "
            f"nms-iou-threshold={self.nms_iou_threshold} "
            f"output-format-type=HAILO_FORMAT_TYPE_FLOAT32"
        )

        setproctitle.setproctitle("Hailo Detection App")
        self.create_pipeline()

    def get_pipeline_string(self):
        source_elements = {
            "rpi": (
                f"libcamerasrc name=src_0 auto-focus-mode=2 ! "
                f"video/x-raw, format={self.network_format}, width=1536, height=864 ! "
                + QUEUE("queue_src_scale")
                + f"videoscale ! "
                f"video/x-raw, format={self.network_format}, width={self.network_width}, height={self.network_height}, framerate=30/1 ! "
            ),
            "usb": (
                f"v4l2src device={self.video_source} name=src_0 ! "
                "video/x-raw, width=640, height=480, framerate=30/1 ! "
            ),
            "rtsp": (
                f"rtspsrc location={self.video_source} name=src_0 message-forward=true ! "
                "rtph265depay ! "
                "queue name=hailo_preprocess_q_0 leaky=no max-size-buffers=10 max-size-bytes=0 max-size-time=0 ! "
                "decodebin ! queue leaky=downstream max-size-buffers=5 max-size-bytes=0 max-size-time=0 ! "
                "video/x-raw, format=I420 ! "
            ),
            "default": (
                f"filesrc location={self.video_source} name=src_0 ! "
                + QUEUE("queue_dec264")
                + "qtdemux ! h264parse ! avdec_h264 max-threads=2 ! "
                "video/x-raw, format=I420 ! "
            )
        }

        source_element = source_elements.get(self.source_type, source_elements["default"])
        source_element += (
            QUEUE("queue_scale")
            + "videoscale n-threads=2 ! "
            + QUEUE("queue_src_convert")
            + "videoconvert n-threads=3 name=src_convert qos=false ! "
            + f"video/x-raw, format={self.network_format}, width={self.network_width}, height={self.network_height}, pixel-aspect-ratio=1/1 ! "
        )

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
                + f"fpsdisplaysink video-sink={self.video_sink} name=hailo_display sync={self.sync} "
                f"text-overlay={self.options_menu.show_fps} signal-fps-measurements=true "
            )
        else:
            pipeline_string += (
                f"{QUEUE('queue_save')} "
                "videoconvert ! "
                "x264enc bitrate=6000 speed-preset=ultrafast tune=zerolatency ! "
                "matroskamux ! "
                f"filesink location={self.saving} "
            )

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
