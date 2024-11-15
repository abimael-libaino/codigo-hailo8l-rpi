import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import argparse
import numpy as np
import setproctitle
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

# -----------------------------------------------------------------------------------------------
# User-defined class to be used in the callback function
# -----------------------------------------------------------------------------------------------
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.new_variable = 42
        self.frame_count = 0
        self.detections_data = []  # List to hold detection data for each frame

    def new_function(self):
        return "The meaning of life is: "
    
    def increment(self):
        self.frame_count += 1

    def get_count(self):
        return self.frame_count
    
    def set_frame(self, frame):
        # Store current frame (not used in this example, but can be useful)
        self.current_frame = frame
    
    def add_detection_data(self, frame_number, detection_count):
        self.detections_data.append({
            'frame': frame_number,
            'number_of_people': detection_count
        })

    def save_to_json(self, filename="detections.json"):
        with open(filename, 'w') as f:
            json.dump(self.detections_data, f, indent=4)

# -----------------------------------------------------------------------------------------------
# User-defined callback function
# -----------------------------------------------------------------------------------------------
def app_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK
        
    user_data.increment()
    frame_number = user_data.get_count()
    string_to_print = f"Frame count: {frame_number}\n"
    
    format, width, height = get_caps_from_pad(pad)

    frame = None
    if user_data.use_frame and format is not None and width is not None and height is not None:
        frame = get_numpy_from_buffer(buffer, format, width, height)

    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    
    detection_count = 0
    for detection in detections:
        label = detection.get_label()
        confidence = detection.get_confidence()
        if label == "person":
            string_to_print += f"Detection: {label} {confidence:.2f}\n"
            detection_count += 1
    
    if user_data.use_frame:
        cv2.putText(frame, f"Detections: {detection_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"{user_data.new_function()} {user_data.new_variable}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        user_data.set_frame(frame)

    user_data.add_detection_data(frame_number, detection_count)
    print(string_to_print)
    
    # Save the data to JSON file (optional: you might want to call this method periodically)
    user_data.save_to_json()

    return Gst.PadProbeReturn.OK

# -----------------------------------------------------------------------------------------------
# User GStreamer Application
# -----------------------------------------------------------------------------------------------
class GStreamerDetectionApp(GStreamerApp):
    def __init__(self, args, user_data):
        super().__init__(args, user_data)
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
                f"rtspsrc location={self.video_source} name=src_0  message-forward=true ! "
                + "rtph265depay ! "
                + "queue name=hailo_preprocess_q_0 leaky=no max-size-buffers=10 max-size-bytes=0 max-size-time=0 ! "
                + "decodebin ! queue leaky=downstream max-size-buffers=5 max-size-bytes=0 max-size-time=0 ! "

                " video/x-raw, format=I420 ! "
            )
        else:
            source_element = (
                f"filesrc location={self.video_source} name=src_0 ! "
                + QUEUE("queue_dec264")
                + " qtdemux ! h264parse ! avdec_h264 max-threads=2 ! "
                " video/x-raw, format=I420 ! "
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
            + QUEUE("queue_enc264")
            + "videoconvert ! x264enc ! mp4mux ! filesink location=test.mp4"
           
        )

        """ 
            
            + QUEUE("queue_videoconvertMP4")
            + "videoconvert n-threads=3 qos=false ! "
            + QUEUE("queue_hailo_save")
            + f"filesink location=test.mp4"

            
            gst-launch-1.0 filesrc location=resources/detection0.mp4 name=src_0 ! decodebin ! queue leaky=no max-size-buffers=30 max-size-bytes=0 max-size-time=0 ! videoscale qos=false n-threads=2 ! 
            video/x-raw, pixel-aspect-ratio=1/1 ! queue leaky=no max-size-buffers=30 max-size-bytes=0 max-size-time=0 ! videoconvert n-threads=2 qos=false ! queue leaky=no max-size-buffers=30 max-size-bytes=0 max-size-time=0 ! 
            hailonet hef-path=/home/airvision/hailo-rpi5-examples/basic_pipelines/../resources/yolov6n.hef batch-size=1 nms-score-threshold=0.3 nms-iou-threshold=0.45 output-format-type=HAILO_FORMAT_TYPE_FLOAT32 ! 
            queue leaky=no max-size-buffers=30 max-size-bytes=0 max-size-time=0 ! hailofilter so-path=/home/airvision/hailo-rpi5-examples/basic_pipelines/../resources/libyolo_hailortpp_post.so config-path=null qos=false !
            queue leaky=no max-size-buffers=30 max-size-bytes=0 max-size-time=0 ! hailooverlay qos=false ! queue leaky=no max-size-buffers=30 max-size-bytes=0 max-size-time=0 ! videoconvert ! x264enc ! mp4mux ! filesink location=video.mp4

        
        """

        print(pipeline_string)
        return pipeline_string

if __name__ == "__main__":
    user_data = user_app_callback_class()
    parser = get_default_parser()
    parser.add_argument(
        "--network",
        default="yolov6n",
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
