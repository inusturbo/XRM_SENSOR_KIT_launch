import launch
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction
import yaml

def launch_setup(context, *args, **kwargs):
    camera_namespace="camera0"
    camera_type = LaunchConfiguration("camera_type").perform(context)
    output_topic = LaunchConfiguration("output_topic").perform(context)
    gpu_id = int(LaunchConfiguration("gpu_id").perform(context))
    mode = LaunchConfiguration("mode").perform(context)
    calib_image_directory = FindPackageShare("tensorrt_yolo").perform(context) + "/calib_image/"
    tensorrt_config_path = FindPackageShare('tensorrt_yolo').perform(context)+ "/config/" + LaunchConfiguration("yolo_type").perform(context) + ".param.yaml"
    camera_container_name = LaunchConfiguration("camera_container_name").perform(context)
    with open(tensorrt_config_path, "r") as f:
        tensorrt_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    container = ComposableNodeContainer(
        name=camera_container_name,
        namespace="/perception/object_detection",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="usb_cam_node",
                namespace=camera_namespace,
                output="screen",
                parameters=[{
                    "video_device": "/dev/video0",
                    "framerate": 30.0,
                    "io_method": "mmap",
                    "frame_id": "camera",
                    "pixel_format": "mjpeg2rgb",
                    "image_width": 1280,
                    "image_height": 720,
                    "camera_name": "camera0",
                    "camera_info_url": "package://usb_cam/config/camera_info.yaml",
                    "brightness": 50,
                    "contrast": 50,
                    "saturation": 50,
                    "sharpness": 80,
                    "gain": -1,
                    "auto_white_balance": true,
                    "white_balance": 4000,
                    "autoexposure": true,
                    "exposure": 100,
                    "autofocus": false,
                    "focus": -1,
                }]
            ),


            ComposableNode(
                namespace='/perception/object_recognition/detection',
                package="tensorrt_yolo",
                plugin="object_recognition::TensorrtYoloNodelet",
                name="tensorrt_yolo",
                parameters=[
                    {
                        "mode": mode,
                        "gpu_id": gpu_id,
                        "onnx_file": FindPackageShare("tensorrt_yolo").perform(context) +  "/data/" + LaunchConfiguration("yolo_type").perform(context) + ".onnx",
                        "label_file": FindPackageShare("tensorrt_yolo").perform(context) + "/data/" + LaunchConfiguration("label_file").perform(context),
                        "engine_file": FindPackageShare("tensorrt_yolo").perform(context) + "/data/"+ LaunchConfiguration("yolo_type").perform(context) + ".engine",
                        "calib_image_directory": calib_image_directory,
                        "calib_cache_file": FindPackageShare("tensorrt_yolo").perform(context) + "/data/" + LaunchConfiguration("yolo_type").perform(context) + ".cache",
                        "num_anchors": tensorrt_yaml_param['num_anchors'],
                        "anchors": tensorrt_yaml_param['anchors'],
                        "scale_x_y": tensorrt_yaml_param['scale_x_y'],
                        "score_threshold": tensorrt_yaml_param['score_threshold'],
                        "iou_thresh": tensorrt_yaml_param['iou_thresh'],
                        "detections_per_im": tensorrt_yaml_param['detections_per_im'],
                        "use_darknet_layer": tensorrt_yaml_param['use_darknet_layer'],
                        "ignore_thresh": tensorrt_yaml_param['ignore_thresh'],
                    }
                ],
                remappings=[
                    ("in/image", camera_namespace + "/image_raw"),
                    ("out/objects", output_topic),
                    ("out/image", output_topic + "/debug/image"),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
        ],
        return [container]
    )

def generate_launch_description():
    launch_arguments = []
    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )
    add_launch_arg("mode","")
    add_launch_arg("input_image","", description="input camera topic")
    add_launch_arg("camera_container_name","")
    add_launch_arg("yolo_type","", description="yolo model type")
    add_launch_arg("label_file","" ,description="tensorrt node label file")
    add_launch_arg("gpu_id","", description="gpu setting")
    add_launch_arg("use_intra_process", "", "use intra process")
    add_launch_arg("use_multithread", "", "use multithread")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )# # Copyright 2020 Tier IV, Inc. All rights reserved.
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction
import yaml

def launch_setup(context, *args, **kwargs):

    output_topic= LaunchConfiguration("output_topic").perform(context)

    image_name = LaunchConfiguration("input_image").perform(context)
    camera_container_name = LaunchConfiguration("camera_container_name").perform(context)
    camera_namespace = "/lucid_vision/" + image_name

    # tensorrt params
    gpu_id = int(LaunchConfiguration("gpu_id").perform(context))
    mode = LaunchConfiguration("mode").perform(context)
    calib_image_directory = FindPackageShare("tensorrt_yolo").perform(context) + "/calib_image/"
    tensorrt_config_path = FindPackageShare('tensorrt_yolo').perform(context)+ "/config/" + LaunchConfiguration("yolo_type").perform(context) + ".param.yaml"

    with open(tensorrt_config_path, "r") as f:
        tensorrt_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    camera_param_path=FindPackageShare("lucid_vision_driver").perform(context)+"/param/"+image_name+".param.yaml"
    with open(camera_param_path, "r") as f:
        camera_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]


    container = ComposableNodeContainer(
        name=camera_container_name,
        namespace="/perception/object_detection",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="lucid_vision_driver",
                plugin="ArenaCameraNode",
                name="arena_camera_node",
                parameters=[{
                    "camera_name": camera_yaml_param['camera_name'],
                    "frame_id": camera_yaml_param['frame_id'],
                    "pixel_format": camera_yaml_param['pixel_format'],
                    "serial_no": camera_yaml_param['serial_no'],
                    "camera_info_url": camera_yaml_param['camera_info_url'],
                    "fps": camera_yaml_param['fps'],
                    "horizontal_binning": camera_yaml_param['horizontal_binning'],
                    "vertical_binning": camera_yaml_param['vertical_binning'],
                    "use_default_device_settings": camera_yaml_param['use_default_device_settings'],
                    "exposure_auto": camera_yaml_param['exposure_auto'],
                    "exposure_target": camera_yaml_param['exposure_target'],
                    "gain_auto": camera_yaml_param['gain_auto'],
                    "gain_target": camera_yaml_param['gain_target'],
                    "gamma_target": camera_yaml_param['gamma_target'],
                    "enable_compressing": camera_yaml_param['enable_compressing'],
                    "enable_rectifying": camera_yaml_param['enable_rectifying'],
                }],
                remappings=[
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),

            ComposableNode(
                namespace='/perception/object_recognition/detection',
                package="tensorrt_yolo",
                plugin="object_recognition::TensorrtYoloNodelet",
                name="tensorrt_yolo",
                parameters=[
                    {
                        "mode": mode,
                        "gpu_id": gpu_id,
                        "onnx_file": FindPackageShare("tensorrt_yolo").perform(context) +  "/data/" + LaunchConfiguration("yolo_type").perform(context) + ".onnx",
                        "label_file": FindPackageShare("tensorrt_yolo").perform(context) + "/data/" + LaunchConfiguration("label_file").perform(context),
                        "engine_file": FindPackageShare("tensorrt_yolo").perform(context) + "/data/"+ LaunchConfiguration("yolo_type").perform(context) + ".engine",
                        "calib_image_directory": calib_image_directory,
                        "calib_cache_file": FindPackageShare("tensorrt_yolo").perform(context) + "/data/" + LaunchConfiguration("yolo_type").perform(context) + ".cache",
                        "num_anchors": tensorrt_yaml_param['num_anchors'],
                        "anchors": tensorrt_yaml_param['anchors'],
                        "scale_x_y": tensorrt_yaml_param['scale_x_y'],
                        "score_threshold": tensorrt_yaml_param['score_threshold'],
                        "iou_thresh": tensorrt_yaml_param['iou_thresh'],
                        "detections_per_im": tensorrt_yaml_param['detections_per_im'],
                        "use_darknet_layer": tensorrt_yaml_param['use_darknet_layer'],
                        "ignore_thresh": tensorrt_yaml_param['ignore_thresh'],
                    }
                ],
                remappings=[
                    ("in/image", camera_namespace + "/image_rect"),
                    ("out/objects", output_topic),
                    ("out/image", output_topic + "/debug/image"),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
        ],

    )
    return [container]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )
    add_launch_arg("mode","")
    add_launch_arg("input_image","", description="input camera topic")
    add_launch_arg("camera_container_name","")
    add_launch_arg("yolo_type","", description="yolo model type")
    add_launch_arg("label_file","" ,description="tensorrt node label file")
    add_launch_arg("gpu_id","", description="gpu setting")
    add_launch_arg("use_intra_process", "", "use intra process")
    add_launch_arg("use_multithread", "", "use multithread")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
