# Copyright (c) 2024 Intelligent Robotics Lab (URJC)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def autostart(context):

    if (LaunchConfiguration('autostart').perform(context) == 'True' or
            LaunchConfiguration('autostart').perform(context) == 'true'):
        configure_event = ExecuteProcess(
            cmd=[[FindExecutable(
                name='ros2'), ' lifecycle', ' set', ' /go2_driver', ' configure']],
            shell=True
        )

        activate_event = ExecuteProcess(
            cmd=[[FindExecutable(
                name='ros2'), ' lifecycle', ' set', ' /go2_driver', ' activate']],
            shell=True
        )

        event_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=configure_event,
                on_exit=[activate_event],
            )
        )

        return [configure_event, event_handler]

    return []


def generate_launch_description():

    camera_calibration_file = os.path.join(
        get_package_share_directory('go2_driver'),
        'config',
        'camera_calibration.yaml'
    )

    declare_camera_cmd = DeclareLaunchArgument(
        'use_camera',
        default_value='True',
        description='Use camera'
    )

    declare_tts_cmd = DeclareLaunchArgument(
        'use_tts',
        default_value='True',
        description='Use text to speech'
    )

    declare_vui_cmd = DeclareLaunchArgument(
        'use_vui',
        default_value='True',
        description='Use voice user interface'
    )

    declare_odometry_cmd = DeclareLaunchArgument(
        'use_odometry',
        default_value='True',
        description='Use odometry'
    )

    declare_joint_states_cmd = DeclareLaunchArgument(
        'use_joint_states',
        default_value='True',
        description='Use joint states'
    )

    declare_services_cmd = DeclareLaunchArgument(
        'use_services',
        default_value='True',
        description='Use services'
    )

    declare_obtacles_avoidance_cmd = DeclareLaunchArgument(
        'use_switch_obtacles_avoidance',
        default_value='True',
        description='Use obtacles avoidance'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically start the components'
    )

    composable_nodes = []

    composable_node = ComposableNode(
        package='go2_driver',
        plugin='go2_driver::Go2Driver',
        name='go2_driver',
        namespace='',
        parameters=[camera_calibration_file, {
            'use_camera': LaunchConfiguration('use_camera'),
            'use_tts': LaunchConfiguration('use_tts'),
            'use_vui': LaunchConfiguration('use_vui'),
            'use_odometry': LaunchConfiguration('use_odometry'),
            'use_joint_states': LaunchConfiguration('use_joint_states'),
            'use_services': LaunchConfiguration('use_services'),
            'use_switch_obtacles_avoidance': LaunchConfiguration('use_switch_obtacles_avoidance'),
        }],
    )

    composable_nodes.append(composable_node)

    container = ComposableNodeContainer(
        name='go2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    event_handler = RegisterEventHandler(
            OnProcessStart(
                target_action=container,
                on_start=[OpaqueFunction(function=autostart)],
            )
        )

    ld = LaunchDescription()
    ld.add_action(declare_camera_cmd)
    ld.add_action(declare_tts_cmd)
    ld.add_action(declare_vui_cmd)
    ld.add_action(declare_odometry_cmd)
    ld.add_action(declare_joint_states_cmd)
    ld.add_action(declare_services_cmd)
    ld.add_action(declare_obtacles_avoidance_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(container)
    ld.add_action(event_handler)

    return ld
