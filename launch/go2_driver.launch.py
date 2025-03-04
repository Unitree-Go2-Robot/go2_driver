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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    use_camera = LaunchConfiguration('use_camera')
    use_tts = LaunchConfiguration('use_tts')
    use_vui = LaunchConfiguration('use_vui')
    use_odometry = LaunchConfiguration('use_odometry')
    use_joint_states = LaunchConfiguration('use_joint_states')
    use_services = LaunchConfiguration('use_services')
    use_switch_obtacles_avoidance = LaunchConfiguration('use_switch_obtacles_avoidance')

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

    composable_nodes = []

    composable_node = ComposableNode(
        package='go2_driver',
        plugin='go2_driver::Go2Driver',
        name='go2_driver',
        namespace='',
        parameters=[{'use_camera': use_camera,
                     'use_tts': use_tts,
                     'use_vui': use_vui,
                     'use_odometry': use_odometry,
                     'use_joint_states': use_joint_states,
                     'use_services': use_services,
                     'use_switch_obtacles_avoidance': use_switch_obtacles_avoidance}],
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

    ld = LaunchDescription()
    ld.add_action(declare_camera_cmd)
    ld.add_action(declare_tts_cmd)
    ld.add_action(declare_vui_cmd)
    ld.add_action(declare_odometry_cmd)
    ld.add_action(declare_joint_states_cmd)
    ld.add_action(declare_services_cmd)
    ld.add_action(declare_obtacles_avoidance_cmd)
    ld.add_action(container)

    return ld
