# Copyright 2021 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description():
    socket_can_sender_node = LifecycleNode(package='ros2_socketcan',
                                           executable='socket_can_sender_node_exe',
                                           name='socket_can_sender',
                                           namespace=TextSubstitution(text=''),
                                           parameters=[{
                                               'interface': LaunchConfiguration('interface'),
                                               'timeout_sec':
                                                   LaunchConfiguration('timeout_sec'),
                                           }],
                                           output='screen')

    socket_can_sender_configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(socket_can_sender_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )

    socket_can_sender_activate_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(socket_can_sender_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('interface', default_value='can0'),
        DeclareLaunchArgument('timeout_sec', default_value='0.01'),
        DeclareLaunchArgument('auto_configure', default_value='true'),
        DeclareLaunchArgument('auto_activate', default_value='true'),
        socket_can_sender_node,
        socket_can_sender_configure_trans_event,
        socket_can_sender_activate_trans_event,
    ])
