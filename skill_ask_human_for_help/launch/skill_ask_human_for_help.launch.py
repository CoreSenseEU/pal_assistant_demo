# Copyright (c) 2025 TODO. All rights reserved.
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
from launch.actions import EmitEvent, RegisterEventHandler, DeclareLaunchArgument
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.substitutions import LaunchConfiguration
from lifecycle_msgs.msg import Transition
from launch_pal import get_pal_configuration

def generate_launch_description():
    pkg = 'skill_ask_human_for_help'
    node = 'skill_ask_human_for_help'

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='False',
        description='Enable debug mode for simulator'
    )

    nav_fail_arg = DeclareLaunchArgument(
        'nav_fail',
        default_value='False',
        description='Force a navigation failure for testing purposes'
    )

    detect_fail_arg = DeclareLaunchArgument(
        'detect_fail',
        default_value='False',
        description='Force a detection failure for testing purposes'
    )

    too_far_arg = DeclareLaunchArgument(
        'too_far',
        default_value='False',
        description='Force a the test person to be too far away from the robot'
    )

    unstable_detect_arg = DeclareLaunchArgument(
        'unstable_detect',
        default_value='False',
        description='Forces the detection of a person to be unstable for testing purposes'
    )

    high_variance_arg = DeclareLaunchArgument(
        'high_variance',
        default_value='False',
        description='Forces the variance of the person detection to be high for testing purposes'
    )

    ld = LaunchDescription()

    ld.add_action(debug_arg)
    ld.add_action(nav_fail_arg)
    ld.add_action(detect_fail_arg)
    ld.add_action(too_far_arg)
    ld.add_action(unstable_detect_arg)
    ld.add_action(high_variance_arg)
    debug = LaunchConfiguration('debug')
    nav_fail = LaunchConfiguration('nav_fail')
    detect_fail = LaunchConfiguration('detect_fail')
    too_far = LaunchConfiguration('too_far')
    unstable_detect = LaunchConfiguration('unstable_detect')
    high_variance = LaunchConfiguration('high_variance')


    # automatically fetch the start parameters for this node,
    # using defaults installed with the skill, as well as possible
    # user overrides
    config = get_pal_configuration(pkg=pkg, node=node, ld=ld)
    config["parameters"].append({
        'debug': debug,
        'nav_fail': nav_fail,
        'detect_fail': detect_fail,
        'too_far': too_far,
        'unstable_detect': unstable_detect,
        'high_variance': high_variance,
    })

    node = LifecycleNode(
        package=pkg,
        executable='start_skill',
        namespace='',
        name=node,
        parameters=config["parameters"],
        remappings=config["remappings"],
        arguments=config["arguments"],
        output='both', emulate_tty=True,
    )

    ld.add_action(node)
    

    # automatically perform the lifecycle transitions to configure and activate
    # the skill at startup
    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=Transition.TRANSITION_CONFIGURE))

    ld.add_action(configure_event)

    activate_event = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_ACTIVATE))],
        handle_once=True))

    ld.add_action(activate_event)

    return ld
