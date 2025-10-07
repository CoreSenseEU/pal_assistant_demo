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
import rclpy
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.action import ActionServer, GoalResponse, ActionClient
from rclpy.task import Future
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from tf2_ros import Buffer, TransformListener
import tf_transformations as t

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from tf2_geometry_msgs import do_transform_pose

import hri

from communication_skills.action import Ask
from interaction_skills.action import AskHumanForHelp
from navigation_skills.action import NavigateToPose
from attention_manager_msgs.srv import SetPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import time
import numpy as np
import math
import json

MAX_DISTANCE = 4
# MAX_DISTANCE = 1 # Used only for too far away case

POSITION_FILTER_COUNT = 5  # Number of positions to filter for stability
# POSITION_FILTER_COUNT = 100 # Only for unstable detection case


class SkillImpl(Node):
    """
    Implementation of ask_human_for_help.

    This is the main class for the skill. It is a ROS2 node that uses the
    lifecycle feature of ROS2 to manage its states.
    """

    def __init__(self) -> None:
        """Construct the node."""
        super().__init__('skill_ask_human_for_help')

        self.get_logger().info("Initialising...")
        self.diag_pub = None
        self.diag_timer = None

        self.timeout_acknowledgement = 45  # seconds for waiting for acknowledgment
        self.timeout_placing = 30  # seconds for waiting for placing the object
        self.timeout_find_user = 30  # seconds for waiting for user
        self.timeout_navigation = 30  # seconds for waiting for navigation

        self.approach_margin = 0.75
        self.position_filter_count = POSITION_FILTER_COUNT
        self.max_distance = MAX_DISTANCE
        self.position = None
        self.closest_std_rms = 0
        self.latest_position_goal = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Params
        self.declare_parameter('debug', False)
        self.declare_parameter('nav_fail', False)
        self.declare_parameter('detect_fail', False)
        self.declare_parameter('too_far', False)
        self.declare_parameter('unstable_detect', False)
        self.declare_parameter('high_variance', False)
        self.debug_mode = self.get_parameter('debug').get_parameter_value().bool_value
        self.nav_fail = self.get_parameter('nav_fail').get_parameter_value().bool_value
        self.detect_fail = self.get_parameter('detect_fail').get_parameter_value().bool_value
        self.too_far = self.get_parameter('too_far').get_parameter_value().bool_value
        self.unstable_detect = self.get_parameter('unstable_detect').get_parameter_value().bool_value
        self.high_variance = self.get_parameter('high_variance').get_parameter_value().bool_value
        self.get_logger().info(f"Debug mode is set to {self.debug_mode}")
        if self.nav_fail:
            self.get_logger().info("Navigation failure is forced for testing purposes.")
        if self.detect_fail:
            self.get_logger().info("Detection failure is forced for testing purposes.")
        if self.too_far:
            self.get_logger().info("Person detected will be too far away for testing purposes.")
        if self.unstable_detect:
            self.get_logger().info("Person detection will be unstable for testing purposes.")
        if self.high_variance:
            self.get_logger().info("Person detection variance will be very high for testing purposes.")

        self.get_logger().info('Skill ask_human_for_help started, but not yet configured.')

    def wait_for_user_response(self, timeout_duration):
        start_time = time.time()
        while time.time() - start_time < timeout_duration:
            if self.speech_confirmation is not None:
                if self.speech_confirmation:
                    return True
                else:
                    return False
            time.sleep(0.5)
        self.get_logger().warn("Timeout waiting for user response.")
        return False

    def ask_goal_response_callback(self, future):
        self.ask_goal_handle = future.result()
        if not self.ask_goal_handle.accepted:
            self.get_logger().info('Ask skill was rejected')
            return

        get_result_future = self.ask_goal_handle.get_result_async()
        get_result_future.add_done_callback(self.ask_get_result_callback)

    def ask_get_result_callback(self, future):
        result = future.result().result
        answers = result.answers
        try:
            answers_dict = json.loads(answers)
            if answers_dict.get("user_accepted_to_help_the_robot") is True:
                self.get_logger().info("User accepted to help.")
                self.speech_confirmation = True
            else:
                self.get_logger().info("User declined to help.")
                self.speech_confirmation = False
        except Exception as e:
            self.get_logger().error(f"Error parsing ask answers: {e}")
            self.speech_confirmation = False

    def acknowledge_goal_response_callback(self, future):
        self.acknowledge_goal_handle = future.result()
        if not self.acknowledge_goal_handle.accepted:
            self.get_logger().info('Ask skill was rejected')
            return

        get_result_future = self.acknowledge_goal_handle.get_result_async()
        get_result_future.add_done_callback(self.acknowledge_get_result_callback)

    def acknowledge_get_result_callback(self, future):
        result = future.result().result
        answers = result.answers
        try:
            answers_dict = json.loads(answers)
            if answers_dict.get("user_confirms_finishing_helping_the_robot") is True:
                self.get_logger().info("User confirmed that he/she has finished helping.")
                self.speech_confirmation = True
            else:
                self.get_logger().info("User did not confirm that he/she has finished helping.")
                self.speech_confirmation = False
        except Exception as e:
            self.get_logger().error(f"Error parsing ask answers: {e}")
            self.speech_confirmation = False

    def navigation_goal_response(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal was rejected!")
            return

        self.get_logger().info("Navigation goal accepted, waiting for completion...")

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future: Future):
        result = future.result().result
        self.get_logger().info(f"Navigation completed with result: {result}")
        if int(result.result.error_code) == 0:
            self.person_approached = True
        else:
            self.person_approached = False

    def send_navigation_goal(self, target_x, target_y, rotation_angle):
        goal_msg = NavigateToPose.Goal()

        pose = Pose()
        pose.position.x = target_x
        pose.position.y = target_y
        pose.position.z = 0.0

        self.latest_position_goal = pose.position
        # Convert rotation angle to quaternion
        quaternion = t.quaternion_from_euler(0.0, 0.0, rotation_angle)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',      # target frame
                'base_footprint',      # source frame
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            transformed_pose = do_transform_pose(pose, transform)

            self.get_logger().info(f'Pose in world frame: {transformed_pose}')
        except Exception as e:
            self.get_logger().warn(f'Could not transform pose: {e}')

        if self.debug_mode:
            transformed_pose = pose

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = transformed_pose.position.x
        goal_msg.pose.pose.position.y = transformed_pose.position.y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = transformed_pose.orientation.x
        goal_msg.pose.pose.orientation.y = transformed_pose.orientation.y
        goal_msg.pose.pose.orientation.z = transformed_pose.orientation.z
        goal_msg.pose.pose.orientation.w = transformed_pose.orientation.w

        self.latest_position_goal = transformed_pose.position

        # Behavior tree full path (if empty using "navigate_to_pose_w_replanning_and_recovery.xml")
        goal_msg.behavior_tree = ''

        self.send_goal_future = self.navigate_action_client.send_goal_async(
            goal_msg)
        self.send_goal_future.add_done_callback(self.navigation_goal_response)

    def detect_person(self, person_ids):
        self.log_to_memory("Looking for person",{"DetectState":True})

        if self.debug_mode:
            if self.detect_fail:
                self.person_detected = False
                return
            self.person_detected = True
            if self.unstable_detect:
                self.detection_stable = False
                return
            self.detection_stable = True
            self.closest_person_id = "debug_person"
            if self.high_variance:
                self.closest_std_rms = 2
            else:
                self.closest_std_rms = 0.01
            if self.too_far:
                self.filtered_closest_trans = [8,0]
            else:
                self.filtered_closest_trans = [1,0]
            return

        
        start_time = time.time()

        while not (self.person_detected and self.detection_stable):
            if time.time() - start_time > self.timeout_find_user:
                self.get_logger().warn("Timeout waiting for user. No valid persons found.")
                return
            else:
                persons_trans = {}
                closest_trans = None
                closest_distance = 100
                closest_person_id = None
                detected_person_within_provided_ids = False
                self.hri_listener.spin_all(5.0)
                try:
                    for person in self.hri_listener.persons:
                    # for body in self.hri_listener.bodies:
                        transform = None
                        print("iterating person " + person)
                        # if body in self.hri_listener.bodies:
                            # transform = self.hri_listener.bodies[body].transform
                            # person = body

                        if self.hri_listener.persons[person].body is not None:
                            if self.hri_listener.persons[person].body.valid:
                                transform = self.hri_listener.persons[person].body.transform
                        elif self.hri_listener.persons[person].face is not None:
                            if self.hri_listener.persons[person].face.valid:
                                transform = self.hri_listener.persons[person].face.transform
                        if transform is not None:
                            trans_x = transform.transform.translation.x
                            trans_y = transform.transform.translation.y
                            persons_trans[person] = [trans_x, trans_y]
                            if person in person_ids:
                                detected_person_within_provided_ids = True
                except BaseException as e:
                    self.get_logger().warn(
                        f"Error accessing person data: {e}")

                for person in persons_trans:
                    trans = persons_trans[person]
                    self.get_logger().info(
                        f"Person {person} detected with coordinates {trans[0]}, {trans[1]}")
                    if detected_person_within_provided_ids and person not in person_ids:
                        self.get_logger().info(
                            f"Skipping person {person} as it isn't in the provided person_ids list.")
                        continue
                    xy_norm = np.linalg.norm([trans[0], trans[1]], ord=2)
                    if xy_norm < closest_distance:
                        closest_trans = trans
                        closest_distance = xy_norm
                        closest_person_id = person

                if closest_trans is not None:
                    if closest_person_id not in self.closest_trans_dict:
                        self.closest_trans_dict[closest_person_id] = [closest_trans]
                    else:
                        if self.closest_trans_dict[closest_person_id][-1] != closest_trans:
                            self.closest_trans_dict[closest_person_id].append(closest_trans)
                for person_id in self.closest_trans_dict:
                    self.person_detected = True
                    if len(self.closest_trans_dict[person_id]) >= self.position_filter_count:
                        self.detection_stable = True
                        filtered_closest_trans_x = 0
                        filtered_closest_trans_y = 0
                        for trans in self.closest_trans_dict[person_id]:
                            filtered_closest_trans_x += trans[0]
                            filtered_closest_trans_y += trans[1]
                        filtered_closest_trans_x = filtered_closest_trans_x / len(
                            self.closest_trans_dict[person_id])
                        filtered_closest_trans_y = filtered_closest_trans_y / len(
                            self.closest_trans_dict[person_id])
                        self.filtered_closest_trans = [filtered_closest_trans_x,
                                                    filtered_closest_trans_y]
                        self.closest_person_id = closest_person_id

                        # Variance
                        sum_squared_diff_x = 0
                        sum_squared_diff_y = 0
                        for trans in self.closest_trans_dict[person_id]:
                            sum_squared_diff_x += (trans[0] - filtered_closest_trans_x) ** 2
                            sum_squared_diff_y += (trans[1] - filtered_closest_trans_y) ** 2

                        std_dev_x = math.sqrt(sum_squared_diff_x / len(self.closest_trans_dict[person_id]))
                        std_dev_y = math.sqrt(sum_squared_diff_y / len(self.closest_trans_dict[person_id]))
                        self.closest_std_rms = math.sqrt((std_dev_x ** 2 + std_dev_y ** 2) / 2)
                time.sleep(0.2)

    def navigate_to_user(self, person_ids):
        # see example script, for now tracking person
        self.get_logger().info("orient robot body towards the persons")
        start_time = time.time()
        self.closest_trans_dict = {}
        self.person_detected = False
        self.detection_stable = False

        self.detect_person(person_ids)

        if not (self.person_detected and self.detection_stable):
            request = SetPolicy.Request()
            request.policy = SetPolicy.Request.FOCUSED_SOCIAL
            self.future = self.attention_client.call_async(request)
            if not self.person_detected:
                self.log_to_memory("Error: No person detected",{"person_detected":False})
                self.log_to_memory("Person standard deviation RMS calculated",{"person_std_rms":0})
            else:
                self.log_to_memory("Person detected",{"person_detected":True})
                self.log_to_memory("Person standard deviation RMS calculated",{"person_std_rms":self.closest_std_rms})

            if not self.detection_stable:
                self.log_to_memory("Error: Detection unstable",{"detection_stable":False})
            else:
                self.log_to_memory("Person detection stable",{"detection_stable":True})

            self.log_to_memory("Error: Task Failure",{"SkillFailed":True})
            return False

        self.log_to_memory("Person detected",{"person_detected":True})
        self.log_to_memory("Person detection stable",{"detection_stable":True})
        self.log_to_memory("Person standard deviation RMS calculated",{"person_std_rms":self.closest_std_rms})
        filtered_closest_trans_x = self.filtered_closest_trans[0]
        filtered_closest_trans_y = self.filtered_closest_trans[1]
        xy_norm = np.linalg.norm([filtered_closest_trans_x, filtered_closest_trans_y], ord=2)
        self.log_to_memory("Person distance calculated",{"person_distance":xy_norm})
        xy_norm_with_margin = max(0, xy_norm - self.approach_margin)
        self.get_logger().info(
          f"{self.closest_person_id} x: {filtered_closest_trans_x}, y: {filtered_closest_trans_y}")
        

        if xy_norm > self.max_distance:
            # Person is too far away, don't consider them
            self.log_to_memory("Error: Task Failure",{"SkillFailed":True})
            return False

        elif xy_norm_with_margin > 0:
            target_x = filtered_closest_trans_x * xy_norm_with_margin / xy_norm
            target_y = filtered_closest_trans_y * xy_norm_with_margin / xy_norm
            rotation = math.atan(target_y / target_x)
            self.send_navigation_goal(
                target_x, target_y, rotation)
            self.log_to_memory("Approaching person",{"NavigateState":True})
        else:
            self.get_logger().info("Person is too close, no navigation needed.")
            self.person_approached = True

        request = SetPolicy.Request()
        request.policy = SetPolicy.Request.FOCUSED_SOCIAL
        self.future = self.attention_client.call_async(request)
        return True

    def run_skill(self, question_to_human, person_ids):
        self.log_to_memory("Running skill",{"InitialState":True})
        self.speech_confirmation = None
        self.person_approached = False

        self.log_to_memory("Setting initial parameters",{"person_approached":False})

        result = AskHumanForHelp.Result()

        # 1. Navigate to the user
        if not self.navigate_to_user(person_ids):
            self.get_logger().warn("Failed to find the person.")
            result.result.error_code = 255
            result.result.error_msg = "Failed to find the person."
            return result
        start_time = time.time()
        while not self.person_approached:
            if time.time() - start_time > self.timeout_navigation or self.nav_fail:
                self.get_logger().warn("Navigation timed out.")
                result.result.error_code = 255
                result.result.error_msg = "Navigation to user failed."
                self.log_to_memory("Error: Navigate to user failed",{"navigation_failure":True})
                self.log_to_memory("Error: Task Failure",{"SkillFailed":True})
                return result
            time.sleep(0.1)

        # Check navigation accuracy
        if self.latest_position_goal is not None and self.position is not None:
            p1 = self.latest_position_goal
            p2 = self.position
            nav_error = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            self.log_to_memory(f"Difference in current position and navigation goal: {nav_error}",{"robot_nav_err":nav_error})

        # 2. Ask the user if he/she can help
        self.log_to_memory("Navigation succeeded",{"navigation_failure":False})
        self.log_to_memory("Asking user for help",{"AskForHelpState":True})
        self.ask_goal_handle = None
        ask_goal = Ask.Goal()
        ask_goal.meta.priority = 4
        ask_goal.question = question_to_human
        ask_goal.answers_schema = json.dumps(
            {"user_accepted_to_help_the_robot": {"type": "boolean"}})
        self.send_ask_goal_future = self.ask.send_goal_async(ask_goal)
        self.send_ask_goal_future.add_done_callback(self.ask_goal_response_callback)
        self.get_logger().info("Asking user for help.")
        if not self.wait_for_user_response(self.timeout_acknowledgement):
            if self.ask_goal_handle is not None:
                self.get_logger().warn("Failed to get positive acknowledgment, cancelling goal.")
                self.ask_goal_handle.cancel_goal_async()
            result.result.error_code = 255
            result.result.error_msg = "User did not respond or declined to help."
            self.log_to_memory("Error: user did not help",{"user_accepted":False})
            self.log_to_memory("Error: Task Failure",{"SkillFailed":True})
            return result
        self.log_to_memory("User accepted",{"user_accepted":True})

        # 3. Wait for the user to place the object on the tray
        self.log_to_memory("Asking user for confirmation",{"WaitForConfirmationState":True})
        self.get_logger().info("Waiting for user to complete the task.")
        self.acknowledge_goal_handle = None
        self.speech_confirmation = None
        ask_goal = Ask.Goal()
        ask_goal.meta.priority = 4
        ask_goal.question = "Please confirm me when you have finished helping me."
        ask_goal.answers_schema = json.dumps(
            {"user_confirms_finishing_helping_the_robot": {"type": "boolean"}})
        self.send_acknowledge_goal_future = self.ask.send_goal_async(ask_goal)
        self.send_acknowledge_goal_future.add_done_callback(
            self.acknowledge_goal_response_callback)
        self.get_logger().info("Asking user for confrimation.")
        if not self.wait_for_user_response(self.timeout_placing):
            if self.acknowledge_goal_handle is not None:
                self.get_logger().warn("Failed to get positive acknowledgment, cancelling goal.")
                self.acknowledge_goal_handle.cancel_goal_async()
            result.result.error_code = 255
            result.result.error_msg = "Task not completed by the user."
            self.log_to_memory("Error:User did not confirm completion",{"user_confirmed":False})
            self.log_to_memory("Error: Task Failure",{"SkillFailed":True})
            return result

        self.log_to_memory("User confirmed",{"user_confirmed":True})
        self.get_logger().info("Task completed successfully")
        result.result.error_code = 0
        result.result.error_msg = "OK"
        self.log_to_memory("Task complete",{"SkillSucceeded":True})
        return result

    def on_request_goal(self, goal_handle):
        """Accept incoming goal if appropriate."""
        if self._state_machine.current_state[1] != "active":
            self.get_logger().error("Skill is not active yet, rejecting goal")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_request_exec(self, goal_handle):
        """Process incoming goal."""
        question_to_human = goal_handle.request.question_to_human
        person_ids = goal_handle.request.person_ids

        if not question_to_human:
            self.get_logger().error("Missing question_to_human to say in request.")
            goal_handle.abort()
            result = AskHumanForHelp.Result()
            result.result.error_code = 74
            result.result.error_msg = "Missing question_to_human to say"
            return result

        result = self.run_skill(question_to_human, person_ids)
        goal_handle.succeed()
        return result

    def on_new_amcl_pose(self,pose_msg):
        self.position = pose_msg.pose.pose.position

    #################################
    #
    # Lifecycle transitions callbacks
    #
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure the skill."""
        self.skill_server = ActionServer(self,
                                         AskHumanForHelp,
                                         "/skill/ask_human_for_help",
                                         goal_callback=self.on_request_goal,
                                         execute_callback=self.on_request_exec)

        # configure and start diagnostics publishing
        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 1)
        self.diag_timer = self.create_timer(1., self.publish_diagnostics)

        # Listening to current pose
        self._pose_sub_cbGroup = MutuallyExclusiveCallbackGroup()
        self._pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.on_new_amcl_pose,
            10, callback_group=self._pose_sub_cbGroup)

        # Episodic Memory
        self.ep_mem_pub = self.create_publisher(DiagnosticArray, '/episodic_memory', 1)

        self.get_logger().info("Skill ask_human_for_help is configured, but not yet active")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate the skill."""
        self.ask = ActionClient(self, Ask, '/skill/ask')
        while not self.ask.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("Ask skill not available.")

        self.attention_client = self.create_client(
            SetPolicy, '/attention_manager/set_policy')
        while not self.attention_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Attention manager service not available.")

        self.hri_listener = hri.HRIListener("hri_listener_node", auto_spin=False)
        self.hri_listener.set_reference_frame(
            "base_footprint")  # Adjust as needed

        self.navigate_action_client = ActionClient(
            self, NavigateToPose, '/skill/navigate_to_pose')
        while not self.navigate_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("Navigate to pose action not available.")

        self.get_logger().info("Skill ask_human_for_help is active and running")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop the timer to stop calling the `run` function (main task of your application)."""
        self.get_logger().info("Stopping skill...")

        self.get_logger().info("Skill ask_human_for_help is stopped (inactive)")
        return super().on_deactivate(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down ask_human_for_help  skill.')

        self.skill_server.destroy()

        self.destroy_timer(self.diag_timer)
        self.destroy_publisher(self.diag_pub)

        self.get_logger().info("Skill ask_human_for_help  finalized.")
        return TransitionCallbackReturn.SUCCESS

    #################################

    def publish_diagnostics(self):

        arr = DiagnosticArray()
        msg = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="/skills/skill_ask_human_for_help",
            message="skill ask_human_for_help is running",
            values=[
                KeyValue(key="Module name", value="skill_ask_human_for_help"),
                KeyValue(key="Current lifecycle state",
                         value=self._state_machine.current_state[1]),
            ],
        )

        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [msg]
        self.diag_pub.publish(arr)

    def log_to_memory(self,message:str,values:dict):
        key_vals = [KeyValue(key="Module name", value="skill_ask_human_for_help")]
        for key in values:
            key_vals.append(KeyValue(key=key,value=str(values[key])))

        arr = DiagnosticArray()
        msg = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="/skills/skill_ask_human_for_help",
            message=message,
            values=key_vals,
        )

        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [msg]
        self.ep_mem_pub.publish(arr)
