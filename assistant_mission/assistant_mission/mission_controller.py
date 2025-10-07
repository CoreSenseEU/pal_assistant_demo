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

import json
import string
import random
import time
import ast

from rclpy.action import ActionClient
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

import assistant_mission.colors as colors

from std_msgs.msg import String
from hri_actions_msgs.msg import Intent
from communication_skills.action import Say, Chat
from navigation_skills.action import NavigateToZone
from unique_identifier_msgs.msg import UUID
from interaction_skills.action import LookFor, AskHumanForHelp
from explainability_msgs.action import GenerateExplanation
from kb_msgs.srv import Manage
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# This is used to validate the skill parameters and expected results
SKILLS = {
    'look_for_human': {
        'params': ['patterns'],
        'result': ['found_entities']},
    'ask_human_for_help': {
        'params': ['question'],
        'result': None},
    'say': {
        'params': ['input'],
        'result': None},
    'navigate_to_zone': {
        'params': ['location'],
        'result': None},
    'explain': {
        'params': ['question'],
        'result': ['explanation']}
}

ROOM_LOCATIONS = ["kitchen", "dining_room", "living_room"]

PROMPT_CHAT_PLAN = """
# Instructions

You are a friendly robot named Tiago.
Your goal is to chat with the people around you and help them by deteecting user intents that will trigger the execution of some skills.
Your responses should be very brief and to the point.

You must always return an intent within the intents field. It must always be of type __intent_start_activity__. Within the data field, provide a dictionary, with the following fields:
    - goal: fill it with the text that the user has said
    - suggested_action_plan: It must be a list of dictionaries, each dictionary must have the following keys:
        - skill: the name of the skill to perform
        - params: the parameters of the skill

Here is the actual list of skills and parameters that you can perform within the suggested_action_plan:

- say:
    - Description: You can use this skill to talk to the user.
    - Parameters:
        - input: the text to say
    - Return values: None
    - Example:
    {
      "skill": "say",
      "params": {
        "input": "Hello, how are you?"
      }
    }
- ask_human_for_help:
    - Description: You can use this skill to approach a human, ask for help and wait until the human has helped you.
    - Parameters:
        - question: the text to ask
    - Return values: None
    - Example:
    {
      "skill": "ask_human_for_help",
      "params": {
        "question": "Can you prepare a coffee?"
      }
    }
- navigate_to_zone:
    - Description: You can use this skill to go to a specific location. Do not use it if you are already in that location.
    - Parameters:
        - location: the name of the location to navigate to. The available locations are: "kitchen", "dining_room" and "living_room".
    - Return values: None
    - Example:
    {
      "skill": 'navigate_to_zone',
      "params": {
        "location": "kitchen"
      }
    }
- explain:
    - Description: You can use this skill to get an explanation of why you have done something.
    - Parameters:
        - question: The question that the user asked about the decision or behaviour of the robot.
    - Return values: None
    - Example:
    {
        "skill": "explain",
        "params": {
            "question": "Why did you do that?"
        }
    }

You can only use these skills. Do not provide skills in the plan that are not in the list above. Use always the specific params for each skill.

You can access the return values of previous skills in the plan using the following syntax:
- '?skill_name.return_value_name' (e.g. '?explain.explanation').

Do not provide any output on the "response" or "results" fields. If you just want to say something to the user, use the "say" skill in the suggested_action_plan, and provide as "goal" the text that the user has said. You can only take and bring things around, if the user asks you to do something else, like washing the dishes or cleaning the floor, use the say skill to tell that you can't do it.

Take into account that the user text comes from a speech recognition system, so it may contain errors. You should try to understand the intention of the user and provide a response that is as close as possible to what the user wants. For example if the input talks about someone else, like "he go to the living room" it is actually asking you to go to the living room.

# Examples

- if the user says: 'Hello', you could return:
{
    "response": "",
    "intents": [
      {
      "intent":"__intent_start_activity__",
      "data": '{"goal": "Hello", "suggested_action_plan": [{'skill': 'say', 'params': {'input': 'Hello! How can I help you today?'}}]}'
      }]
    "results": None
}

- if the user says: 'get me a coffee' when you are in a specific room, e.g. 'dining_room', you could return (note that the last navigate_to skill is to return to the initial location after the coffee has been prepared):
{
    "response": "",
    "intents": [
      {
      "intent":"__intent_start_activity__",
      "data": '{"goal": "get me a coffe", "suggested_action_plan": [{'skill': 'say', 'params': {'input': 'Sure, I'll bring you a coffee'}}, {'skill': 'navigate_to_zone', 'params': {'location': 'kitchen'}}, {'skill': 'ask_human_for_help', 'params': {'question': 'Hello, can you prepare a coffee and put it on the my tray so I bring it to someone else?'}}, {'skill': 'navigate_to_zone', 'params': {'location': 'dining_room'}}, {'skill': 'say', 'params': {'input': 'Here is your coffee'}}]}'
      }],
    "results": {}
}

- if the user says: 'bring me a book' when you are in the 'kitchen', you could return:
{
    "response": "",
    "intents": [
      {
      "intent":"__intent_start_activity__",
      "data": '{"goal": "bring me a book", "suggested_action_plan": [{'skill': 'say', 'params': {'input': 'Sure, I'll do it'}}, {'skill': 'navigate_to_zone', 'params': {'location': 'living_room'}}, {'skill': 'ask_human_for_help', 'params': {'question': 'Hello, can pick a book and place it on the my tray?'}}, {'skill': 'navigate_to_zone', 'params': {'location': 'kitchen'}}, {'skill': 'say', 'params': {'input': 'Here is your book'}}]}'
      }],
    "results": {}
}

- if the user says: 'Take this to the kitchen table' when you are in the dining_room, you could return:
{
    "response": "",
    "intents": [
      {
      "intent":"__intent_start_activity__",
      "data": '{"goal": "take this to the kitchen table", "suggested_action_plan": [{'skill': 'say', 'params': {'input': 'No problem, I'll do it'}}, {'skill': 'navigate_to_zone', 'params': {'location': 'kitchen'}}, {'skill': 'ask_human_for_help', 'params': {'question': 'Hello, can you place the object on the kitchen table?'}}, {'skill': 'navigate_to_zone', 'params': {'location': 'dining_room'}}, {'skill': 'say', 'params': {'input': 'The object has been placed on the kitchen table'}}]}'
      }],
    "results": {}
}

- if the user says: 'Go to the kitchen', and you are not in that room, you could return:
{
    "response": "",
    "intents": [
      {
      "intent":"__intent_start_activity__",
      "data": '{"goal": "go to the kitchen", "suggested_action_plan": [{'skill': 'say', 'params': {'input': 'Sure, I'll go to the kitchen'}}, {'skill': 'navigate_to_zone', 'params': {'location': 'kitchen'}}]}'
      }],
    "results": {}
}

- if the user says: 'Go to the kitchen', and you are already in that room, you could return:
{
    "response": "",
    "intents": [
      {
      "intent":"__intent_start_activity__",
      "data": '{"goal": "go to the kitchen", "suggested_action_plan": [{'skill': 'say', 'params': {'input': 'Sorry, I'm already in the kitchen'}}]}'
      }],
    "results": {}
}

- if the user says: 'Why didn't you go to the kichen?', you could return:
{
    "response": "",
    "intents": [
      {
      "intent":"__intent_start_activity__",
      "data": '{"goal": "Why didn't you go to the kichen?", "suggested_action_plan": [{'skill': 'explain', 'params': {'question': 'Why didn't you go to the kichen?}}, {'skill': 'say', 'params': {'input': '?explain.explanation'}}]}'
      }],
    "results": {}
}

- if the user says: 'Why not?', you could return:
{
    "response": "",
    "intents": [
      {
      "intent":"__intent_start_activity__",
      "data": '{"goal": "Why not?", "suggested_action_plan": [{'skill': 'explain', 'params': {'question': 'Why not?'}}, {'skill': 'say', 'params': {'input': '?explain.explanation'}}]}'
      }],
    "results": {}
}

- if the user says: 'Why did you do that?', you could return:
{
    "response": "",
    "intents": [
      {
      "intent":"__intent_start_activity__",
      "data": '{"goal": "Why did you do that?", "suggested_action_plan": [{'skill': 'explain', 'params': {'question': 'Why did you do that?'}}, {'skill': 'say', 'params': {'input': '?explain.explanation'}}]}'
      }],
    "results": {}
}

- if the user says: 'Why couldn't you bring me a coffee?', you could return:
{
    "response": "",
    "intents": [
      {
      "intent":"__intent_start_activity__",
      "data": '{"goal": "Why couldn't you bring me a coffee?", "suggested_action_plan": [{'skill': 'explain', 'params': {'question': 'Why couldn't you bring me a coffee?'}}, {'skill': 'say', 'params': {'input': '?explain.explanation'}}]}'
      }],
    "results": {}
}
}

"""

PROMPT_CHAT_BUSY = """
# Instructions
You are a friendly robot. You are currently busy with a task and you cannot chat with the user.
You should let the user know that you are busy and that you will be available later.
Your responses should be brief and to the point.

Do not provide any output in the "intents", or "results" fields.

"""

# -----------------------------------------------------------------------------
# helper functions
# -----------------------------------------------------------------------------


def generate_uuid():
    uuid = []
    for i in range(len(uuid), 16):
        uuid.append(ord(random.choice(
            string.ascii_letters + string.digits)))
    return uuid


def uuid2string(uuid):
    return ''.join(chr(byte) for byte in uuid)


def check_string_list(input_value):
    if isinstance(input_value, str):
        try:
            evaluated = ast.literal_eval(input_value)
            if isinstance(evaluated, list):
                return evaluated
        except (ValueError, SyntaxError):
            pass
    return None


def validate_params(param, expected_param_type, dict_values):
    """
        Returns an evaluated and validated param, or error
    """

    # check if it's a var that requires evaluation
    if isinstance(param, str) and param[0] == '?':
        param = dict_values.get(param, None)

    # check if param is of exptected type
    if expected_param_type == 'string':
        if isinstance(param, str):
            return 0, param
        else:
            return -2, f"Param is not a string: {param}"

    elif expected_param_type == 'list':
        if isinstance(param, list):
            return 0, param
        elif isinstance(param, str):
            param = check_string_list(param)
            if param is None:
                return -2, f"Param is not a list: {param}"
            else:
                return 0, param
        else:
            return -2, f"Param is not a list: {param}"

    else:
        return 0, param

# -----------------------------------------------------------------------------
# classes
# -----------------------------------------------------------------------------


class Skill():
    def __init__(self, _type, _params) -> None:
        self.type = _type        # type of skill
        self.params = _params    # dictionary of params
        self.uuid = None         # uuid of the action
        self.handle = None       # action handle
        self.start_time = None  # time when the skill was started

    def __str__(self):
        return (f"Skill type: {self.type}\n"
                f"Skill params: {self.params}\n"
                f"Skill uuid: {self.uuid}\n"
                f"Skill handle: {self.handle}")


class Task():
    def __init__(self, _instruction, _name, _skill_seq, _params,
                 _last_location, _is_valid=True, _task_error_msg="") -> None:
        self.instruction = _instruction  # user instruction
        self.task_name = _name       # name of task
        self.task_id = uuid2string(generate_uuid())  # unique task id
        self.skill_seq = _skill_seq      # skills to achieve task
        self.next_skill_idx = 0  # index of next skill to execute
        self.skills_params = _params   # dictionary of params to evaluate
        self.initial_location = _last_location
        self.is_valid = _is_valid  # is the task valid?
        self.task_error_msg = _task_error_msg  # error message if the task is not valid

    def __str__(self):
        string = f"Task: {self.task_name} with id {self.task_id} \n"
        string += "Skills:\n"

        for s in self.skill_seq:
            string += f"{s}\n"
            string += "--\n"

        string += f"Params to evaluate: {self.skills_params}"
        return string


class MissionController(Node):

    def __init__(self) -> None:
        """Construct the node."""
        super().__init__('mission_assistant_mission')

        self.get_logger().info("Initialising...")

        # variables
        self._timer = None

        self.tasks = None
        self.current_task = None   # task being achieved
        self.current_task_info = None  # info about the current task
        self.current_skill = None   # skill under execution
        self.last_known_robot_location = None  # current location of the robot

        self.first_time_in_state = True
        self.state = 'idle'

        self.skill_timeout = 60.0  # seconds

        # subscribers
        self._intent_sub = None

        # publishers
        self._diagnostics_pub = None
        self._task_info_pub = None

        # available skills the robot can perform
        self._skill_say_client = None
        self._skill_look_for_human_client = None
        self._skill_ask_help_client = None
        self._skill_navigate_to_client = None
        self._skill_explain_client = None
        self._chat_busy_goal_handle = None

        self.srv_kb_manage = None

        self.get_logger().info('Node initialised. Ready to transition to '
                               'configure.')

    def on_intent(self, msg):

        self.get_logger().info("Received an intent %s with data %s" % (
            msg.intent, msg.data))
        try:
            data = json.loads(msg.data) if msg.data else {}
        except json.JSONDecodeError:
            self.get_logger().warn("The intent's data field is not a"
                                   f" valid json object:\n{msg.data}")
            data = {"raw": msg.data}
            return

        source = msg.source  # noqa: F841
        modality = msg.modality  # noqa: F841
        confidence = msg.confidence  # noqa: F841
        priority_hint = msg.priority  # noqa: F841

        time.sleep(1)

        if msg.intent == "__intent_start_activity__":

            task = self.validate_task_plan(data)

            if task and task.is_valid:
                self.log(f"Added task from intent: {task.task_name}")
                self.get_logger().info(str(task))
                chatbot_configuration = {
                    "prompt": PROMPT_CHAT_BUSY}
                if len(self.tasks) == 0:
                    chat_goal = Chat.Goal()
                    chat_goal.meta.caller = "mission_controller"
                    chat_goal.meta.priority = 2
                    chat_goal.role.configuration = json.dumps(chatbot_configuration)

                    self._chat_goal_future = self._skill_chat_busy_client.send_goal_async(
                        chat_goal)
                    self._chat_goal_future.add_done_callback(self.chat_busy_response_callback)
                self.tasks.append(task)

            else:
                # TODO: ask planner to create new plan?
                self.log("Invalid task")
                self.publish_internal_state(
                    {"mission_status":
                     "The task " + msg.intent + " has been rejected due to invalid data"})
                if task:
                    self.current_task_info = {
                        "task_id": task.task_id,
                        "instruction": task.instruction,
                        "task_status": "valid" if task.is_valid else "invalid",
                        "task_error_msg": task.task_error_msg,
                        "skill_sequence": [
                            {
                                "skill": skill.type,
                                "params": skill.params,
                                "status": "waiting",
                                "error_msg": ""
                            }
                            for skill in list(task.skill_seq)
                        ]
                    }

                self._task_info_pub.publish(
                    String(data=json.dumps(self.current_task_info)))

                goal = Say.Goal()
                goal.meta.priority = 5
                goal.input = "Sorry, I was not able to perform the task"
                self._skill_say_future = self._skill_say_client.send_goal_async(
                    goal)
        else:
            self.get_logger().warn("I don't know yet how to process intent "
                                   "<%s>" % msg.intent)
            self.publish_internal_state(
                {"mission_status":
                 "The intent " + msg.intent + " can't be processed"})

    def validate_task_plan(self, data):
        '''
            Validates the proposed plan in terms of:
            - valid skill
            - valid skill params (inputs)
            - valid skill result (outputs)
        '''
        task = None
        skills_params = {}
        skills = []
        is_valid = True
        task_error_msg = ""

        if "suggested_action_plan" not in data:
            self.log("No suggested action plan")
            is_valid = False
        if "goal" not in data:
            self.log("No task name")
            is_valid = False

        for skill in data['suggested_action_plan']:
            skill_name = skill.get('skill')
            self.log(f"skill: {skill_name}")
            params = skill.get('params')

            if skill_name in SKILLS:
                expected_params = SKILLS.get(skill_name).get('params', [])
                self.log(f"- expected_params: {expected_params}")
                self.log(f"- received_params: {params}")

                for p in expected_params:
                    if p not in params:
                        self.log("Mandatory params in skill are missing:"
                                 f"expected params: {expected_params}"
                                 f"provided params: {params}")
                        # return None
                        is_valid = False
                        task_error_msg += f"Mandatory params in skill {skill_name} are missing: {expected_params}. "

                    # For navigation skills, check if the location is valid
                    if skill_name == 'navigate_to_zone':
                        if isinstance(params.get('location'), str):
                            if params['location'] not in ROOM_LOCATIONS:
                                self.log(f"Invalid location {params['location']} for skill {skill_name}")
                                is_valid = False
                                task_error_msg += f"Invalid location {params['location']} for skill {skill_name}. Available locations are: {ROOM_LOCATIONS}. "
                        else:
                            self.log(f"Location param should be a string for skill {skill_name}")
                            is_valid = False
                            task_error_msg += f"Location param should be a string for skill {skill_name}. "

                    # add param that needs to be evaluated at runtime
                    # they should be type str and start with '?'
                    param_to_eval = params.get(p)
                    if (isinstance(param_to_eval, str) and
                            '?' == param_to_eval[0]):
                        skills_params[param_to_eval] = None

            else:
                self.log(f"Unknown skill {skill_name}")
                # self.log("Ignoring skill")
                is_valid = False
                task_error_msg += f"Unknown skill {skill_name}. "
            s = Skill(skill_name, params)
            skills.append(s)

        task = Task(data.get('goal', None), data.get('goal', None), skills,
                    skills_params, self.last_known_robot_location, is_valid, task_error_msg)

        return task

    def chat_busy_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal to start "busy" chat rejected')
            return

        self.get_logger().info('Goal to start "busy" chat accepted')
        self._chat_busy_goal_handle = goal_handle

    def on_set_goal(self, future):
        """ Generic set goal function for any task/skill """

        goal_handle = future.result()
        goal_id = uuid2string(goal_handle.goal_id.uuid)
        self.get_logger().info(f"[on_set_goal] Goal id: {goal_id}")

        if not goal_handle.accepted:
            self.get_logger().warn(f"{goal_handle}: Goal rejected")
            # TODO: HOW TO FOLLOW-UP IF THE GOAL IS REJECTED
            return

        self.current_skill.handle = goal_handle.get_result_async()
        # self.current_skill.handle.add_done_callback(self.on_goal_done)

    # def on_goal_done(self, future):
    #     goal_handle = future.result()
    #     result = goal_handle.result
    #     error_code = result.result.error_code
    #     error_msg = result.result.error_msg
    #     self.get_logger().info(f"[on_goal_done]: {error_code}, {error_msg}")

    def on_feedback(self, msg):
        self.get_logger().info("[on_feedback]: %s" % msg.feedback)

    def reason(self):
        """
        Picks next task to achieve.
        Simple approach: first in list, if none active
        """
        if self.tasks:
            if self.current_task:
                pass  # task in progress
            else:
                self.current_task = self.tasks[0]
                self.log(f"current task: {str(self.current_task.task_name)}")
                self.publish_internal_state(
                    {"mission_status":
                     "The task " + self.current_task.task_name + " has been started"})

                self.current_task_info = {
                    "task_id": self.current_task.task_id,
                    "instruction": self.current_task.instruction,
                    "task_status": "valid" if self.current_task.is_valid else "invalid",
                    "task_error_msg": self.current_task.task_error_msg,
                    "skill_sequence": [
                        {
                            "skill": skill.type,
                            "params": skill.params,
                            "status": "waiting",
                            "error_msg": ""
                        }
                        for skill in list(self.current_task.skill_seq)
                    ]
                }

                if "explain" not in [s.type for s in self.current_task.skill_seq]:
                    self._task_info_pub.publish(
                        String(data=json.dumps(self.current_task_info)))

    def start_next_skill(self):
        """ Launch new skill """

        if self.current_task.next_skill_idx >= len(self.current_task.skill_seq):
            self.current_skill = None
            return 1, "OK"
        else:
            self.current_skill = self.current_task.skill_seq[
                self.current_task.next_skill_idx]
            self.current_task.next_skill_idx += 1
            self.current_skill.start_time = time.time()
            self.current_task_info['skill_sequence'][
                self.current_task.next_skill_idx - 1]['status'] = 'Running'
            if "explain" not in [s.type for s in self.current_task.skill_seq]:
                self._task_info_pub.publish(
                    String(data=json.dumps(self.current_task_info)))

            if self.current_skill.type == 'say':
                uuid = generate_uuid()
                self.current_skill.uuid = uuid2string(uuid)

                input_text = self.current_skill.params.get('input', '')
                print("INPUT:",input_text)
                err_code, input_text = validate_params(
                    input_text,
                    'string',
                    self.current_task.skills_params)
                if err_code < 0:
                    return -2, f"Not valid text: {input_text}"

                goal = Say.Goal()
                goal.meta.priority = 5
                goal.input = input_text
                self._skill_say_future = self._skill_say_client.send_goal_async(
                    goal,
                    feedback_callback=self.on_feedback,
                    goal_uuid=UUID(uuid=uuid))

                self._skill_say_future.add_done_callback(self.on_set_goal)

            elif self.current_skill.type == 'look_for_human':
                uuid = generate_uuid()
                self.current_skill.uuid = uuid2string(uuid)

                goal = LookFor.Goal()
                patterns = self.current_skill.params.get('patterns', [])

                err_code, patterns = validate_params(
                    patterns,
                    'list',
                    self.current_task.skills_params)
                if err_code < 0:
                    return -2, f"Not valid recipient: {patterns}"
                goal.patterns = patterns

                self._skill_look_for_human_future = (
                    self._skill_look_for_human_client.send_goal_async(
                        goal,
                        feedback_callback=self.on_feedback,
                        goal_uuid=UUID(uuid=uuid)))

                self._skill_look_for_human_future.add_done_callback(
                    self.on_set_goal)

            elif self.current_skill.type == 'ask_human_for_help':
                uuid = generate_uuid()
                self.current_skill.uuid = uuid2string(uuid)

                goal = AskHumanForHelp.Goal()

                # person_ids = self.current_skill.params.get('person_ids', [])
                # err_code, recipient = validate_params(
                #     person_ids,
                #     'list',
                #     self.current_task.skills_params)
                # if err_code < 0:
                #     return -2, f"Not valid recipient: {recipient}"

                question = self.current_skill.params.get('question', '')
                err_code, _ = validate_params(
                    question,
                    'string',
                    self.current_task.skills_params)
                if err_code < 0:
                    return -2, f"Not valid text: {question}"
                goal.question_to_human = question

                self._skill_ask_help_future = (
                    self._skill_ask_help_client.send_goal_async(
                        goal,
                        feedback_callback=self.on_feedback,
                        goal_uuid=UUID(uuid=uuid)))

                self._skill_ask_help_future.add_done_callback(
                    self.on_set_goal)

            elif self.current_skill.type == 'navigate_to_zone':
                uuid = generate_uuid()
                self.current_skill.uuid = uuid2string(uuid)

                location = self.current_skill.params.get('location', '')

                err_code, location = validate_params(
                    location,
                    'string',
                    self.current_task.skills_params)
                if err_code < 0 or location == '':
                    return -2, f"Not valid location: {location}"

                if location not in ROOM_LOCATIONS:
                    msg = String()
                    msg.data = "Unavailable target zone to navigate: " + location
                    return -2, f"Not available location: {location}"

                self.log(f"Going to {location}")

                goal = NavigateToZone.Goal()
                goal.zone_name = location
                goal.succeed_on_zone_entry = True

                goal.meta.priority = 0
                goal.meta.caller = "mission_controller"

                self._skill_navigate_to_future = (
                    self._skill_navigate_to_client.send_goal_async(
                        goal,
                        feedback_callback=self.on_feedback,
                        goal_uuid=UUID(uuid=uuid)))

                self._skill_navigate_to_future.add_done_callback(
                    self.on_set_goal)

            elif self.current_skill.type == 'explain':
                uuid = generate_uuid()
                self.current_skill.uuid = uuid2string(uuid)

                goal = GenerateExplanation.Goal()
                question = self.current_skill.params.get('question', '')
                err_code, question = validate_params(
                    question,
                    'string',
                    self.current_task.skills_params)
                if err_code < 0 or question == '':
                    return -2, f"Not valid question: {question}"

                goal.question = question

                self._skill_explain_future = (
                    self._skill_explain_client.send_goal_async(
                        goal,
                        feedback_callback=self.on_feedback,
                        goal_uuid=UUID(uuid=uuid)))

                self._skill_explain_future.add_done_callback(
                    self.on_set_goal)
            else:
                self.log(f"Skill not implemented <{self.current_skill.type}>")
                return -1, "Not implemented skill"

        return 0, "OK"

    def retrieve_param_values(self, result):
        # only save values for those params that are expected
        if self.current_skill.type == 'look_for_human':
            if '?look_for_human.found_entities' in self.current_task.skills_params:
                if result.result.found_entities:
                    self.current_task.skills_params[
                        '?look_for_human.found_entities'
                    ] = result.result.found_entities
                else:
                    self.current_task.skills_params['?look_for_human.found_entities'] = []
            else:
                self.get_logger().info('Skill not providing values for params')
        elif self.current_skill.type == 'explain':
            if '?explain.explanation' in self.current_task.skills_params:
                if result.result.explanation:
                    self.current_task.skills_params[
                        '?explain.explanation'] = result.result.explanation
                else:
                    self.current_task.skills_params['?explain.explanation'] = ''
            else:
                self.get_logger().info('Skill not providing values for params')

    def execute(self):
        """
        State machine:
        - idle: waiting for a task to be selected
        - executing: execute sequence of skills to achieve the current task
        """

        if self.state == 'idle':
            if self.first_time_in_state:
                self.log('Waiting for the next task')
                self.first_time_in_state = False

            if self.current_task:
                error_code, error_msg = self.start_next_skill()

                if error_code == 0:
                    self.state = 'executing'
                    self.first_time_in_state = True

                elif error_code == 1:
                    self.log(f"Task achieved: {self.current_task.task_name}")
                    self.publish_internal_state(
                        {"mission_status":
                         "The task " + self.current_task.task_name + " has been finished"})
                    self.tasks.remove(self.current_task)
                    self.current_task = None
                    if len(self.tasks) == 0 and self._chat_busy_goal_handle is not None:
                        self._chat_busy_goal_handle.cancel_goal_async()

                else:
                    self.log("Can't achieve the task "
                             f"{self.current_task.task_name} because "
                             f"could not start the skill ({error_msg})")

                    self.publish_internal_state(
                        {"task_status":
                         "The skill " + self.current_skill.type +
                            " cannot be started due to " + error_msg})

                    goal = Say.Goal()
                    goal.meta.priority = 5
                    goal.input = "Sorry, I can't start the skill " + \
                        self.current_skill.type.replace('_', ' ') + "due to wrong parameters"
                    self._skill_say_future = self._skill_say_client.send_goal_async(
                        goal)

                    # TODO: For now, we abort the task. Replan?
                    self.publish_internal_state(
                        {"mission_status":
                         "The task " + self.current_task.task_name + " has been aborted"})
                    self.tasks.remove(self.current_task)
                    self.current_task = None
                    self.current_skill = None
                    if len(self.tasks) == 0 and self._chat_busy_goal_handle is not None:
                        self._chat_busy_goal_handle.cancel_goal_async()

        elif self.state == 'executing':
            if self.first_time_in_state:
                self.log(
                    f'Executing skill {self.current_skill.type} (uuid {self.current_skill.uuid})')

                self.publish_internal_state(
                    {"task_status":
                     "The skill " + self.current_skill.type + " has been started"})
                self.first_time_in_state = False

            if time.time() - self.current_skill.start_time > self.skill_timeout:
                self.log(f"Skill {self.current_skill.type} timed out after "
                         f"{self.skill_timeout} seconds")

                # Cancel the action
                if self.current_skill.handle is not None:
                    try:
                        self.current_skill.handle.cancel_goal_async()
                    except Exception as e:
                        self.get_logger().warn(f"Error cancelling skill: {e}")

                self.publish_internal_state(
                    {"task_status":
                     "The skill " + self.current_skill.type +
                     " has timed out after " + str(self.skill_timeout) +
                     " seconds"})

                self.current_task_info['skill_sequence'][
                    self.current_task.next_skill_idx - 1]['status'] = 'failed'
                self.current_task_info['skill_sequence'][
                    self.current_task.next_skill_idx - 1]['error_msg'] = \
                    "The skill has timed out"

                if "explain" not in [s.type for s in self.current_task.skill_seq]:
                    self._task_info_pub.publish(
                        String(data=json.dumps(self.current_task_info)))

                last_task_skills = self.current_task.skill_seq
                last_task_initial_location = self.current_task.initial_location
                last_skill_type = self.current_skill.type
                self.tasks.remove(self.current_task)
                if self._chat_busy_goal_handle is not None:
                    self._chat_busy_goal_handle.cancel_goal_async()
                self.current_task = None
                self.current_skill = None
                self.state = 'idle'
                self.first_time_in_state = True

                skill_explain_in_current_task = \
                    "explain" in [s.type for s in last_task_skills]

                if not skill_explain_in_current_task and \
                        last_skill_type != 'say':
                    skills = []
                    if last_task_initial_location is not None and \
                            last_skill_type != 'navigate_to_zone':
                        skills.append(
                            Skill('navigate_to_zone', {
                                "location": last_task_initial_location}))
                    skills.append(
                        Skill("say", {
                            "input": "Sorry, I was not able to perform the task "}))
                    recovery_task = Task("", "inform of task failure",
                                         skills, {"location": None, "input": None}, None)
                    self.tasks.append(recovery_task)
                    self.log(f"Added recovery task: {recovery_task.task_name}")
                    self.get_logger().info(str(recovery_task))

            elif self.current_skill.handle is None:
                pass

            elif self.current_skill.handle.done():
                result = self.current_skill.handle.result()
                # retrieve the action result (success or aborted)

                if result.status == 4:  # SUCCEEDED
                    # Retrieve the result if succeeded
                    error_code = result.result.result.error_code
                    error_msg = result.result.result.error_msg
                else:
                    error_code = -1
                    error_msg = "The skill has been aborted"

                self.log(f"<{self.current_skill.uuid}> finished with result:"
                         f"<{error_code} , {error_msg}>")

                if error_code != 0:
                    self.log('Removing skill and aborting task')
                    self.publish_internal_state({
                        "task_status":
                            "The skill " + self.current_skill.type +
                            " has failed with error " + error_msg,
                        "mission_status":
                            "The task " + self.current_task.task_name + " has been aborted"})

                    self.current_task_info['skill_sequence'][
                        self.current_task.next_skill_idx - 1]['status'] = 'failed'
                    self.current_task_info['skill_sequence'][
                        self.current_task.next_skill_idx - 1]['error_msg'] = error_msg
                    if "explain" not in [s.type for s in self.current_task.skill_seq]:
                        self._task_info_pub.publish(
                            String(data=json.dumps(self.current_task_info)))
                    last_task_skills = self.current_task.skill_seq
                    last_task_initial_location = self.current_task.initial_location
                    self.tasks.remove(self.current_task)
                    self.current_task = None
                    if self._chat_busy_goal_handle is not None:
                        self._chat_busy_goal_handle.cancel_goal_async()

                    skill_explain_in_current_task = \
                        "explain" in [s.type for s in last_task_skills]

                    if not skill_explain_in_current_task and \
                            self.current_skill.type != 'say':
                        skills = []
                        if last_task_initial_location is not None:
                            skills.append(
                                Skill('navigate_to_zone', {
                                    "location": last_task_initial_location}))
                        skills.append(
                            Skill("say", {
                                "input": "Sorry, I was not able to perform the task "}))
                        recovery_task = Task("", "inform of task failure", skills,
                                             {"location": None, "input": None}, None)
                        self.tasks.append(recovery_task)
                        self.log(f"Added recovery task: {recovery_task.task_name}")
                        self.get_logger().info(str(recovery_task))

                else:
                    if self.current_skill.type == 'navigate_to_zone':
                        self.last_known_robot_location = self.current_skill.params.get(
                            'location', '')
                        self.publish_internal_state({
                            "skill_status":
                                "The robot has arrived to the " +
                                self.current_skill.params.get('location', ''),
                            "task_status":
                                "The skill " + self.current_skill.type + " has been finished"})
                    else:
                        self.publish_internal_state(
                            {"task_status":
                             "The skill " + self.current_skill.type + " has been finished"})
                    self.current_task_info['skill_sequence'][
                        self.current_task.next_skill_idx - 1]['status'] = 'succeeded'
                    if "explain" not in [s.type for s in self.current_task.skill_seq]:
                        self._task_info_pub.publish(
                            String(data=json.dumps(self.current_task_info)))
                        # retrieve values for params that need to be evaluated
                    self.retrieve_param_values(result)

                self.current_skill = None
                self.state = 'idle'
            else:
                pass  # waiting until task is finished
        else:
            self.log('Unknown state {self.state}')

    def run(self) -> None:
        """Background task of your application."""

        self.reason()
        self.execute()

    def log(self, log_msg):
        msg = f"[{self.state}] {log_msg}"
        self.get_logger().info(colors.cyan(msg))

    def publish_internal_state(self, status_messages):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        values = []
        for status_type in status_messages:
            values.append(KeyValue(key=status_type, value=status_messages[status_type]))
        msg = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="/mission_controller",
            message="status update",
            values=values)
        arr.status.append(msg)
        self._diagnostics_pub.publish(arr)

    #########################################################################

    #################################
    #
    # Lifecycle transitions callbacks
    #
    #################################
    def _activate_communication_hub(self):
        service_name = '/communication_hub/change_state'
        client = self.create_client(ChangeState, service_name)

        while not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service {service_name} not available.")

        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_ACTIVATE
        client.call_async(request)

    def _reset_KB(self):
        request = Manage.Request()
        request.action = 'clear'
        request.parameters = ['keep_defaults']
        self.srv_kb_manage.call_async(request)
        self.get_logger().info("KB cleared")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure the node."""

        self.tasks = []
        self.current_task = None
        self.current_skill = None
        self.state = 'idle'
        self.first_time_in_state = True

        # self._activate_communication_hub()

        # skills: ROS 2 action clients
        self._skill_say_cbGroup = MutuallyExclusiveCallbackGroup()
        self._skill_say_client = ActionClient(
            self, Say, '/skill/say',
            callback_group=self._skill_say_cbGroup)
        while not self._skill_say_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'say' skill not yet available, "
                                   "waiting again...")

        self._skill_chat_and_plan_cbGroup = MutuallyExclusiveCallbackGroup()
        self._skill_chat_and_plan_client = ActionClient(
            self, Chat, '/skill/chat',
            callback_group=self._skill_chat_and_plan_cbGroup)

        self._skill_chat_busy_cbGroup = MutuallyExclusiveCallbackGroup()
        self._skill_chat_busy_client = ActionClient(
            self, Chat, '/skill/chat',
            callback_group=self._skill_chat_busy_cbGroup)

        self._skill_look_for_human_cbGroup = MutuallyExclusiveCallbackGroup()
        self._skill_look_for_human_client = ActionClient(
            self, LookFor, '/skill/look_for_human',
            callback_group=self._skill_look_for_human_cbGroup)

        self._skill_ask_help_cbGroup = MutuallyExclusiveCallbackGroup()
        self._skill_ask_help_client = ActionClient(
            self, AskHumanForHelp, '/skill/ask_human_for_help',
            callback_group=self._skill_ask_help_cbGroup)
        while not self._skill_ask_help_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'ask_human_for_help' skill not yet "
                                   "available, waiting again...")

        self._skill_navigate_to_cbGroup = MutuallyExclusiveCallbackGroup()

        self._skill_navigate_to_client = ActionClient(
            self, NavigateToZone, '/skill/navigate_to_zone',
            callback_group=self._skill_navigate_to_cbGroup)
        while not self._skill_navigate_to_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'navigate_to_zone' skill not yet "
                                   "available, waiting again...")

        self._skill_explain_cbGroup = MutuallyExclusiveCallbackGroup()
        self._skill_explain_client = ActionClient(
            self, GenerateExplanation, '/skill/explain',
            callback_group=self._skill_explain_cbGroup)
        while not self._skill_explain_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'explain' skill not yet "
                                   "available, waiting again...")

        self.srv_kb_manage = self.create_client(
            Manage, '/kb/manage')
        if not self.srv_kb_manage.wait_for_service(
                timeout_sec=1.0):
            self.get_logger().info(
                '/kb/manage service not available')

        # self._reset_KB()
        self.get_logger().info("Node configured. Ready to transition to "
                               "activate.")

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate the node."""

        self._intents_sub_cbGroup = MutuallyExclusiveCallbackGroup()

        # subscribers
        self._intents_sub = self.create_subscription(
            Intent,
            '/intents',
            self.on_intent,
            10, callback_group=self._intents_sub_cbGroup)

        # Call the Chat action and wait for the action to finish
        # TODO, this should be done somewhere else and handle cases when the chat stops?
        chatbot_configuration = {
            "prompt": PROMPT_CHAT_PLAN,
            "semantic_state_aggregator": {
                "filter_classes_kb": ["Robot", "dbr:Chair", "dbr:Book", "dbr:Bottle", "dbr:Table_(furniture)", "dbr:Laptop", "dbr:Smartphone", "Suitcase", "ZOI"],
                "filter_predicates_kb": ["isIn", "isOn", "sees"],
                "mission_state_enabled": True,
                "task_state_enabled": True,
                "skill_state_enabled": True,
                "filter_names_diagnostics": ["/mission_controller"],
                "filter_keys_diagnostics": ["_status", "task_status", "skill_status"]
            }}

        chat_goal = Chat.Goal()
        chat_goal.meta.caller = "mission_controller"
        chat_goal.meta.priority = 1
        chat_goal.role.configuration = json.dumps(chatbot_configuration)

        self._chat_goal_future = self._skill_chat_and_plan_client.send_goal_async(chat_goal)

        # Publisher for task info
        self._task_info_pub = self.create_publisher(
            String, '/task_info', 1)

        # Publisher for diagnostics
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)

        self.get_logger().info("Listening to incoming intents on the "
                               f"{self._intents_sub.topic_name} topic")

        timer_period = 0.1  # in sec
        self._timer = self.create_timer(timer_period, self.run)

        self.get_logger().info('Node activated and running.')

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the node."""
        self.get_logger().info("Stopping application")

        self.tasks = None
        self.current_task = None
        self.current_skill = None
        self.state = None
        self.first_time_in_state = None

        self._skill_say_client = None
        self._skill_chat_and_plan_client = None
        self._skill_chat_busy_client = None
        self._skill_look_for_human_client = None
        self._skill_ask_help_client = None
        self._skill_navigate_to_client = None
        self.srv_kb_manage = None

        # Cancel the chat goal
        if self._chat_goal_future and self._chat_goal_future.done():
            goal_handle = self._chat_goal_future.result()
            if goal_handle.accepted:
                self.get_logger().info('Canceling chat goal')
                self._chat_goal_future.cancel_goal_async()
        if self._chat_busy_goal_handle and self._chat_busy_goal_handle.done():
            goal_handle = self._chat_busy_goal_handle.result()
            if goal_handle.accepted:
                self.get_logger().info('Canceling chat busy goal')
                self._chat_busy_goal_handle.cancel_goal_async()

        self.get_logger().info('Node de-activated.')

        return super().on_deactivate(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown the node, after a shutting-down transition is requested."""
        # Cancel the chat goal
        if self._chat_goal_future and self._chat_goal_future.done():
            goal_handle = self._chat_goal_future.result()
            if goal_handle.accepted:
                self.get_logger().info('Canceling chat goal')
                self._chat_goal_future.cancel_goal_async()
        if self._chat_busy_goal_handle and self._chat_busy_goal_handle.done():
            goal_handle = self._chat_busy_goal_handle.result()
            if goal_handle.accepted:
                self.get_logger().info('Canceling chat busy goal')
                self._chat_busy_goal_handle.cancel_goal_async()

        self._skill_say_client.destroy()
        self._skill_chat_and_plan_client.destroy()
        self._skill_chat_busy_client.destroy()

        self.destroy_timer(self._timer)
        self.destroy_subscription(self._intents_sub)
        self.destroy_publisher(self._task_info_pub)
        self.destroy_publisher(self._diagnostics_pub)

        self.destroy_client(self.srv_kb_manage)

        self.get_logger().info('Shutting down node.')

        return TransitionCallbackReturn.SUCCESS
