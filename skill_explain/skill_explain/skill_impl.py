import json
import requests

from rclpy.action import ActionClient
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from rclpy.action import ActionServer, GoalResponse

from std_msgs.msg import String
from explainability_msgs.action import GenerateExplanation, GenerateComponentExplanation
from explainability_msgs.msg import Explanation
# from communication_skills.action import Say
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import time

# Default LLM configuration (will be overridden by ROS parameters if provided)
DEFAULT_LLM_MODEL = 'gpt-4.1-mini'
DEFAULT_LLM_HOST = 'https://api.openai.com'
DEFAULT_API_KEY = '<insert_your_api_key_here>'

COMPONENT_EXPLAINERS = {
    "component_explain_planner": {
        "description": "Explains high-level failures, such as when the robot generates an invalid plan or tries to perform an invalid skill.",
        "skills": [None],
        "explainee": "task planning"
    },
    "component_explain_navigation": {
        "description": "Explains errors in navigation, such as the robot not being able to reach a goal due to an obstacle, or the robot taking a strange or suboptimal path to its goal.",
        "skills": ["navigate_to_zone"],
        "explainee": "autonomous navigation system"
    },
    "component_explain_ask_human_for_help": {
        "description": "Explains errors in the 'ask_human_for_help' skill, where the robot identifies a human and asks them to help them by putting an object on the robot's tray. Errors could include non-compliance from the human, failed human detections, or social norm violations such as stopping too far away or too close to the human, or not looking at them.",
        "skills": ["ask_human_for_help"],
        "explainee": "ability to ask for help from humans"
    },
    "component_explain_say": {
        "description": "Explains failures with the 'say' skill, where the robot says a sentence. This could fail due to the say skill node being inactive, or if a timeout runs out.",
        "skills": ["say"],
        "explainee": "ability to speak"
    },
}


class SkillInfo:
    def __init__(self, skill_data: dict, current_time, get_logger):
        self.name = skill_data["skill"]
        self.params = skill_data["params"]
        self.status = skill_data["status"]
        self.error_msg = skill_data["error_msg"]
        self.creation_time = current_time
        self.update_time = current_time
        self.get_logger = get_logger

    def to_dict(self):
        return {
            "skill": self.name,
            "params": self.params,
            "status": self.status,
            "error_msg": self.error_msg
        }

    def needs_update(self, other_skill_info):
        if self.name == other_skill_info.name and self.params == other_skill_info.params:
            # This definitely refers to the same skill
            if self.status == other_skill_info.status and self.error_msg == other_skill_info.error_msg:
                # Nothing has changed, no update needed
                return False
            else:
                return True
        else:
            self.get_logger().warn(
                f"Attempting to update uncomparable skills: {self.name} with params {self.params} and {other_skill_info.name} with params {other_skill_info.params}")
            return False

    def update(self, other_skill_info, current_time):
        self.status = other_skill_info.status
        self.error_msg = other_skill_info.error_msg
        self.update_time = current_time


class TaskInfo:
    def __init__(self, data: dict, current_time, get_logger):
        self.instruction = data["instruction"]
        self.id = data["task_id"]
        self.task_status = data['task_status']
        self.task_error_msg = data['task_error_msg']
        self.skill_sequence = [SkillInfo(skill, current_time, get_logger)
                               for skill in data["skill_sequence"]]
        self.creation_time = current_time
        self.update_time = current_time
        self.get_logger = get_logger

    def to_dict(self):
        return {
            "instruction": self.instruction,
            "task_id": self.id,
            "task_status": self.task_status,
            "task_error_msg": self.task_error_msg,
            "skill_sequence": [skill.to_dict() for skill in self.skill_sequence],
        }

    def update(self, new_task_info, current_time):
        # Update time
        self.update_time = current_time
        self.task_status = new_task_info.task_status
        self.task_error_msg = new_task_info.task_error_msg

        # Update the relevant skill
        # NOTE: We assume the two task infos have the same skill sequences, just with updated statuses/error_msgs
        for my_skill, new_skill in zip(self.skill_sequence, new_task_info.skill_sequence):
            if my_skill.needs_update(new_skill):
                my_skill.update(new_skill, current_time)

                self.get_logger().info(
                    f"Updated skill {my_skill.name} with status '{my_skill.status}', error_msg '{my_skill.error_msg}' for task {self.id}")

    def skill_failed(self):
        '''
        Determines whether one of the skills failed and, if so, which one
        '''
        for skill, i in zip(self.skill_sequence, range(len(self.skill_sequence))):
            if skill.status.lower() == "failed":
                return True, i
        return False, None


class GenerateExplanationSkillImpl(Node):
    """
    Implementation of skill_explain.

    This is the main class for the skill. It is a ROS2 node that uses the
    lifecycle feature of ROS2 to manage its states.

    This basic skill template does not perform any particular task, and
    should be used as a starting point to implement your own skills.

    Don't forget to update accordingly the manifest of the skill, included in
    package.xml, to describe what your skill is actually doing.
    """

    def __init__(self) -> None:
        """Construct the node."""
        super().__init__('skill_explain')

        # Declare LLM parameters
        self.declare_parameter('llm_model', DEFAULT_LLM_MODEL)
        self.declare_parameter('llm_host', DEFAULT_LLM_HOST)
        self.declare_parameter('api_key', DEFAULT_API_KEY)

        # Attributes
        self.task_info_buffer = {}

        # Subscribers
        self._task_info_subscriber = None

        # Get LLM configuration from ROS parameters
        self.llm_model = (self.get_parameter('llm_model').get_parameter_value().string_value
                          or DEFAULT_LLM_MODEL)
        self.llm_host = (self.get_parameter('llm_host').get_parameter_value().string_value
                    or DEFAULT_LLM_HOST)
        
        self.api_key = (self.get_parameter('api_key').get_parameter_value().string_value or DEFAULT_API_KEY)

        self.get_logger().info(f"Using LLM model: {self.llm_model}, host: {self.llm_host}")

        # self.llm_client = OpenAI(
        #     base_url=llm_host + '/v1',
        #     api_key=api_key,
        # )

        # Component Explainers
        self.component_explainers = None

        # Declare ROS parameters
        # You should also add the same parameters to config/00-defaults.yml
        self.declare_parameter(
            'parameter_name', 'parameter_value',
            ParameterDescriptor(description='A parameter for the skill')
        )

        self.get_logger().info("Initialising...")
        # self._timer = None
        self._diag_pub = None
        self._diag_timer = None

        self._nb_requests = 0

        self.get_logger().info('Skill skill_explain started, but not yet configured.')

    def run_skill(self, question):
        """
        The skill for detecting the relevant failures, coordinating component explainers and generating the final explanation
        """
        # Initial Logging
        self.get_logger().info("...running the skill explain")

        # Fetch the last known plan
        # TODO: At a later stage, we may want to use the question to decide which plan to fetch
        # task_info = self.fetch_relevant_task_info(question, only_with_instructions=True)
        task_info = self.fetch_latest_task_info(only_with_instructions=True)

        # Decide which component explainer to call based on the queried plan
        result = self.query_component_explainer(question, task_info)

        return result

    def on_request_goal(self, goal_handle):
        """Accept incoming goal if appropriate."""
        if self._state_machine.current_state[1] != "active":
            self.get_logger().error("Skill is not active yet, rejecting goal")
            return GoalResponse.REJECT

        # Reject if task info buffer is empty
        if len(self.task_info_buffer) == 0:
            self.get_logger().error("No task info available, rejecting goal")
            return GoalResponse.REJECT

        self.get_logger().info("Accepted a new goal")
        return GoalResponse.ACCEPT

    def on_request_exec(self, goal_handle):
        self.get_logger().info(f"Current state: {self._state_machine.current_state[1]}")
        """Process incoming goal."""
        self.get_logger().info(
            f"Executing the skill with question: {goal_handle.request.question}")

        # perform request here
        list_of_explanations = self.run_skill(question=goal_handle.request.question)

        for explanation in list_of_explanations:
            self.get_logger().info(f"Explanation received: {explanation}")

        if len(list_of_explanations) == 0:
            self.get_logger().warn("No explanations received, returning default message")
            final_explanation = "I can't explain this right now, sorry."
        elif len(list_of_explanations) == 1:
            # If only one explanation, use it directly
            final_explanation = list_of_explanations[0]
            self.get_logger().info(f"Final explanation: {final_explanation}")
        else:
            # Combine all explanations into a single string
            # final_explanation = "because ".join(list_of_explanations)
            final_explanation = self.summarize_explanations(list_of_explanations)
            self.get_logger().info(f"Final explanation: {final_explanation}")

        self.get_logger().info("Goal executed successfully")
        goal_handle.succeed()

        return GenerateExplanation.Result(explanation=final_explanation)

    #################################
    #
    # Explanation Coordination Logic
    #
    def fetch_latest_task_info(self, only_with_instructions: bool = False):
        '''
        Fetches the latest task info from the task info buffer
        '''
        if len(list(self.task_info_buffer.keys())) == 1:
            # Only one task info, just return it
            return self.task_info_buffer[list(self.task_info_buffer.keys())[0]]
        else:
            # Need to check times
            latest_task = next(iter(self.task_info_buffer.values()))

            for task_info in self.task_info_buffer.values():
                current_time_ns = task_info.update_time.sec * 1e9 + task_info.update_time.nanosec
                latest_time_ns = latest_task.update_time.sec * 1e9 + latest_task.update_time.nanosec
                if current_time_ns > latest_time_ns:
                    if only_with_instructions:
                        if task_info.instruction is not None and task_info.instruction != "":
                            latest_task = task_info
                    else:
                        latest_task = task_info

            self.get_logger().info(f"Explaining task {latest_task.id}")
            return latest_task

    def fetch_relevant_task_info(self, question: str, only_with_instructions: bool = False):
        '''
        Fetches the relevant task info based on the question
        '''
        # Append task infos from the buffer into a dict
        task_infos = {task_info.id: task_info.to_dict()
                      for task_info in self.task_info_buffer.values()}

        # prepare the prompt for the LLM to select the relevant task info

        task_info_selection_prompt_system = f"""
        You're given a list of task infos related to a robot's activities, which include instructions, unique task IDs, and detailed skill sequences. 
        Each skill sequence includes the skill name, parameters, and execution status.
        Act as a Task Selector, utilizing your analytical skills to match user questions with the most relevant task info.
        Your task is to analyze the provided task infos to determine which one is the most relevant to the user’s question.
        The most relevant task info is the one that best matches the information in the question.
        For example, if the question is about a specific task that the robot was performing, you should select the task info that contains the instruction related to that task.
        Output format: Answer: [task_id]\n
        Where [task_id] is the ID of the task info that best matches the question that seeks an explanation.
        """
        task_info_selection_prompt_user = f"""
        Task infos: {str(task_infos)}
        User question: {question}
        """

        # Publish feedback that the LLM is selecting the task info
        feedback_msg = String()
        feedback_msg.data = f"LLM is selecting the task info for question: {question}"
        self.get_logger().info(feedback_msg.data)

        headers = {}
        if self.api_key:
            headers['Authorization'] = f"Bearer {self.api_key}"
        response = requests.post(
            f'{self.llm_host}/v1/chat/completions',
            json={
                'model': self.llm_model,
                'messages': [
                    {
                        'role': 'system',
                        'content': task_info_selection_prompt_system,
                    },
                    {
                        'role': 'user',
                        'content': task_info_selection_prompt_user,
                    }],
                'temperature': 0.0,
                'stream': False},
            headers=headers)
        if response.status_code != requests.codes.ok:
            raise RuntimeError(
                f'Ollama server response [{response.status_code}]: {response.text}')
        response_json = response.json()['choices'][0]

        response = str(response_json['message']['content']).strip()

        # # prompt and parse the response
        # response = self.llm_client.chat.completions.create(
        #     model=self.llm_model,
        #     temperature=0.0,
        #     messages=[
        #         {
        #             'role': 'system',
        #             'content': task_info_selection_prompt_system,
        #         },
        #         {
        #             'role': 'user',
        #             'content': task_info_selection_prompt_user,
        #         },
        #     ])
        # response = response.choices[0].message.content

        for line in response.split('\n'):
            if 'Answer:' in line:
                task_id = line[len('Answer:'):].strip()
                # LLM sometimes adds extra characters, so we remove them
                task_id = ''.join(e for e in task_id if e.isalnum())
                if task_id in task_infos:
                    self.get_logger().info(
                        f"Selected task info with ID {task_id} based on question '{question}'")
                    return self.task_info_buffer[task_id]

        self.get_logger().warn(f"LLM selected invalid task ID, defaulting to latest task info")
        return self.fetch_latest_task_info(only_with_instructions=only_with_instructions)

    def query_component_explainer(self, question: str, task_info: TaskInfo):
        self.component_explain_responded = False
        skill_failure, skill_index = task_info.skill_failed()
        if skill_failure:
            # One of the skills failed, should query the corresponding component explainer

            # Identify the correct component explainer
            component_explainer = "component_explain_planner"  # Default
            for component in COMPONENT_EXPLAINERS:
                if task_info.skill_sequence[skill_index].name in COMPONENT_EXPLAINERS[component]["skills"]:
                    component_explainer = component
                    break

            self.get_logger().info(
                f"Skill failure detected, passing task {task_info.id} to {component_explainer}")
            self.get_logger().info(
                f"Skill explain selected the {component_explainer} component explainer")

            # Route request to component explainer
            # TODO: Process the message accordingly
            prev_skill_time = task_info.skill_sequence[skill_index -
                                                       1].update_time if skill_index != 0 else task_info.creation_time
            curr_skill_time = task_info.skill_sequence[skill_index].update_time

            json_data = {
                "question": question,
                "time_range_start": f"{prev_skill_time.sec}.{prev_skill_time.nanosec}",
                "time_range_end": f"{curr_skill_time.sec}.{curr_skill_time.nanosec}",
                "task_info": task_info.to_dict()
            }

            result = self.send_goal_to_component_explainer(component_explainer, json_data)

            # skill_name = task_info.skill_sequence[skill_index].name.replace("_"," ")

            # list_of_explanations = [f"I failed to perform the skill {skill_name}"] # uncomment to enable level 0 explanations
            list_of_explanations = []
            for explanation in result.explanations:
                list_of_explanations.append(explanation.explanation)

            return list_of_explanations
        elif task_info.task_status.lower() == "invalid":
            # The task is invalid, so we need to explain why
            component_explainer = "component_explain_planner"
            self.get_logger().info(
                f"Task {task_info.id} is invalid, passing to {component_explainer}")
            self.get_logger().info(
                f"Skill explain selected the {component_explainer} component explainer")
            # Route request to component explainer
            json_data = {
                "question": question,
                "time_range_start": f"{task_info.creation_time.sec}.{task_info.creation_time.nanosec}",
                "time_range_end": f"{task_info.update_time.sec}.{task_info.update_time.nanosec}",
                "task_info": task_info.to_dict()
            }
            goal = GenerateComponentExplanation.Goal()
            goal.json_data = json.dumps(json_data)
            result = self.send_goal_to_component_explainer(component_explainer, json_data)
            list_of_explanations = []
            for explanation in result.explanations:
                list_of_explanations.append(explanation.explanation)
            return list_of_explanations
        else:
            # None of the skills failed, need to use the question to determine the correct component explainer
            # Choose the component explainer
            component_explainer = self.select_component_explainer(question, task_info)
            self.get_logger().info(
                f"Skill explain selected the {component_explainer} component explainer")

            if component_explainer in self.component_explainers:
                json_data = {
                    "question": question,
                    "time_range_start": f"{task_info.creation_time.sec}.{task_info.creation_time.nanosec}",
                    "time_range_end": f"{task_info.update_time.sec}.{task_info.update_time.nanosec}",
                    "task_info": task_info.to_dict()
                }

                goal = GenerateComponentExplanation.Goal()
                goal.json_data = json.dumps(json_data)

                result = self.send_goal_to_component_explainer(component_explainer, json_data)

                # Uncomment to enable level 0 explanations
                # list_of_explanations = [f"I had issues with my {COMPONENT_EXPLAINERS[component_explainer]['explainee']}"]
                list_of_explanations = []
                for explanation in result.explanations:
                    list_of_explanations.append(explanation.explanation)

                return list_of_explanations
            else:
                list_of_explanations = [
                    f"Component explainer {component_explainer} not yet implemented"]

                return list_of_explanations

    def select_component_explainer(self, question: str, task_info: TaskInfo):
        # Detect the component from the question

        # Possible improvement: Trim the list using only the relevant components (e.g. remove ask_human_for_help if it was not in the plan)

        component_descriptions = [
            f"{component} - {COMPONENT_EXPLAINERS[component]['description']}" for component in COMPONENT_EXPLAINERS]

        component_classification_prompt = f"""

        # Instructions
        You are a classification model in a robot explainability system.
        In this system, the robot performs simple tasks around the house for a user.
        The robot's behaviour is described by a list of skills.
        Your job is to select which component a question should be forwarded to, based on the content of the question.
        Some components are tailored to particular skills and others to high-level planning.
        You must answer in only one word, which is the name of the component.

        Here is the list of possible components and a description of each one: {component_descriptions}.

        These are the only components you can select. Do not select a component that does not appear in this list. Only answer with one word, the name of the component.

        Take into account that the user text comes from a speech recognition system, so it may contain errors. You should try to understand the intention of the user and select the correct component accordingly.

        # Examples

        - if the user says 'Why didn't you do anything?'
        - your response: 'component_explain_planner'

        - if the user says 'Why did you take that path?'
        - your response: 'component_explainer_navigation'

        - if the user says 'Why did you stop so far away from me?'
        - your response: 'component_explain_ask_human_for_help'

        - if the user says 'Why didn't you do what I asked you to do?'
        - your response: 'component_explain_planner'

        - if the user says 'Why did you take so long to get to the living room?'
        - your response: 'component_explainer_navigation'

        - if the user says 'Why did you stop so close to me?'
        - your response: 'component_explain_ask_human_for_help'

        - if the user says 'Why did you go to the kitchen, when the bottle was in the living room?'
        - your response: 'component_explain_planner'

        - if the user says 'Why did you spin around like that?'
        - your response: 'component_explainer_navigation'

        - if the user says 'Why didn't you face me when you spoke?'
        - your response: 'component_explain_ask_human_for_help'

        - if the user says 'Why did you bring the book to the dining room?'
        - your response: 'component_explain_planner'

        - if the user says 'Why couldn't you do what I asked you to do?'
        - your response: 'component_explain_planner'

        - if the user says 'Why did you try to do something you aren't programmed to do?'
        - your response: 'component_explain_planner'

        - if the user says 'Why did you behave so strangely?'
        - your response: 'component_explain_planner'

        - if the user says 'Why did you do that?'
        - your response: 'component_explain_planner'
        
        """

        headers = {}
        if self.api_key:
            headers['Authorization'] = f"Bearer {self.api_key}"
        response = requests.post(
            f'{self.llm_host}/v1/chat/completions',
            json={
                'model': self.llm_model,
                'messages': [
                    {
                        'role': 'system',
                        'content': component_classification_prompt,
                    },
                    {
                        'role': 'user',
                        'content': question,
                    }],
                'temperature': 0.0,
                'stream': False},
            headers=headers)
        if response.status_code != requests.codes.ok:
            raise RuntimeError(
                f'Ollama server response [{response.status_code}]: {response.text}')
        response_json = response.json()['choices'][0]
        selected_component_explainer = str(response_json['message']['content']).strip()

        # response = self.llm_client.chat.completions.create(
        #     model=self.llm_model,
        #     temperature=0.0,
        #     messages=[
        #         {
        #             'role': 'system',
        #             'content': component_classification_prompt,
        #         },
        #         {
        #             'role': 'user',
        #             'content': question,
        #         },
        #     ])

        # selected_component_explainer = response.choices[0].message.content
        if selected_component_explainer in COMPONENT_EXPLAINERS:
            return selected_component_explainer
        else:
            # The LLM did not choose a valid intent, default to planner explainer
            default_component = "component_explain_planner"
            self.get_logger().warn(
                f"LLM selected {selected_component_explainer}, which is invalid. Defaulting to {default_component}")
            return default_component

    def summarize_explanations(self, explanations: list) -> str:
        """
        Summarizes the explanations into a single string.
        Uses an LLM prompt to summarize the explanations.
        """
        # Prepare the prompt for the LLM to summarize the explanations
        ans_key = "Summarized Explanation:"
        summarization_prompt_system = f"""You are a summarization model in a robot explainability system. Your goal is to clearly and concisely explain the robot’s behavior in a single sentence from the robot’s first-person perspective to help users easily understand why it acted as it did.

        Requirements:
        
        1. Use simple, user-friendly language in **first person**, focused on clarity.
        2. Do not suggest the user ask for more information.
        3. Avoid apologies; provide straightforward, factual explanations.
        4. Clearly state the specific causes behind the robot’s behavior.
        5. Don't speculate beyond the information provided in the explanations.
        
        Output format: {ans_key} [summarized_explanation]
        """
        # Join the explanations into a single string
        raw_explanations = " This happened because ".join(explanations)

        summarization_prompt_user = f"""
        Explanations: {raw_explanations}
        """

        headers = {}
        if self.api_key:
            headers['Authorization'] = f"Bearer {self.api_key}"
        response = requests.post(
            f'{self.llm_host}/v1/chat/completions',
            json={
                'model': self.llm_model,
                'messages': [
                    {
                        'role': 'system',
                        'content': summarization_prompt_system,
                    },
                    {
                        'role': 'user',
                        'content': summarization_prompt_user,
                    }],
                'temperature': 0.0,
                'stream': False},
            headers=headers)
        if response.status_code != requests.codes.ok:
            raise RuntimeError(
                f'Ollama server response [{response.status_code}]: {response.text}')
        response_json = response.json()['choices'][0]
        response = str(response_json['message']['content']).strip()

        # # Send the prompt to the LLM
        # response = self.llm_client.chat.completions.create(
        #     model=self.llm_model,
        #     temperature=0.0,
        #     messages=[
        #         {
        #             'role': 'system',
        #             'content': summarization_prompt_system,
        #         },
        #         {
        #             'role': 'user',
        #             'content': summarization_prompt_user,
        #         },
        #     ])

        # Parse the response
        # response = response.choices[0].message.content

        for line in response.split('\n'):
            if ans_key in line:
                return line[len(ans_key):].strip()
        return response

    def on_new_task_info(self, msg):
        '''
        The callback when the subscriber receives a new task info

        Expects msg.data to be a json string with the following format

        {
            instruction:string, # The instruction by the user which was used to generate the plan
            task_id:int, # A task id, used to ensure that updates to existing plans can be made to the right plan
            skill_sequence:[
                    {
                        skill_name: {
                                params:{...}, # The key:values passed as parameters to the skill
                                status:string, # The status returned by the skill
                                error_msg:string # The error message returned by the skill
                            }
                    }
                ]
        }
        '''
        # Logging
        self.get_logger().info(f"Received a new plan with data {msg.data}")

        # Process the message
        try:
            data = json.loads(msg.data) if msg.data else {}
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid json object received: \n{msg.data}")
            return

        # Get the current time
        current_time = self.get_clock().now().to_msg()

        # Create a task info object
        task_info = TaskInfo(data, current_time, self.get_logger)

        # Update the task info buffer
        if task_info.id in self.task_info_buffer:
            # The task already exists, update it with new information
            self.task_info_buffer[task_info.id].update(task_info, current_time)
        else:
            # The task does not exist, insert it into the buffer
            self.task_info_buffer[task_info.id] = task_info

    def component_explain_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by component explain')
            return

        self.get_logger().info('Goal accepted by component explain')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.component_get_result_callback)

    def component_get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        self.component_explain_responded = True

    def send_goal_to_component_explainer(self, component_explainer: str, json_data: dict):
        goal = GenerateComponentExplanation.Goal()
        goal.json_data = json.dumps(json_data)

        client = self.component_explainers[component_explainer]

        self.get_logger().info(f'Sending goal to {component_explainer}')

        # Wait for the action server to be available
        timeout_count = 0
        while not client.wait_for_server(timeout_sec=1.0):
            timeout_count += 1
            if timeout_count >= 3:
                self.get_logger().warn(f'{component_explainer} not available after 3 seconds')
                result = GenerateComponentExplanation.Result()
                explanation = Explanation()
                explanation.explanation = "Sorry, I can't explain this right now."
                result.explanations = [explanation]
                result.error_msg = "component explainer not available"
                return result
            self.get_logger().info(f'{component_explainer} not available, waiting...')

        # Send the goal to the action server
        future = client.send_goal_async(goal)
        future.add_done_callback(self.component_explain_response_callback)

        # Wait for the result with timeout
        timeout_duration = 10.0  # 10 seconds timeout
        start_time = self.get_clock().now()

        while not self.component_explain_responded:
            current_time = self.get_clock().now()
            elapsed_time = (current_time - start_time).nanoseconds / 1e9

            if elapsed_time >= timeout_duration:
                self.get_logger().warn(f"Timeout waiting for result from {component_explainer}")
                result = GenerateComponentExplanation.Result()
                explanation = Explanation()
                explanation.explanation = "Sorry, I can't explain this right now."
                result.explanations = [explanation]
                result.error_msg = f"Timeout waiting for result from {component_explainer}"
                return result

            result_future = future.result()
            if result_future:
                result = result_future.get_result().result
                return result

            # Small delay to prevent busy waiting
            time.sleep(0.1)

        # This should not be reached, but return a default result just in case
        result = GenerateComponentExplanation.Result()
        explanation = Explanation()
        explanation.explanation = "Sorry, I can't explain this right now."
        result.explanations = [explanation]
        result.error_msg = f"Unexpected error getting result from {component_explainer}"
        return result

    #################################
    #
    # Lifecycle transitions callbacks
    #
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Configure the skill.

        You usually want to do the following in this state:
        - Read ROS parameters (if any)
        - Create ROS action clients and servers
        - Create ROS publishers and subscribers
        - Start publishing diagnostic information

        While the skill is configured, but not activated, it should not
        perform any actions that are not required for configuration, such as
        effectively processing data or calling external services.
        For instance, incoming goals on an action server should be rejected.

        :return: The state machine either invokes a transition to the
            "inactive" state or stays in "unconfigured" depending on the
            return value.
            TransitionCallbackReturn.SUCCESS transitions to "inactive".
            TransitionCallbackReturn.FAILURE transitions to "unconfigured".
            TransitionCallbackReturn.ERROR or any uncaught exceptions to
            "errorprocessing"
        """

        # example API for our skill
        # (note that we create the action server in the configure state,
        # so that the API is available to users, but we only start accepting
        # goals in the activate state)
        self.skill_server = ActionServer(self,
                                         GenerateExplanation,
                                         "/skill/explain",
                                         goal_callback=self.on_request_goal,
                                         execute_callback=self.on_request_exec)

        # Component Explainers
        self._component_explain_planner_client = ActionClient(
            self, GenerateComponentExplanation, '/component_explain_planner/explain')

        self._component_explain_navigation_client = ActionClient(
            self, GenerateComponentExplanation, '/component_explain_navigation/explain')

        self._component_explain_ask_human_for_help_client = ActionClient(
            self, GenerateComponentExplanation, '/component_explain_ask_human_for_help/explain')

        self._component_explain_say_client = ActionClient(
            self, GenerateComponentExplanation, '/component_explain_say/explain')

        self.component_explainers = {
            "component_explain_planner": self._component_explain_planner_client,
            "component_explain_navigation": self._component_explain_navigation_client,
            "component_explain_ask_human_for_help": self._component_explain_ask_human_for_help_client,
            "component_explain_say": self._component_explain_say_client,
        }

        # configure and start diagnostics publishing
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)
        self._diag_timer = self.create_timer(1., self.publish_diagnostics)

        self.get_logger().info("Skill skill_explain is configured, but not yet active")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """
        Activate the skill.

        You usually want to do the following in this state:
        - Create and start any timers performing periodic tasks
        - Start processing data, and accepting action goals, if any

        """
        # Subscribers
        self._task_info_sub_cbGroup = MutuallyExclusiveCallbackGroup()
        self._task_info_subscriber = self.create_subscription(
            String,
            '/task_info',
            self.on_new_task_info,
            10, callback_group=self._task_info_sub_cbGroup)

        self.get_logger().info("Skill skill_explain is active and running...")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop the timer to stop calling the `run` function (main task of your application)."""
        self.get_logger().info("Stopping skill...")
        # self.destroy_timer(self._timer)

        self.get_logger().info("Skill skill_explain is stopped (inactive)")
        return super().on_deactivate(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Shutdown the node, after a shutting-down transition is requested.

        :return: The state machine either invokes a transition to the
            "finalized" state or stays in the current state depending on the
            return value.
            TransitionCallbackReturn.SUCCESS transitions to "finalized".
            TransitionCallbackReturn.FAILURE remains in current state.
            TransitionCallbackReturn.ERROR or any uncaught exceptions to
            "errorprocessing"
        """

        self.destroy_subscription(self._task_info_subscriber)

        self.get_logger().info('Shutting down skill_explain skill.')

        self.skill_server.destroy()

        self.destroy_timer(self._diag_timer)
        self.destroy_publisher(self._diag_pub)

        self.get_logger().info("Skill skill_explain finalized.")
        return TransitionCallbackReturn.SUCCESS

    #################################

    def publish_diagnostics(self):

        arr = DiagnosticArray()
        msg = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="/skill/skill_explain",
            message="skill skill_explain is running",
            values=[
                KeyValue(key="Module name", value="skill_explain"),
                KeyValue(key="Current lifecycle state",
                         value=self._state_machine.current_state[1]),
            ],
        )

        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [msg]
        self._diag_pub.publish(arr)
