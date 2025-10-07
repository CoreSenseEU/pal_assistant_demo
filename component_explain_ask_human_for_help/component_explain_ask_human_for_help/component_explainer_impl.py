import json
import requests

from rclpy.action import ActionServer, GoalResponse
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


from explainability_msgs.action import GenerateComponentExplanation
from explainability_msgs.msg import Explanation

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from component_explain_ask_human_for_help.causal_model import CausalModel
from component_explain_ask_human_for_help.explain import Explainer, CounterfactualExplanation
from component_explain_ask_human_for_help.dummy_skill import AskHumanForHelpState, NAMES_TO_NODES

DEFAULT_NAV_ERR_THRESHOLD = 0.5
DEFAULT_STD_RMS_THRESHOLD = 0.5
DEFAULT_MAX_DISTANCE = 4.0
DEFAULT_LLM_MODEL = 'gpt-4.1-mini'
DEFAULT_LLM_HOST = 'https://api.openai.com'
DEFAULT_API_KEY = '<insert_your_api_key_here>'

SYSTEM_DESCRIPTION_PROMPT_ELEMENT = """
    Here are the meanings of the variables used in the formal explanations:
    "Executed_Initial":"The robot started executing the 'Ask Human For Help' skill.",
    "Executed_Detection":"The robot started looking for a person to ask for help.",
    "Executed_Navigate":"The robot started approaching the person to ask for help.",
    "Executed_AskForHelp":"The robot started asking the person for help.",
    "Executed_WaitForConfirmation":"The robot started asking the person for confirmation that they have helped the robot.",
    "Executed_Success":"The robot successfully completed the 'Ask Human For Help' skill.",
    "Executed_Failure":"The robot failed to complete the 'Ask Human For Help' skill.",
    "Transition_Initial":"Transition from the initial state, typically to the detection state.",
    "Transition_Detection":"Transition from the detection state, typically to the navigation or ask for help state.",
    "Transition_Navigate":"Transition from the navigation state, typically to the ask for help state.",
    "Transition_AskForHelp":"Transition from the ask for help state, typically to the wait for confirmation state.",
    "Transition_WaitForConfirmation":"Transition from the wait for confirmation state, typically to the success state.",
    "Transition_Failure":"Not used",
    "Transition_Success":"Not used",
    "user_accepted_0":"True if the user accepted the robot's request for help, False otherwise.",
    "user_confirmed_0":"True if the user confirmed that they have helped the robot, False otherwise.",
    "person_approached_0":"True if the robot approached a person to ask for help, False otherwise.",
    "person_detected_0":"True if the robot detected a person to ask for help, False otherwise.",
    "detection_stable_0":"True if the detection of a person is stable, False otherwise.",
    "person_distance_0":"The distance to the detected person.",
    "navigation_failure_0":"True if the robot failed to navigate to the person, False otherwise.",
"""


class explainerImpl(Node):
    """
    Implementation of component_explain_ask_human_for_help.
    """

    def __init__(self) -> None:
        """Construct the node."""
        super().__init__('explainer_component_explain_ask_human_for_help')

        # Declare LLM parameters
        self.declare_parameter('llm_model', DEFAULT_LLM_MODEL)
        self.declare_parameter('llm_host', DEFAULT_LLM_HOST)
        self.declare_parameter('api_key', DEFAULT_API_KEY)
        self.declare_parameter('std_rms_threshold', DEFAULT_STD_RMS_THRESHOLD)
        self.declare_parameter('nav_err_threshold', DEFAULT_NAV_ERR_THRESHOLD)
        self.declare_parameter('max_distance', DEFAULT_MAX_DISTANCE)

        self.get_logger().info("Initialising...")
        self._timer = None
        self._diag_pub = None
        self._diag_timer = None

        self.explainer_server = None  # action server to start/stop this explainer

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

        self.reset_explanation_model()

        self.explainer_running = False
        self.completed = 0

        self.std_rms_threshold = DEFAULT_STD_RMS_THRESHOLD
        self.nav_err_threshold = DEFAULT_NAV_ERR_THRESHOLD
        self.max_distance = DEFAULT_MAX_DISTANCE

        self.get_logger().info('explainer component_explain_ask_human_for_help started, but not yet configured.')

    def reset_explanation_model(self):
        self.causal_model = CausalModel()
        self.current_fsm_state = None

    def on_request_goal(self, goal_handle):
        """Accept incoming goal if appropriate."""
        if self._state_machine.current_state[1] != "active":
            self.get_logger().error("explainer is not active, rejecting goal")
            return GoalResponse.REJECT

        self.get_logger().info("Accepted a new goal")
        return GoalResponse.ACCEPT

    def on_request_exec(self, goal_handle):
        """Process incoming goal."""
        input_data = goal_handle.request.json_data
        input_data = json.loads(input_data)

        question = input_data.get("question", "")
        time_range_start = input_data.get("time_range_start", "")
        time_range_end = input_data.get("time_range_end", "")

        self.std_rms_threshold = self.get_parameter('std_rms_threshold').get_parameter_value().double_value
        self.nav_err_threshold = self.get_parameter('nav_err_threshold').get_parameter_value().double_value
        self.max_distance = self.get_parameter('max_distance').get_parameter_value().double_value

        # Boilerplate stuff
        self.causal_model.set_fsm_parameters({
            "max_distance": self.max_distance,
        })
        self.get_logger().info(f"Starting the explainer with question <{question}>")
        feedback_msg = GenerateComponentExplanation.Feedback()
        feedback_msg.status = "explainer started"
        goal_handle.publish_feedback(feedback_msg)
        self.explainer_running = True
        self.get_logger().info(f"Ask Human For Help State: {self.causal_model.fsm_state.values}")

        # Case where no execution occurred
        if self.current_fsm_state is None:
            explanation_text = "I didn't execute the Ask Human For Help skill."
        else:
            # Explanation
            if self.causal_model.fsm_state.get_value("Executed_Failure"):
                # We reached the failure state, for now, we assume that if this is the case, we must explain the failure
                self.get_logger().info("Failure detected, explaining failure...")
                explainer = Explainer(model=self.causal_model)
                failed_node = self.get_failed_node()
                if failed_node is not None:
                    query = explainer.construct_query(
                        foils={f"{failed_node}": None}, ignore_none=True)
                    formal_explanations = explainer.explain(query=query)
                    self.get_logger().info(f"Found {len(formal_explanations)} explanations:")
                    for exp, i in zip(formal_explanations, range(len(formal_explanations))):
                        self.get_logger().info(f"Explanation {i}: {exp.text()}")
                else:
                    # TODO: Process the question to generate a more general explanation
                    self.get_logger().error("No failed node found in the causal model, cannot explain failure.")
                    raise ValueError(
                        "No failed node found in the causal model, cannot explain failure.")
                explanation_text = self.formal_explanations_to_text(formal_explanations)

            else:
                # No failure detected, see if other flags are set
                if "person_std_rms_0" in self.causal_model.fsm_state.values:
                    rms_val = self.causal_model.fsm_state.get_value("person_std_rms_0")
                    self.get_logger().info(f"STD RMS: {rms_val}")
                if "robot_nav_err_0" in self.causal_model.fsm_state.values:
                    nav_err_val = self.causal_model.fsm_state.get_value("robot_nav_err_0")
                    self.get_logger().info(f"Nav Error: {nav_err_val}")

                if "person_std_rms_0" in self.causal_model.fsm_state.values and self.causal_model.fsm_state.get_value("person_std_rms_0") > self.std_rms_threshold:
                    # High variance in person detection
                    templated_explanation = "Perhaps my sensors were too noisy, or the person was moving too much. This could explain my behaviour."
                    explanation_text = templated_explanation
                elif "robot_nav_err_0" in self.causal_model.fsm_state.values and self.causal_model.fsm_state.get_value("robot_nav_err_0") > self.nav_err_threshold:
                    # High error between goal and actual position
                    templated_explanation = "I aimed for an appropriate position, but my final position is a bit different due to errors in my navigation."
                    explanation_text = templated_explanation
                else:
                    explanation_text = self.generic_explanation(
                        question, time_range_start, time_range_end)

        generated_explanation = Explanation(
            component_name="component_explain_ask_human_for_help",
            explanation=explanation_text)

        self.get_logger().info(f"Generated explanation: {generated_explanation.explanation}")

        explanations = [generated_explanation]

        goal_handle.succeed()
        return GenerateComponentExplanation.Result(explanations=explanations)

    def on_new_episodic_memory_update(self, msg: DiagnosticArray):

        diagnostic_msg = msg.status[0]

        if diagnostic_msg.name != "/skills/skill_ask_human_for_help":
            # Not relevant
            return

        self.get_logger().info(f"Received new episodic memory update - {diagnostic_msg.message}")

        NODE_STATE_UPDATES = {
            "InitialState": "Initial",
            "DetectState": "Detection",
            "NavigateState": "Navigate",
            "AskForHelpState": "AskForHelp",
            "WaitForConfirmationState": "WaitForConfirmation",
            "SkillFailed": "Failure",
            "SkillSucceeded": "Success",
        }

        var_updates = {}

        # Process update
        for key_val in diagnostic_msg.values:
            if key_val.key == "Module name":
                # Can ignore
                continue
            elif key_val.key in NODE_STATE_UPDATES:
                if key_val.key == "InitialState":
                    # New execution, reset the model
                    self.reset_explanation_model()
                    self.get_logger().info("Resetting the explanation model")
                else:
                    old_state = self.current_fsm_state
                    self.causal_model.fsm_state.set_value(
                        f"Transition_{self.current_fsm_state}", NODE_STATE_UPDATES[key_val.key])
                    self.get_logger().info(
                        f"Transitioning from {old_state} to {NODE_STATE_UPDATES[key_val.key]}")
                self.current_fsm_state = NODE_STATE_UPDATES[key_val.key]
                self.causal_model.fsm_state.set_value(f"Executed_{self.current_fsm_state}", True)

            elif key_val.key in AskHumanForHelpState.default_state:
                # Must be a variable in the state
                # TODO: Work out which version of the state variable to use, this is a very simple dirty way of doing it
                if key_val.key not in var_updates:
                    var_updates[key_val.key] = 0

                self.causal_model.fsm_state.set_value(
                    f"{key_val.key}_{var_updates[key_val.key]}", key_val.value)
                self.get_logger().info(
                    f"Setting {key_val.key}_{var_updates[key_val.key]} to {key_val.value} in the explanation model")
                var_updates[key_val.key] += 1
            else:
                self.get_logger().warn(f"Unknown key in episodic memory update: {key_val.key}")

    def get_failed_node(self):
        for node in self.causal_model.fsm_state.node_types:
            if self.causal_model.fsm_state.node_types[node] == "Transition" and self.causal_model.fsm_state.get_value(node) == "Failure":
                return node
        return None

    def formal_explanations_to_text(self, explanations):
        if len(explanations) == 0:
            # TODO: Something better
            return "No explanations found."
        elif len(explanations) == 1:
            return self.explain_single_formal_explanation(explanations[0])
        else:
            return self.explain_multiple_formal_explanations(explanations)

    def explain_single_formal_explanation(self, explanation: CounterfactualExplanation):
        single_formal_system_prompt = f"""
        You are an expert in explaining the reasoning behind a system's behavior.
        Your task is to take a formal explanation of a system's behavior and convert it into a human-readable explanation that is easy to understand.

        The formal explanation is provided in the following format:
        - fact: a fact or a set of facts that describe the system's behavior
        - reason: a fact or a set of facts that explain why the system behaved in a certain way
        - counterfactual: an alternate fact in a counterfactual scenario
        - foil: a fact that would have been true if the counterfactual had been true

        Given this formal explanation, your task is to generate a human-readable explanation that describes the system's behavior, the reason for that behavior, and what would have happened if the counterfactual had been true.
        The explanation should be concise, clear, and easy to understand for a human audience.
        The explanation should be only one or two sentences long, and should not contain any technical jargon or complex language.

        The explanation should be returned as a string, with no additional formatting or structure.

        {SYSTEM_DESCRIPTION_PROMPT_ELEMENT}

        Here are some examples:

        Example 1:
        - fact: "Transition_Detection = Failure"
        - reason: "person_detected_0 = False"
        - counterfactual: "person_detected_0 = True"
        - foil: "Transition_Detection = AskForHelp"

        Your response:
        "I didn't detect anyone, so I couldn't ask for help. If I had detected someone, I would have asked them for help."

        Example 2:
        - fact: "Transition_Navigate = Failure"
        - reason: "navigation_failure_0 = True"
        - counterfactual: "navigation_failure_0 = False"
        - foil: "Transition_Navigate = AskForHelp"
        Your response:
        "I couldn't approach the person because my navigation failed. If my navigation had not failed, I would have approached the person and asked them for help."

        Example 3:
        - fact: "Transition_AskForHelp = Failure"
        - reason: "user_accepted_0 = False"
        - counterfactual: "user_accepted_0 = True"
        - foil: "Transition_AskForHelp = WaitForConfirmation"
        Your response:
        "I asked the person for help, but they didn't help me. If they had helped me, I would have waited for their confirmation before proceeding."

        Example 4:
        - fact: "Transition_WaitForConfirmation = Failure"
        - reason: "user_confirmed_0 = False"
        - counterfactual: "user_confirmed_0 = True"
        - foil: "Transition_WaitForConfirmation = Success"
        Your response:
        "I waited for the person to help me, but they didn't confirm that they had helped me. If they had confirmed, I would have succeeded in my task."

         Example 5:
        - fact: "Transition_Detection = Failure"
        - reason: "detection_stable_0 = False"
        - counterfactual: "detection_stable_0 = True"
        - foil: "Transition_Detection = AskForHelp"
        Your response:
        "I detected a person, but the detection was too unstable to determine if it really was a person. If the detection had been stable, I would have asked the person for help."
        
        """

        single_formal_user_prompt = f"""
        fact: {explanation.fact_text()}, "reason: {explanation.reason_text()}, "counterfactual: {explanation.intervention_text()}, "foil: {explanation.foil_text()}
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
                        'content': single_formal_system_prompt,
                    },
                    {
                        'role': 'user',
                        'content': single_formal_user_prompt,
                    }],
                'temperature': 0.0,
                'stream': False},
            headers=headers)
        if response.status_code != requests.codes.ok:
            raise RuntimeError(
                f'Ollama server response [{response.status_code}]: {response.text}')
        response_json = response.json()['choices'][0]

        explanation = str(response_json['message']['content']).strip()
        return explanation

        # response = self.llm_client.chat.completions.create(
        #     model=self.llm_model,
        #     temperature=0.0,
        #     messages=[
        #         {
        #             'role': 'system',
        #             'content': single_formal_system_prompt,
        #         },
        #         {
        #             'role': 'user',
        #             'content': single_formal_user_prompt,
        #         },
        #     ])

        # # Make sure response is a string in json format
        # text_explanation = response.choices[0].message.content
        # return text_explanation

    def explain_multiple_formal_explanations(self, explanations: list[CounterfactualExplanation]):
        multiple_formal_system_prompt = f"""
        You are an expert in explaining the reasoning behind a system's behavior.
        Your task is to take multiple formal explanations of a system's behavior and convert them into a single human-readable explanation that is easy to understand.

        Each formal explanation is provided in the following format:
        "explanation_i - fact: ..., reason: ..., counterfactual: ..., foil: ..."

        where

        - fact: a fact or a set of facts that describe the system's behavior
        - reason: a fact or a set of facts that explain why the system behaved in a certain way
        - counterfactual: an alternate fact in a counterfactual scenario
        - foil: a fact that would have been true if the counterfactual had been true

        Given these formal explanations, your task is to generate a human-readable explanation that describes the system's behavior, the reason for that behavior, and what would have happened if the counterfactual had been true.
        The explanation should be concise, clear, and easy to understand for a human audience.
        The explanation should be only one or two sentences long, and should not contain any technical jargon or complex language.
        The explanations should be combined into a single coherent explanation that covers all the provided explanations, but should not be repetitive or redundant or too long.
        The explanation should be in the first person, as if the robot is explaining its own behavior.
        
        The explanation should be returned as a string, with no additional formatting or structure.

        {SYSTEM_DESCRIPTION_PROMPT_ELEMENT}

        Here are some examples:

        Example 1:
        "explanation_0 - fact: Transition_Detection = Failure, reason: person_distance_0 = 5, counterfactual: person_distance_0 = 0.0, foil: Transition_Detection = AskForHelp"
        "explanation_1 - fact: Transition_Detection = Failure, reason: person_distance_0 = 5, counterfactual: person_distance_0 = 0.5, foil: Transition_Detection = AskForHelp"
        "explanation_2 - fact: Transition_Detection = Failure, reason: person_distance_0 = 5, counterfactual: person_distance_0 = 1.0, foil: Transition_Detection = Navigate"
        "explanation_3 - fact: Transition_Detection = Failure, reason: person_distance_0 = 5, counterfactual: person_distance_0 = 1.5, foil: Transition_Detection = Navigate"
        "explanation_4 - fact: Transition_Detection = Failure, reason: person_distance_0 = 5, counterfactual: person_distance_0 = 2.0, foil: Transition_Detection = Navigate"
        "explanation_5 - fact: Transition_Detection = Failure, reason: person_distance_0 = 5, counterfactual: person_distance_0 = 2.5, foil: Transition_Detection = Navigate"
        "explanation_6 - fact: Transition_Detection = Failure, reason: person_distance_0 = 5, counterfactual: person_distance_0 = 3.0, foil: Transition_Detection = Navigate"
        "explanation_7 - fact: Transition_Detection = Failure, reason: person_distance_0 = 5, counterfactual: person_distance_0 = 3.5, foil: Transition_Detection = Navigate"
        "explanation_8 - fact: Transition_Detection = Failure, reason: person_distance_0 = 5, counterfactual: person_distance_0 = 4.0, foil: Transition_Detection = Navigate"

        Your response:
        "The closest person was 5 meters away, which is too far for me to ask for help. If the person had been closer, I would have asked them for help."
        
        """

        multiple_formal_user_prompt = ""
        for i, exp in enumerate(explanations):
            multiple_formal_user_prompt += f"explanation_{i} - fact: {exp.fact_text()}, reason: {exp.reason_text()}, counterfactual: {exp.intervention_text()}, foil: {exp.foil_text()}\n"
        
        headers = {}
        if self.api_key:
            headers['Authorization'] = f"Bearer {self.api_key}"
        response = requests.post(
            f'{self.llm_host}/v1/chat/completions',
            json={
                'model': self.llm_model,
                'messages': [
                    {'role': 'system',
                        'content': multiple_formal_system_prompt,
                    },
                    {
                        'role': 'user',
                        'content': multiple_formal_user_prompt,
                    }],
                'temperature': 0.0,
                'stream': False},
            headers=headers)
        if response.status_code != requests.codes.ok:
            raise RuntimeError(
                f'Ollama server response [{response.status_code}]: {response.text}')
        response_json = response.json()['choices'][0]
        explanation = str(response_json['message']['content']).strip()
        return explanation

        # response = self.llm_client.chat.completions.create(
        #     model=self.llm_model,
        #     temperature=0.0,
        #     messages=[
        #         {
        #             'role': 'system',
        #             'content': multiple_formal_system_prompt,
        #         },
        #         {
        #             'role': 'user',
        #             'content': multiple_formal_user_prompt,
        #         },
        #     ])

        # text_explanation = response.choices[0].message.content

        # return text_explanation

    def generic_explanation(self, question, time_range_start="", time_range_end=""):
        return "As far as I can tell, nothing went wrong when I asked the human for help."

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

        While the explainer is configured, but not activated, it should not
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
        # configure and start diagnostics publishing
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)
        self._diag_timer = self.create_timer(1., self.publish_diagnostics)

        # create here publishers, subscribers, clients, servers, etc.
        # required to implement the explainer (if any)
        self._episodic_memory_sub_cbGroup = MutuallyExclusiveCallbackGroup()
        self._episodic_memory_subscriber = self.create_subscription(
            DiagnosticArray,
            '/episodic_memory',
            self.on_new_episodic_memory_update,
            10, callback_group=self._episodic_memory_sub_cbGroup)

        # create the control server for ourselves
        self.explainer_server = ActionServer(
            self, GenerateComponentExplanation, "/component_explain_ask_human_for_help/explain",
            goal_callback=self.on_request_goal,
            execute_callback=self.on_request_exec)

        self.get_logger().info("explainer component_explain_ask_human_for_help is configured, but not yet active")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """
        Activate the skill.

        You usually want to do the following in this state:
        - Create and start any timers performing periodic routines
        - Start processing data, and accepting action goals, if any

        """

        self.get_logger().info("component_explain_ask_human_for_help is active and running")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop the timer to stop calling the `run` function."""
        self.get_logger().info("Stopping explainer...")
        self.destroy_timer(self._timer)

        self.get_logger().info("component_explain_ask_human_for_help is stopped (inactive)")
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
        self.get_logger().info('Shutting down component_explain_ask_human_for_help.')
        self.destroy_timer(self._diag_timer)
        self.destroy_publisher(self._diag_pub)

        self.destroy_timer(self._timer)

        self.get_logger().info("component_explain_ask_human_for_help finalized.")
        return TransitionCallbackReturn.SUCCESS

    #################################

    def publish_diagnostics(self):

        arr = DiagnosticArray()
        msg = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="/component_explain_ask_human_for_help",
            message="component_explain_ask_human_for_help is running",
            values=[
                KeyValue(key="Module name", value="component_explain_ask_human_for_help"),
                KeyValue(key="Current lifecycle state",
                         value=self._state_machine.current_state[1]),
                KeyValue(key="Current completion percentage",
                         value=f"{self.completed}"),
            ],
        )

        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [msg]
        self._diag_pub.publish(arr)
