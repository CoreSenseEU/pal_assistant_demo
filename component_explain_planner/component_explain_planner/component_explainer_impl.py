import json

from rclpy.action import ActionServer, GoalResponse
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from explainability_msgs.action import GenerateComponentExplanation
from explainability_msgs.msg import Explanation

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from component_explain_planner.LLM_utils import BaseLLMUtils
from component_explain_planner.ollama_wrapper import OllamaWrapper

# Default LLM configuration (will be overridden by ROS parameters if provided)
DEFAULT_LLM_MODEL = 'gpt-4.1-mini'
DEFAULT_LLM_HOST = 'https://api.openai.com'
DEFAULT_API_KEY = '<insert_your_api_key_here>'

class explainerImpl(Node):
    """
    Implementation of component_explain_planner.
    """

    def __init__(self) -> None:
        """Construct the node."""
        super().__init__('explainer_component_explain_planner')

        # Declare LLM parameters
        self.declare_parameter('llm_model', DEFAULT_LLM_MODEL)
        self.declare_parameter('llm_host', DEFAULT_LLM_HOST)
        self.declare_parameter('api_key', DEFAULT_API_KEY)

        self.get_logger().info("Initialising...")
        self._timer = None
        self._diag_pub = None
        self._diag_timer = None

        self.explainer_server = None  # action server to start/stop this explainer

        self.explainer_running = False
        self.LLM_wrapper = None
        self.LLM_utils = None

        self.get_logger().info('explainer component_explain_planner started, but not yet configured.')

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

        # check if it's a valid JSON
        try:
            question = input_data.get("question", "")
            task_info = input_data.get("task_info", "")
        except json.JSONDecodeError as e:
            self.get_logger().error("Failed to decode JSON")
            # publish feedback and return failure
            feedback_msg = GenerateComponentExplanation.Feedback()
            feedback_msg.status = "failed to decode JSON"
            goal_handle.publish_feedback(feedback_msg)
            goal_handle.abort()
            return GenerateComponentExplanation.Result(explanations=[])

        self.get_logger().info(f"Starting the explainer with question <{question}>")

        feedback_msg = GenerateComponentExplanation.Feedback()
        feedback_msg.status = "explainer started"

        goal_handle.publish_feedback(feedback_msg)

        self.explainer_running = True
        instruction = task_info['instruction']
        task_status = task_info['task_status']
        task_error_msg = task_info['task_error_msg']
        if task_error_msg.strip() == "":
            task_error_msg = "None"
        skill_seq = task_info['skill_sequence']
        # Future idea: If a skill has failed, maybe remove subsequent skills from the sequence to avoid LLM hallucination.

        # dummy explainer that returns a templated explanation for empty plan, otherwise forwards to LLM
        if len(skill_seq) == 0:
            explanation = "I could not find a valid plan for the instruction: " + instruction
        else:
            feedback_msg = GenerateComponentExplanation.Feedback()
            feedback_msg.status = "Generating explanation using LLM"
            goal_handle.publish_feedback(feedback_msg)
            prompt = self.LLM_utils.prepare_prompt(
                replacements={
                    '[INSTRUCTION]': instruction,
                    '[TASK_STATUS]': task_status,
                    '[TASK_ERROR_MESSAGE]': task_error_msg,
                    '[SKILL_SEQUENCE]': str(skill_seq),
                    '[QUESTION]': question}, agent_name='basic-plan-explainer'
            )
            explanation = self.LLM_utils.get_response(prompt, ans_key='Answer:')

        feedback_msg.status = "explainer completed"
        goal_handle.publish_feedback(feedback_msg)

        self.explainer_running = False

        generated_explanation = Explanation(
            component_name="component_explain_planner",
            explanation=explanation)

        self.get_logger().info(f"Generated explanation: {generated_explanation.explanation}")

        explanations = [generated_explanation]

        goal_handle.succeed()
        return GenerateComponentExplanation.Result(explanations=explanations)

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

        # create the control server for ourselves
        self.explainer_server = ActionServer(
            self, GenerateComponentExplanation, "/component_explain_planner/explain",
            goal_callback=self.on_request_goal,
            execute_callback=self.on_request_exec)

        # Get LLM configuration from ROS parameters
        self.llm_model = (self.get_parameter('llm_model').get_parameter_value().string_value
                          or DEFAULT_LLM_MODEL)
        llm_host = (self.get_parameter('llm_host').get_parameter_value().string_value
                    or DEFAULT_LLM_HOST)

        api_key = (self.get_parameter('api_key').get_parameter_value().string_value or DEFAULT_API_KEY)

        self.get_logger().info(f"Using LLM model: {self.llm_model}, host: {llm_host}")

        self.LLM_wrapper = OllamaWrapper(model_name=self.llm_model, model_host=llm_host, api_key=api_key)
        self.LLM_utils = BaseLLMUtils(self.LLM_wrapper)

        self.get_logger().info("explainer component_explain_planner is configured, but not yet active")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """
        Activate the skill.

        You usually want to do the following in this state:
        - Create and start any timers performing periodic routines
        - Start processing data, and accepting action goals, if any

        """

        self.get_logger().info("component_explain_planner is active and running")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop the timer to stop calling the `run` function."""
        self.get_logger().info("Stopping explainer...")
        self.destroy_timer(self._timer)

        self.get_logger().info("component_explain_planner is stopped (inactive)")
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
        self.get_logger().info('Shutting down component_explain_planner.')
        self.destroy_timer(self._diag_timer)
        self.destroy_publisher(self._diag_pub)

        self.destroy_timer(self._timer)

        self.get_logger().info("component_explain_planner finalized.")
        return TransitionCallbackReturn.SUCCESS

    #################################

    def publish_diagnostics(self):

        arr = DiagnosticArray()
        msg = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="/component_explain_planner",
            message="component_explain_planner is running",
            values=[
                KeyValue(key="Module name", value="component_explain_planner"),
                KeyValue(key="Current lifecycle state",
                         value=self._state_machine.current_state[1]),
            ],
        )

        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [msg]
        self._diag_pub.publish(arr)
