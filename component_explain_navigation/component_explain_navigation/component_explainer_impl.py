import json

# from openai import OpenAI

from rclpy.action import ActionServer, GoalResponse
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from explainability_msgs.action import GenerateComponentExplanation
from explainability_msgs.msg import Explanation

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from rcl_interfaces.msg import Log

import requests

from geometry_msgs.msg import PoseWithCovarianceStamped

from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

DEFAULT_LLM_MODEL = 'gpt-4.1-mini'
DEFAULT_LLM_HOST = 'https://api.openai.com'
DEFAULT_API_KEY = '<insert_your_api_key_here>'


class explainerImpl(Node):
    """
    Implementation of component_explain_navigation.
    """

    def __init__(self) -> None:
        """Construct the node."""
        super().__init__('explainer_component_explain_navigation')

        # Declare LLM parameters
        self.declare_parameter('llm_model', DEFAULT_LLM_MODEL)
        self.declare_parameter('llm_host', DEFAULT_LLM_HOST)
        self.declare_parameter('api_key', DEFAULT_API_KEY)

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

        self.api_key = (self.get_parameter('api_key').get_parameter_value().string_value
                   or DEFAULT_API_KEY)

        self.get_logger().info(f"Using LLM model: {self.llm_model}, host: {self.llm_host}")

        # self.llm_client = OpenAI(
        #     base_url=self.llm_host + '/v1',
        #     api_key=self.api_key,
        # )

        self.logs = []
        self.last_log_msg = {}

        self.high_localization_variance_count = 0

        self.loggers_filter = ["amcl", "global_costmap.global_costmap",
                               "local_costmap.local_costmap", "behaviour_server",
                               "planner_server", "controller_server", 
                            #    "bt_navigator",
                               "skill_navigate_to_zone", "skill_navigate_to_pose"]
        
        self.skip_if_found = [
            "KeepoutFilter",
            "SpeedFilter",
            "missed its desired rate",
            "Client requested to cancel the goal. Cancelling",
            "Goal was canceled. Stopping the robot",
            "The /scan observation buffer has not been updated",
            "Message Filter dropping message",
            "Optimizer reset"
        ]

        self.is_charging = None
        self.is_joystick_manual = None

        self.get_logger().info('component_explain_navigation started, but not yet configured.')

    def on_request_goal(self, goal_handle):
        """Accept incoming goal if appropriate."""
        if self._state_machine.current_state[1] != "active":
            self.get_logger().error("explainer is not active, rejecting goal")
            return GoalResponse.REJECT

        self.get_logger().info("Accepted a new goal")
        return GoalResponse.ACCEPT

    def on_request_exec(self, goal_handle):
        """Process incoming goal."""
        print("Executing goal...")
        input_data = goal_handle.request.json_data
        input_data = json.loads(input_data)

        question = input_data.get("question", "")
        time_range_start = input_data.get("time_range_start", "")
        time_range_end = input_data.get("time_range_end", "")

        self.get_logger().info(f"Starting the explainer with question <{question}>")

        feedback_msg = GenerateComponentExplanation.Feedback()
        feedback_msg.status = "explainer started"
        goal_handle.publish_feedback(feedback_msg)

        explanation = self.generate_explanation(
            question=question,
            time_range_start=time_range_start,
            time_range_end=time_range_end)

        feedback_msg.status = "explainer completed"
        goal_handle.publish_feedback(feedback_msg)

        generated_explanation = Explanation(
            component_name="component_explain_navigation",
            explanation=explanation)

        self.get_logger().info(f"Generated explanation: {generated_explanation.explanation}")

        explanations = [generated_explanation]

        goal_handle.succeed()
        return GenerateComponentExplanation.Result(explanations=explanations)

    def generate_explanation(self, question, time_range_start, time_range_end):
        """
        Generate an explanation based on the question and time range using ollama.
        """
        relevant_logs = self.get_relevant_logs(
            time_range_start=time_range_start,
            time_range_end=time_range_end)

        if self.is_charging:
            self.get_logger().info("Using log: The robot is plugged in, so it won't be able to navigate.")
            relevant_logs.insert(0, "The robot is charging, so it won't be able to navigate.")
        if self.is_joystick_manual:
            self.get_logger().info("Using log: The joystick is in manual mode, so it won't be able to navigate.")
            relevant_logs.insert(
                0, "The joystick is in manual mode, so it won't be able to navigate.")

        print(f"Relevant logs: {relevant_logs}")

        if len(relevant_logs) == 0:
            relevant_logs = ["No relevant logs found in the specified time range."]

        relevant_logs = "\n".join(relevant_logs)

        prompt_system = """
        You are a helpful robot explainer. Your task is to provide clear and concise explanations for the user's questions about your navigation system.

        You're given a list of relevant ROS2 logs from your the navigation system, and a user question.

        Your task is to generate a concise and informative explanation that answers the root caose of the user's question. 

        The user is not an expert in robotics, so provide a simple explanation that only highlights the main reason that answers the question.

        The explanation must be very short, of maximum one sentence. It should be concise and to the point. Do not use bullets or lists.

        Answer the question as if you were the robot itself, and you were asked to explain your behaviour to the user.

        Explain only the main reason that answers the question, and do not provide any additional information. If part of the reason is the joystick being in manual mode, or the robot being charging, mention it explicitly and do not provide any additional information.

        If you have no information to answer the question, respond with "I do not have enough information to answer this question.".

        Examples:
        - User question: "Why did the robot fail to navigate?"
        - Relevant logs include information about not finding a valid path to the goal, but it is not mentioned that the manual mode is selected in the joystick or that the robot is charging.
            - Response: "Too many obstacles prevented me to find a valid path."

        - User question: "Why did the robot fail to navigate?"
        - Relevant logs indicate that the joystick is in manual mode at least once.
            - Response: "The manual mode is selected in the joystick."

        - User question: "Why did the robot fail to navigate?"
        - Relevant logs indicate that the robot is charging.
            - Response: "The robot is plugged in, so autonomous navigation is disabled."

        - User question: "Why did the robot fail to navigate?"
        - Relevant logs indicate no issues in finding the path or following it, but the goal was not reached.
            - Response: "I reached the maximum time that I have to navigate, and I did not reach the goal."

        - User question: "Why did it take so long to navigate?"
        - Relevant logs include information about a high uncertainty in the robot's position or starting position in lethal space.
            - Response: "I had issues determining my position."

        - User question: "Why did it take so long to navigate?"
        - Relevant logs indicate that the the robot failed to create a plan or had to clear the local or global costmants. In the end the skill completed successfully.
            - Response: "I had to find paths due to moving obstacles in the environment."

        - User question: "Why did it take so long to navigate?"
        - Relevant logs do not mention any failure to create a plan or clearing of the local or global costmaps. The skill completed successfully.
            - Response: "I need some time to compute the path, and my maximum speed is limited to ensure safety."

        - User question: "Why did the robot fail to navigate to the garden?"
        - Relevant logs are empty.
            - Response: "I do not have enough information to answer this question."
        """
        prompt_user = f"""
        Relevant logs: {relevant_logs}
        User question: {question}
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
                        'content': prompt_system,
                    },
                    {
                        'role': 'user',
                        'content': prompt_user,
                    }],
                'temperature': 0.0,
                'stream': False},
            headers=headers)
        if response.status_code != requests.codes.ok:
            raise RuntimeError(
                f'Ollama server response [{response.status_code}]: {response.text}')
        response_json = response.json()['choices'][0]

        explanation = str(response_json['message']['content']).strip()
        self.get_logger().info(f"Generated explanation: {explanation}")
        return explanation


        # # prompt and parse the response
        # response = self.llm_client.chat.completions.create(
        #     model=self.llm_model,
        #     temperature=0.0,
        #     messages=[
        #         {
        #             'role': 'system',
        #             'content': prompt_system,
        #         },
        #         {
        #             'role': 'user',
        #             'content': prompt_user,
        #         },
        #     ])
        # explanation = response.choices[0].message.content

        # self.get_logger().info(f"Generated explanation: {explanation}")
        # return explanation

    def get_relevant_logs(self, time_range_start, time_range_end):
        """
        Retrieve relevant logs from the ROS /rosout topic within the specified time range.
        """
        relevant_logs = []
        for log in self.logs:
            log_time = float(log['timestamp'])
            if float(time_range_start) <= log_time <= float(time_range_end):
                relevant_logs.append(f"[{log['name']}] {log['msg']}")
        return relevant_logs

    def rosout_callback(self, msg: Log):
        if msg.name.lower() in [logger.lower() for logger in self.loggers_filter]:

            if msg.name in self.last_log_msg:
                if self.last_log_msg[msg.name] == msg.msg:
                    # Avoid storing duplicate log messages
                    return

            for skip_phrase in self.skip_if_found:
                if skip_phrase in msg.msg:
                    # Avoid storing logs that match any of the skip phrases
                    return

            self.logs.append({
                'timestamp': f"{self.get_clock().now().to_msg().sec}.{self.get_clock().now().to_msg().nanosec}",
                'name': msg.name,
                'msg': msg.msg,
                'level': msg.level,
            })
            self.get_logger().info(f"Received log: [{msg.name}] {msg.msg}")

            self.last_log_msg[msg.name] = msg.msg

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        Callback for the /amcl_pose topic.
        This is used to check the robot's localization status.
        """
        # Compute mean covariance of the robot's pose
        covariance = msg.pose.covariance

        # XY mean variance (only diagonal elements)
        xy_variance = (covariance[0] + covariance[7]) / 2

        # Orientation variance
        orientation_variance = covariance[35]

        if xy_variance > 0.2 or orientation_variance > 0.2:
            self.high_localization_variance_count += 1
            if self.high_localization_variance_count > 5:
                self.get_logger().warn(
                    f"Robot pose covariance is high, indicating poor localization. XY variance: {xy_variance}, Orientation variance: {orientation_variance}")
                self.logs.append({
                    'timestamp': f"{self.get_clock().now().to_msg().sec}.{self.get_clock().now().to_msg().nanosec}",
                    'name': 'amcl_pose',
                    'msg': f"High uncertainty in the robot's position.",
                    'level': Log.WARN,
                })
        else:
            self.high_localization_variance_count -= 1

    def joy_priority_callback(self, msg: Bool):
        """
        Callback for the /joy_priority topic.
        This is used to check if the joystick is in manual mode.
        """
        if msg.data and msg.data != self.is_joystick_manual:
            self.is_joystick_manual = True

        elif not msg.data and self.is_joystick_manual != msg.data:
            self.is_joystick_manual = False

    def plugged_callback(self, msg: Bool):
        """
        Callback for the /power/is_charging topic.
        This is used to check if the robot is plugged in.
        """
        if msg.data and self.is_charging != msg.data:
            self.is_charging = msg.data

        elif not msg.data and self.is_charging != msg.data:
            self.is_charging = msg.data

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
            self, GenerateComponentExplanation, "/component_explain_navigation/explain",
            goal_callback=self.on_request_goal,
            execute_callback=self.on_request_exec)

        # Create a subscriber to /rosout
        self.rosout_subscriber = self.create_subscription(
            Log,
            '/rosout',
            self.rosout_callback,
            10
        )

        # Create a subscriber to amcl_pose
        self.amcl_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        # Create a subscriber to /joy_priority
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.joy_priority_subscriber = self.create_subscription(
            Bool,
            '/joy_priority',
            self.joy_priority_callback,
            latched_qos
        )

        self.charging_subscriber = self.create_subscription(
            Bool,
            '/power/is_charging',
            self.plugged_callback,
            10
        )

        self.get_logger().info("explainer component_explain_navigation is configured, but not yet active")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """
        Activate the skill.

        You usually want to do the following in this state:
        - Create and start any timers performing periodic routines
        - Start processing data, and accepting action goals, if any

        """

        self.get_logger().info("component_explain_navigation is active and running")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop the timer to stop calling the `run` function."""
        self.get_logger().info("Stopping explainer...")
        self.destroy_timer(self._timer)

        self.get_logger().info("component_explain_navigation is stopped (inactive)")
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
        self.get_logger().info('Shutting down component_explain_navigation.')
        self.destroy_timer(self._diag_timer)
        self.destroy_publisher(self._diag_pub)

        self.destroy_timer(self._timer)

        self.get_logger().info("component_explain_navigation finalized.")
        return TransitionCallbackReturn.SUCCESS

    #################################

    def publish_diagnostics(self):

        arr = DiagnosticArray()
        msg = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="/component_explain_navigation",
            message="component_explain_navigation is running",
            values=[
                KeyValue(key="Module name", value="component_explain_navigation"),
                KeyValue(key="Current lifecycle state",
                         value=self._state_machine.current_state[1])
            ],
        )

        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [msg]
        self._diag_pub.publish(arr)
