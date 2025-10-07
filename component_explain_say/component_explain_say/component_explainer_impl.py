import time

from rclpy.action import ActionServer, GoalResponse
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from explainability_msgs.action import GenerateComponentExplanation
from explainability_msgs.msg import Explanation

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.srv import GetParameters


class explainerImpl(Node):
    """
    Implementation of component_explain_say.
    """

    def __init__(self) -> None:
        """Construct the node."""
        super().__init__('explainer_component_explain_say')

        self.get_logger().info("Initialising...")
        self._timer = None
        self._diag_pub = None
        self._diag_timer = None

        self.declare_parameter('default_tts_timeout', 20.0)
        self.default_tts_timeout = 20.0

        self.explainer_server = None  # action server to start/stop this explainer

        self.explainer_running = False
        self.completed = 0

        self.get_logger().info('explainer component_explain_say started, but not yet configured.')

    def on_request_goal(self, goal_handle):
        """Accept incoming goal if appropriate."""
        if self._state_machine.current_state[1] != "active":
            self.get_logger().error("explainer is not active, rejecting goal")
            return GoalResponse.REJECT

        self.get_logger().info("Accepted a new goal")
        return GoalResponse.ACCEPT

    def on_request_exec(self, goal_handle):
        """Process incoming goal."""

        feedback_msg = GenerateComponentExplanation.Feedback()
        feedback_msg.status = "explainer started"

        goal_handle.publish_feedback(feedback_msg)

        feedback_msg.status = "explainer completed"
        goal_handle.publish_feedback(feedback_msg)

        max_time_to_talk = self.get_parameter('default_tts_timeout').get_parameter_value().double_value  # Default value if the service is not available

        # # Uncomment  to get the ros2 parameter for the maximum time to talk from communication_hub node
        # client = self.create_client(GetParameters, '/communication_hub/get_parameters'

        # if client.wait_for_service(timeout_sec=5.0):
        #     request = GetParameters.Request()
        #     request.names = ['multi_modal_expression_timeout']

        #     future = client.call_async(request)

        #     start_time = time.time()
        #     while not future.done() and time.time() - start_time < 5.0:
        #         time.sleep(0.1)  # wait for the service to respond

        #     if future.result() and future.result().values:
        #         max_time_to_talk = future.result().values[0].double_value

        # max_time_to_talk = int(max_time_to_talk) if max_time_to_talk is not None else None

        print(f"Max time to talk: {max_time_to_talk}")

        if max_time_to_talk is None:
            explanation = "If what I need say is too long, I will stop talking after some time."
        else:
            explanation = (f"If what I say is longer than {max_time_to_talk} seconds, "
                            "I will stop talking after that time.")

        generated_explanation = Explanation(
            component_name="component_explain_say",
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
            self, GenerateComponentExplanation, "/component_explain_say/explain",
            goal_callback=self.on_request_goal,
            execute_callback=self.on_request_exec)

        self.get_logger().info("explainer component_explain_say is configured, but not yet active")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """
        Activate the skill.

        You usually want to do the following in this state:
        - Create and start any timers performing periodic routines
        - Start processing data, and accepting action goals, if any

        """

        self.get_logger().info("component_explain_say is active and running")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop the timer to stop calling the `run` function."""
        self.get_logger().info("Stopping explainer...")
        self.destroy_timer(self._timer)

        self.get_logger().info("component_explain_say is stopped (inactive)")
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
        self.get_logger().info('Shutting down component_explain_say.')
        self.destroy_timer(self._diag_timer)
        self.destroy_publisher(self._diag_pub)

        self.destroy_timer(self._timer)

        self.get_logger().info("component_explain_say finalized.")
        return TransitionCallbackReturn.SUCCESS

    #################################

    def publish_diagnostics(self):

        arr = DiagnosticArray()
        msg = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="/component_explain_say",
            message="component_explain_say is running",
            values=[
                KeyValue(key="Module name", value="component_explain_say"),
                KeyValue(key="Current lifecycle state",
                         value=self._state_machine.current_state[1]),
                KeyValue(key="Current completion percentage",
                         value=f"{self.completed}"),
            ],
        )

        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [msg]
        self._diag_pub.publish(arr)

