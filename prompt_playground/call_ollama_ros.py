from ollama import chat, Client
from pydantic import BaseModel, Field
from typing import Literal, List, Annotated, Union, Dict, Optional
from typing_extensions import TypedDict
import time
import json
from hri_actions_msgs.msg import Intent as IntentMsg
from hri_msgs.msg import LiveSpeech
from tts_msgs.action import TTS
from chatbot_msgs.srv import GetSystemPrompt


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String


class GenericSkill(BaseModel):
    skill: str
    params: Optional[Dict]
    result: Optional[Dict]


class StartActivity(BaseModel):
    type: Literal[IntentMsg.START_ACTIVITY]
    object: str
    # plan: List[Union[SaySkill, AskSkill, NavigateToSkill, LookForHumanSkill, AskHumanForHelpSkill]]
    plan: List[GenericSkill]


class PerformMotion(BaseModel):
    type: Literal[IntentMsg.PERFORM_MOTION]
    object: str


class ChatbotResponse(BaseModel):
    say: Optional[str]
    user_intent: Optional[Union[PerformMotion, StartActivity]]
    result: Optional[str]


class StringNode(Node):
    def __init__(self):
        super().__init__("ollama_node")
        # self.subscription = self.create_subscription(
        #     String, "/llm_context_manager/system_prompt_updates", self.listener_callback, 10
        # )

        self.subscription_speech = self.create_subscription(
            LiveSpeech, "/humans/voices/anonymous_speaker/speech", self.listener_speech_callback, 10
        )

        self.tts_action_client = ActionClient(self, TTS, "/tts_engine/tts")

        self.publisher = self.create_publisher(IntentMsg, "/intents", 10)

        self._system_prompt_client = self.create_client(
            GetSystemPrompt, "/llm_context_manager/get_system_prompt")
        while not self._system_prompt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        print("Service available")

        self.client = Client(host='http://192.168.88.110:11434')

        self.chat_history = []
        self.temperature = 0
        # model = 'deepseek-r1:1.5b'
        self.model = "phi4"

        self.get_logger().info("Node started!")

    # def listener_callback(self, msg):
    #     print("SYSTEM: " + msg.data)
    #     self.chat_history.append({"role": "system", "content": msg.data})

    def listener_speech_callback(self, msg):
        self.user_sentence = msg.final
        future = self._system_prompt_client.call_async(
            GetSystemPrompt.Request(prompt_keyword="chat_and_plan"))
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        response = future.result()
        system_update_prompt = response.prompt

        if response.error == "":
            print(f"SYSTEM: {system_update_prompt} \n")
            self.chat_history.append({"role": "system", "content": system_update_prompt})

        print("USER: " + self.user_sentence + "\n")

        print(f"\n\n...waiting for chatbot answer \n")

        self.chat_history.append({"role": "user", "content": self.user_sentence})
        init_time = time.time()
        response = self.client.chat(
            messages=self.chat_history,
            model=self.model,
            options={'temperature': self.temperature},
            format=ChatbotResponse.model_json_schema(),
            keep_alive=-1
        )
        print(f"Time taken to generate: {time.time() - init_time} \n")
        print("ASSISTANT: " + response.message.content + "\n")

        # res = ChatbotResponse.model_validate_json(response.message.content)
        self.res = json.loads(response.message.content)
        self.chat_history.append({"role": "assistant", "content": response.message.content})

        # TO REMOVE: fixing plan
        # res = {
        #     "say": "Sure, I'll bring you a coffee. Just to confirm, should I prepare it in the kitchen?",
        #     "user_intent": {
        #         "type": "__intent_start_activity__",
        #         "object": "prepare_coffee",
        #         "plan": [
        #             {
        #                 "skill": "navigate_to",
        #                 "params": {
        #                     "goal": "kitchen"
        #                 },
        #                 "result": None
        #             },
        #             {
        #                 "skill": "look_for_human",
        #                 "params": {
        #                     "patterns": "[]"
        #                 },
        #                 "result": None
        #             },
        #             {
        #                 "skill": "ask_human_for_help",
        #                 "params": {
        #                     "text": "Can you prepare a coffee?",
        #                     "recipient": "?look_for_human.found_entities"
        #                 },
        #                 "result": None
        #             },
        #             {
        #                 "skill": "navigate_to",
        #                 "params": {
        #                     "goal": "specific_room"
        #                 },
        #                 "result": None
        #             },
        #             {
        #                 "skill": "say",
        #                 "params": {
        #                     "text": "Here is your coffee"
        #                 },
        #                 "result": None
        #             }
        #         ]
        #     },
        #     "result": None
        # }

        if self.res["say"]:
            # Call tts
            goal = TTS.Goal()
            goal.input = self.res["say"]
            future = self.tts_action_client.send_goal_async(goal)
            time.sleep(len(self.res["say"])*0.12)

        user_intent = self.res["user_intent"]
        if user_intent:
            if user_intent["type"] == IntentMsg.PERFORM_MOTION:
                data = {'object': user_intent["object"]}
                self.publisher.publish(
                    IntentMsg(intent=IntentMsg.PERFORM_MOTION, data=json.dumps(data)))
            elif user_intent["type"] == IntentMsg.START_ACTIVITY:
                data = {'goal': user_intent["object"],
                        'suggested_action_plan': user_intent["plan"]}
                self.publisher.publish(
                    IntentMsg(intent=IntentMsg.START_ACTIVITY, data=json.dumps(data)))


def main(args=None):
    rclpy.init(args=args)
    node = StringNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
