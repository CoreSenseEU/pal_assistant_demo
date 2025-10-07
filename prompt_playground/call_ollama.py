from ollama import chat, Client
from pydantic import BaseModel, Field
from typing import Literal, List, Annotated, Union, Dict, Optional
from typing_extensions import TypedDict
import csv
from jinja2 import Environment, FileSystemLoader
import datetime
import os
from hri_actions_msgs.msg import Intent as IntentMsg
import time


client = Client(
    host='http://192.168.88.110:11434',
)

# class SaySkillParams(TypedDict, total=False):
#     text: str


# class SaySkill(BaseModel):
#     skill: Literal["say"]
#     # params: Dict
#     # params: Dict[Literal["text"], Optional[str]]
#     params: SaySkillParams
#     result: Optional[str]


# class AskSkillParams(TypedDict, total=False):
#     item: str


# class AskSkill(BaseModel):
#     skill: Literal["ask"]
#     # params: Dict
#     # params: Dict[Literal["item"], Optional[str]]
#     params: AskSkillParams
#     result: Optional[str]


# class NavigateToSkillParams(TypedDict, total=False):
#     location: str


# class NavigateToSkill(BaseModel):
#     skill: Literal["navigate_to"]
#     # params: Dict
#     # params: Dict[Literal["location"], Optional[str]]
#     params: NavigateToSkillParams
#     result: Optional[str]


# class LookForHumanSkill(BaseModel):
#     skill: Literal["look_for_human"]
#     params: Optional[Dict]
#     result: Optional[str]


# class AskHumanForHelpSkillParams(TypedDict, total=False): # Keys optional
#     question: str
#     human_id: str


# class AskHumanForHelpSkill(BaseModel):
#     skill: Literal["ask_human_for_help"]
#     # params: Dict
#     # params: Dict[Literal["question","human_id"], Optional[str]] # All keys cumpulsory
#     params: AskHumanForHelpSkillParams
#     result: Optional[str]

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


# class StartActivityData(BaseModel):
#     type: Literal[IntentMsg.START_ACTIVITY]
#     object: str = Field(..., description="The activity to start. E.g. 'prepare_coffee' or 'dance'")
#     plan: List[GenericSkill] = Field(..., description="The plan to execute")


# class PerformMotionData(BaseModel):
#     type: Literal[IntentMsg.PERFORM_MOTION]
#     object: str = Field(..., description="The motion to perform")


# class Intent(BaseModel):
#     type: Literal[IntentMsg.MOVE_TO,
#                   IntentMsg.PERFORM_MOTION,
#                   IntentMsg.START_ACTIVITY,
#                   ] | None = Field(..., description="The detected user intent")
#     data: Annotated[Union[MoveToData, PerformMotionData, StartActivityData], Field(discriminator='type')]


# class ChatbotResponse(BaseModel):
#     say: str | None = Field(..., description="The verbal sentence that the robot will say. Start always with 'listen to me'")
#     user_intent: Intent | None = Field(..., description="The detected user intent")
#     result: str | None = Field(..., description="The result of the chatbot action")

#############################################

# Fetch the list of skills
# TODO, allow to return values and use them later.

SKILLS = [
    {"name": "say", "description": "You can use this skill to say something that is not an ackowledgement or asking for clarifications.",
        "params": ["text"],
        "result": None,
        "example": "say('Sure, I can do that')"},
    {"name": "ask", "description": "request a specific bit of information from a user",
        "params": ["item"],
        "result": ["answer"],
        "example": "ask('name')"},
    {"name": "navigate_to", "description": "navigate to a location",
        "params": ["location"],
        "result": None,
        "example": "navigate_to('kitchen')"},
    {"name": "look_for_human", "description": "look for a human around you",
        "params": [],
        "result": ["human_id"],
        "example": "look_for_human()"},
    {"name": "ask_human_for_help", "description": "ask a human for help",
        "params": ["question", "human_id"],
        "result": ["acknowledgement"],
        "example": "ask_human_for_help('Can you open the door for me, please?')"},
]

# TODO, this should come from KB
kb = {
    "zois": ["kitchen", "dinning_room", "bathroom", "bedroom"],
    "current_position": "dinning_room"
}

# PROMPT = "prompt.tpl.j2"
# PROMPT = "prompt-linear-plan.tpl.j2"
PROMPT = "chat_and_plan.tpl.j2"


def parse_prompt_to_msgs(prompt):
    """Split the prompt by sections starting with roles (SYSTEM, USER, ASSISTANT) and
    return a list of messages with the corresponding role and content."""

    messages = []
    lines = prompt.split("\n")
    role = None
    content = ""
    for line in lines:
        if line.startswith("SYSTEM:"):
            if role:
                messages.append({"role": role, "content": content})
                content = ""
            role = "system"
        elif line.startswith("USER:"):
            if role:
                messages.append({"role": role, "content": content})
                content = ""
            role = "user"
        elif line.startswith("ASSISTANT:"):
            if role:
                messages.append({"role": role, "content": content})
                content = ""
            role = "assistant"
        else:
            content += "\n" + line
    if role:
        messages.append({"role": role, "content": content})
    return messages


# create a jinja environment and load the Template
env = Environment(loader=FileSystemLoader('.'))

template = env.get_template(PROMPT)
prompt = template.render(
    robot_name="TIAGo",
    skills=SKILLS,
    kb=kb
)


def log_prompt(prompt):
    # first, open all the files in rendered_prompts and check whether one existing file is identical to the current PROMPT
    # if so, do not save the current prompt and return the filename of the existing file
    # if not, save the current prompt in a new file and return the filename of the new file
    for file in os.listdir("rendered_prompts"):
        with open(f"rendered_prompts/{file}", "r") as f:
            if f.read() == prompt:
                print(f"[prompt already exists in file {file} - reusing it]")
                return f"rendered_prompts/{file}"

    # save the rendered prompt in a file call rendered_prompt/p-<date>-<time>.txt
    filename = f"rendered_prompts/p-{datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.txt"
    with open(filename, "w") as f:
        f.write(prompt)
    print(f"[new prompt saved in file {filename}]")
    return filename


print(prompt)

filename = log_prompt(prompt)

idx = 0
temperature = 0
# model = 'deepseek-r1:1.5b'
# model = 'deepseek-r1:8b'
# model = 'deepseek-r1:14b' # Decent, around 20 s for plans, less than 5 for say
model = "phi4" # Good, around 20 s for plans, less than 5 for say
# model = "gemma2:9b" # Still slow,around 20 s
# model = "llama3.1:8b" # Not very good
# model = "mistral-nemo" # Not very good


print(f"\n\n...waiting for chatbot answer -- round {idx}...")
print()

chat_history = parse_prompt_to_msgs(prompt)

if chat_history[-1]["role"] != "user":
    user_sentence = input("Enter your sentence: ")
    chat_history.append({"role": "user", "content": user_sentence})

response = client.chat(
    messages=chat_history,
    model=model,
    options={'temperature': temperature},
    format=ChatbotResponse.model_json_schema(),
)

print(response.message.content)

chat_history.append({"role": "assistant", "content": response.message.content})

while True:
    # idx += 1
    # plan = res.user_intent.suggested_action_plan
    # if not plan:
    #     plan = []
    # with open('coffee-planning.csv', 'a', newline='') as csvfile:
    #     planwriter = csv.writer(csvfile, quoting=csv.QUOTE_MINIMAL)
    #     planwriter.writerow(
    #         [idx, model, filename, len(plan), temperature] + plan)
    # if idx % 50 == 0:
    #     temperature += 0.05
    # if temperature > 0.5:
    #     break
    # break

    # Ask for keyboard input to detect next user sentence to process
    user_sentence = input("Enter your sentence: ")
    init_time = time.time()
    if user_sentence == "exit":
        break
    else:
        print(f"\n\n...waiting for chatbot answer -- round {idx}... \n")
        chat_history.append({"role": "user", "content": user_sentence})
        response = client.chat(
            messages=chat_history,
            model=model,
            options={'temperature': temperature},
            format=ChatbotResponse.model_json_schema(),
        )
        print(time.time() - init_time)
        print(response.message.content)

        chat_history.append({"role": "assistant", "content": response.message.content})
