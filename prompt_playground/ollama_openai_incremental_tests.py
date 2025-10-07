from ollama import chat, Client
from openai import OpenAI
from pydantic import BaseModel
from typing import Literal, List
import os
from jinja2 import Template

from hri_actions_msgs.msg import Intent as IntentMsg


os.system('export OPENAI_API_KEY=""<insert_your_api_key_here>"')

# Define the data models for the chatbot response and the user intent
class Intent(BaseModel):
    type: Literal[IntentMsg.BRING_OBJECT,
                  IntentMsg.GRAB_OBJECT,
                  IntentMsg.PLACE_OBJECT,
                  IntentMsg.GUIDE,
                  IntentMsg.MOVE_TO,
                  IntentMsg.SAY,
                  IntentMsg.GREET,
                  IntentMsg.START_ACTIVITY,
                  ]
    object: str | None
    recipient: str | None
    input: str | None
    goal: str | None
    suggested_action_plan: List[str] | None


class ChatbotResponse(BaseModel):
    verbal_ack: str | None
    user_intent: Intent | None


client = Client(
  host='http://192.168.6.182:11434',
)

openAI_client = OpenAI()

#############################################

# Fetch the list of skills


SKILLS = [
    {"name": "say", "desc": "say something", "params": [
        "text"], "example": "say('Hello!')"},
    {"name": "ask", "desc": "request a specific bit of information from a user", "params": [
        "question"], "example": "ask('What is your name?')"},
    {"name": "navigate_to", "desc": "navigate to a location",
     "params": ["location"], "example": "navigate_to('kitchen')"},
    {"name": "look_for_item", "desc": "look for an item",     "params": [
        "rdf_pattern"], "example": "look_for_item('* hasColor blue')"},
    {"name": "ask_human_for_help",     "desc": "ask a human for help", "params": [
        "question"], "example": "ask_human_for_help('Can you open the door for me, please?')"},
    {"name": "carry_object",     "desc": "carry an object to a destination", "params": [
        "object", "location"], "example": "carry_object('book1', 'living_room')"},
]

# Define the knowledge base
kb = {
    "* rdf:type ZoneOfInterest": ["kitchen", "living room", "bedroom"],
    "* rdf:type Artefact": ["coffee", "milk", "sugar"],
    "myself isIn *": ["living room"]
}

# assemble the system prompt using Jinja2 templating.
# the template is call 'prompt.tpl.j2' and is located in the same directory as this script.

# PROMPT = "prompt.tpl.j2"
PROMPT = "prompt-linear-plan.tpl.j2"

template = Template(open(PROMPT).read())
system_prompt = template.render(
    robot_name="TIAGo",
    user_id="user123",
    skills=SKILLS,
    kb=kb
)

# print(system_prompt)
chat_history_ollama = [
        {
            'role': 'system',
            'content': system_prompt,
        },
        {
            'role': 'user',
            'content': '[human_1]: Get me a coffee.',
        }
    ]


chat_history_openai = chat_history_ollama.copy()

print("\n\n...waiting for chatbot answer...")
response = client.chat(
    messages=chat_history_ollama,
    # model='deepseek-r1:1.5b',
    model='phi4',
    options={'temperature': 0},
    format=ChatbotResponse.model_json_schema(),
    # host=ollama_host
)
res = ChatbotResponse.model_validate_json(response.message.content)
response = openAI_client.beta.chat.completions.parse(
                        model= "gpt-4o-mini",
                        messages= chat_history_openai,
                        temperature=0,
                        max_tokens=1000,
                        response_format=ChatbotResponse)
        
res_open_ai = response.choices[0].message.parsed
# for i in range(len(chat_history)):
#     message = chat_history[i]
#     if i == 0:
#         print("Initial prompt")
#     else:
#         print(message["role"] + ": " + message["content"]) 
# print(res)
new_chat = [  
        {
            'role': 'system',
            'content': "Internal update: the robot has started navigating to the kitchen.",
        },
        {
            'role': 'system',
            'content': "Now you are still a friendly robot, but you can't anymore generate plans, you can just talk about how the weather is starting to get good. You can also talk about other small-talk like hobbies or interests. Use the verbal_ack field for that. Do not ever provide a detected user_intent."
        },
        {   'role': 'user',
            'content': '[human_2]: Get me a coffee.',
        },
    ]

chat_history_ollama.append(
        {
            'role': 'assistant',
            'content': str(res),
        }
)
chat_history_ollama.extend(new_chat)

chat_history_openai.append(
        {
            'role': 'assistant',
            'content': str(res_open_ai),
        }
)
chat_history_openai.extend(new_chat)

response = client.chat(
    messages=chat_history_ollama,
    model='phi4',
    options={'temperature': 0},
    format=ChatbotResponse.model_json_schema(),
    # host=ollama_host
)

res = ChatbotResponse.model_validate_json(response.message.content)

response = openAI_client.beta.chat.completions.parse(
                        model= "gpt-4o-mini",
                        messages= chat_history_openai,
                        temperature=0,
                        max_tokens=1000,
                        response_format=ChatbotResponse)
        
res_open_ai = response.choices[0].message.parsed


new_chat= [
        {
            'role': 'system',
            'content': "Internal update: the robot has reached the kitchen.",
        },
        {
            'role': 'system',
            'content': "Environment update: human_123 is present in the kitchen.",
        },
        {   'role': 'system',
            'content': "Internal update: the robot has successfully approached human_123.",
        },
        {   'role': 'system',
            'content': "Internal update: The robot has asked human_123 for help.",
        },
        {   'role': 'system',
            'content': "human_3 has prepared a coffe and placed it on the robot's tray.",
        },
        {
            'role': 'system',
            'content': "Now you are still a friendly robot, but you can just tell the user that you are in a rush, and you can't help anymore. Use the verbal_ack field for that. Do not ever provide a detected user_intent."
        },
        {
            'role': 'system',
            'content': "The robot has started to bring back the coffe to the human_1"
        },
        {   'role': 'user',
            'content': '[human_4]: Hey, can you prepare a coffee for me as well?',
        },
    ]

chat_history_ollama.append(
        {
            'role': 'assistant',
            'content': str(res),
        }
)
chat_history_ollama.extend(new_chat)

chat_history_openai.append(
        {
            'role': 'assistant',
            'content': str(res_open_ai),
        }
)
chat_history_openai.extend(new_chat)

response = client.chat(
    messages=chat_history_ollama,
    model='phi4',
    options={'temperature': 0},
    format=ChatbotResponse.model_json_schema(),
    # host=ollama_host
)

res = ChatbotResponse.model_validate_json(response.message.content)
response = openAI_client.beta.chat.completions.parse(
                        model= "gpt-4o-mini",
                        messages= chat_history_openai,
                        temperature=0,
                        max_tokens=1000,
                        response_format=ChatbotResponse)
        
res_open_ai = response.choices[0].message.parsed


new_chat= [
        {
            'role': 'system',
            'content': "The robot has reached again human_1 and delivered the coffee.",
        },
        {
            'role': 'system',
            'content': "Now you are again the friendly robot, and you can generate plans using the initial instructions.",
        },
        {   'role': 'user',
            'content': '[human_5]: Hello, can you bring this book to the bedroom?',
        },
    ]


chat_history_ollama.append(
        {
            'role': 'assistant',
            'content': str(res),
        }
)
chat_history_ollama.extend(new_chat)

chat_history_openai.append(
        {
            'role': 'assistant',
            'content': str(res_open_ai),
        }
)
chat_history_openai.extend(new_chat)

response = client.chat(
    messages=chat_history_ollama,
    model='phi4',
    options={'temperature': 0},
    format=ChatbotResponse.model_json_schema(),
    # host=ollama_host
)

res = ChatbotResponse.model_validate_json(response.message.content)
response = openAI_client.beta.chat.completions.parse(
                        model= "gpt-4o-mini",
                        messages= chat_history_openai,
                        temperature=0,
                        max_tokens=1000,
                        response_format=ChatbotResponse)
        
res_open_ai = response.choices[0].message.parsed


print(" OLLAMA response:")
for i in range(len(chat_history_ollama)):
    message = chat_history_ollama[i]
    if i == 0:
        print("Initial prompt")
    else:
        print(message["role"] + ": " + message["content"]) 
print(res)

print()

print(" OPENAI response:")
for i in range(len(chat_history_openai)):
    message = chat_history_openai[i]
    if i == 0:
        print("Initial prompt")
    else:
        print(message["role"] + ": " + message["content"]) 
print(res_open_ai)