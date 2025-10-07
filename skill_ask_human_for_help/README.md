Ask human for help
====================

This skill allows the robot to ask a human for help when it is unable to
complete a task on its own.

The skill relies on two other skills, `/skill/ask` and `/skill/navigate_to_pose`.

To use it manually, you can follow these steps:
1. Launch the skill:

`ros2 launch ask_help ask_help.launch.py`

2. If not already running, ensure the `/skill/ask` and `/skill/navigate_to_pose` skills are running.

3. Trigger the skill by sending a request to the `/skill/ask_human_for_help` action goal:

```bash
ros2 action send_goal /skill/ask_human_for_help interaction_skills/action/AskHumanForHelp "meta:
  caller: ''
  priority: 0
person_ids: []
question_to_human: 'Can you put a bottle on my tray?'"
```

The robot will then ask the human for help with the specified task.

The person IDs can be specified to prioritize specific individuals, if available, or left empty to ask anyone nearby.