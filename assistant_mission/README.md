PAL demo of an assistant robot, integrating the latest developments in HRI and navigation.
====================
Welcome to the PAL demo.

## Prerequisites


## Compile, install and launch the mission controller

You need a ROS 2 environment to compile the demo.

```
> cd /home/user/exchange/ws
> colcon build
> source install/setup.bash
```

You can now start your mission controller with:

```
> ros2 launch assistant_mission assistant_mission.launch.py
```

## Testing

Send an intent-plan to the robot:

```
ros2 topic pub -1 /intents hri_actions_msgs/msg/Intent "intent: '__intent_start_activity__'
data: '{\"goal\":\"talk_twice\", \"suggested_action_plan\": [{\"skill\":\"say\", \"params\":{\"input\":\"hello\"}}, {\"skill\":\"say\", \"params\":{\"input\":\" raquel\"}}]}'
source: ''
modality: ''
priority: 0
confidence: 0.0"
```
