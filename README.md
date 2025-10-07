# PAL Assistant Demo

PAL demo of our interactive robots

## Interactive simulator

A reduced version of the demo can be run in simulation, using the interaction simulator, which includes face detection, chat interface and the ability to add some objects to the scene.

First, make sure to have the interaction simulator package installed and copy the files in the `sim_config_files` folder to `~/.pal/config` in your docker container.

Then, launch the simulation dependencies and the assistant mission:

```bash
ros2 launch assistant_mission simulation_dependencies.launch.py 

ros2 launch assistant_mission assistant_mission.launch.py 
```

Type in the right chat an instruction, or publish directly plans through the intent topic:

```bash
ros2 topic pub -1 /intents hri_actions_msgs/msg/Intent "intent:  '__intent_start_activity__'
data: '{\"goal\": \"get_coffee\", \"instruction\": \"get me a coffee\", \"suggested_action_plan\": [{\"skill\": \"navigate_to_zone\", \"params\": {\"location\": \"kitchen\"}}, {\"skill\": \"ask_human_for_help\", \"params\": {\"request\": \" Hello, can you pick up a coffee mug and place it on my tray?\"}}, {\"skill\": \"navigate_to_zone\", \"params\": {\"location\": \"living_room\"}}, {\"skill\": \"say\", \"params\": {\"input\": \"Here is your coffee.\"}}]}'
source: 'unknown'
modality: '__modality_speech__'
priority: 0
confidence: 0.0"
```

You can then modify the skills code to trigger different behaviours, like failures in some of them. By default it is using the real say, ask_human_for_help and explain skills, and the simulated navigation skills.

## Real robot

### Instalation and setup

#### Install the navigation skills

```bash
sudo apt install pal-alum-skill-navigate-to-pose
sudo apt install pal-alum-skill-navigate-to-zone
```

#### Install remap-related packages

```bash
sudo apt install pal-alum-remap-manager
sudo apt install pal-alum-remap-plugin-robot
sudo apt install pal-alum-remap-plugin-nav-map
sudo apt install libopenvdb8.2
sudo apt install pal-alum-yolo-ros-onnx
sudo apt install pal-alum-yolo-models
```

Run ldconfig to update the library cache:

```bash
sudo ldconfig
```

Deploy the plugin objects package from the `demo` branch:

```bash
git clone -b demo git@gitlab:interaction/remap_plugin_objects.git
ros2 run pal_deploy deploy -p pal-alum-remap-plugin-objects tiago-pro-0c
```

#### Deploy the demo packages

```bash
git clone git@gitlab:demos/pal_assistant_demo.git
ros2 run pal_deploy deploy -p assistant_mission tiago-pro-0c
```

#### Copy the configuration files

```bash
scp -r robot_config_files/config pal@tiago-pro-0c:/home/pal/.pal/config
```

#### Copy or create the map

##### Option A: Reuse on of the existing maps

```bash
scp -r robot_config_files/llm_demo pal@tiago-pro-0c:/home/pal
```

##### Option B: Create a new map

Set up connection with the robot and open rviz with your PC, or ssh -X -C on the robot and open rviz there. Use the rviz config from the llm_demo folder.

```bash
pal module stop localization

pal module start slam
```

Move the robot with the joystick and create the map. When finished, save the map:

```bash
ros2 run nav2_map_server map_saver_cli -t /map -f /home/pal/llm_demo/my_map --free 0.196 --occ 0.65
pal module stop slam
pal module start localization
```

Prepare your environment.yaml file that creates building with floor, map, zones, etc. You can use the one in the llm_demo folder as a template, while changing the map path to your new map. 

#### Activate the desired map
Update the used environment:

```bash
pal module stop advanced_navigation
sudo rm ~/.pal/stores.db
ros2 run eulero_utils init_db_from_yaml /home/pal/llm_demo/environment.yaml
pal module start advanced_navigation
```

In rviz move the zones to the desired locations, and add new ones if needed. To move them you need the "interact" and "zones" buttons in the top toolbar (add it with the `+` button if you don't have them). Save the final configuration to the environment.yaml (this is needed for remap):

```bash
 ros2 run eulero_utils dump_db_to_yaml /home/pal/llm_demo/environment.yaml
```

#### Enable all the required modules

```bash
pal module enable asr_vosk && pal module enable communication_hub && pal module enable hri_body_detect && pal module enable hri_person_manager && pal module enable semantic_state_aggregator && pal module enable chatbot_ollama && pal module enable skill_ask_human_for_help && pal module enable skill_explain && pal module enable skill_navigate_to_pose && pal module enable skill_navigate_to_zone && pal module enable component_explain_ask_human_for_help && pal module enable component_explain_navigation && pal module enable component_explain_planner && pal module enable component_explain_say && pal module enable assistant_mission && pal module enable attention_manager
```

#### Optionally, use the external usb microphone

First make sure that the 99-audio-usb-mic-config.yaml is being used in .pal/config/.

Set the set-default-source to the usb microphone (found in `pacmd list-sources`), in `/etc/pulse/default.pa.d/pal-config.pa` For example `set-default-source alsa_input.usb-Jieli_Technology_USB_Composite_Device_433130353331342E-00.mono-fallback`

Finally run:

```bash
pulseaudio -k
pal module restart pulseaudio
pal module disable respeaker_ros && pal module enable audio_capture
```

#### Set the LLM endpoint and model

Modify the parameters in `.pal/config` for chatbot_ollama and explainability modules. If using open ai, provide the api key in the `api_key` parameter as well.

#### Restart the pal module manager

```bash
pal module_manager restart
```

### Usage

#### Check installation/configuration is fine

1. Ensure that the right deployed_ws is there. There might be a copy on the robot at deployed_ws_llm_demo.

2. Ensure that all the configuration files are in ~/.pal/config

3. Ensure that the map you want is used, otherwise run the steps to activate the desired map from the previous section.

4. Ensure that all the necessary modules are enabled, otherwise run the command to enable them from the previous section.


#### Disable unused controllers to save on CPU

```bash
ros2 control switch_controllers --deactivate gripper_right_controller arm_right_controller gripper_left_controller arm_left_controller
```

#### Check that the camera is publishing

```bash
ros2 topic echo /head_front_camera/color/image_raw
```

If it is not, restart the rgbd module, and if it still does not work, restart the robot.

#### Set up navigation

Localize the robot, by launching rviz and setting the initial pose with the "2D Pose Estimate" button. Then, move a bit the robot with the joystick until the laser scan matches the map. It is recommended to run rviz into the robot to run the rviz configuration, and connect through cable with a `ssh -X pal@10.68.0.1`:

```bash
rviz2 -d /home/pal/llm_demo/llm_demo.rviz
```

If you see that the robot model is not full, or see some tf warnings, you might need to restart the robot.

After being localized, you can set a goal with the "2D Nav Goal" button in rviz to check that navigation works. Then, close rviz (if running on the robot, to save CPU) and disconnect the cable.

#### Run the demo

1. Restart some of the main modules for the demo, to start fresh and avoid issues. Use this command later to restart the demo if needed.

```bash
pal module restart skill_ask_human_for_help && pal module restart chatbot_ollama && pal module restart communication_hub && pal module restart assistant_mission
```

2. See the logs of assistant mission to see its progress through the tasks

```bash
pal module log assistant_mission tail -fn 3333
```

3. Tell the robot to go to one of the rooms (it does not matter if the robot is already there). This is a way to let the robot know where it starts.

4. Ask it to bring you something or deliver something to another room.

You can talk directly to the robot, but if asr is too bad you can lower the volume and then manually send fake speech recognition results with:

```bash
ros2 param set /volume capture 0
ros2 topic pub -1 /humans/voices/anonymous_speaker/speech hri_msgs/msg/LiveSpeech "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
incremental: ''
final: 'Hello'
confidence: 0.0
locale: ''"
```

#### (Optional) Use remap to detect and map objects

Due to a not fully understood issue with ROS2 bandwidth or buffers, the depth image stops publishing if there are more than two suscribers. For this reason, the remap objects plugin does not work at the same time as hri_body_detect. Therefore, you should stop hri_body_detect if you want to use remap objects. Once those objects are detected and mapped, you can stop the remap objects plugin and restart hri_body_detect. Another option is to run only remap to reason about objects (and visualize the knowledge graph in `http://tiago-pro-0c:8010/`), and not use body detection at all (which is needed to approach humans, but not to navigate or deliver objects). Finally, the demo can also be run without remap, but then you need to specify to the robot the room name where to bring the object, instead of just the object name.

In any case, the following commands are used to run remap:

```bash
ros2 launch remap_manager remap_manager.launch.py
ros2 service call /remap_manager/add_plugin remap_msgs/srv/AddPlugin "plugin_name: 'PluginObjects'
threaded: false"
ros2 service call /remap_manager/add_plugin remap_msgs/srv/AddPlugin "plugin_name: 'PluginNavMap'
threaded: false" 
ros2 service call /remap_manager/add_plugin remap_msgs/srv/AddPlugin "plugin_name: 'PluginRobot'
threaded: false"
ros2 launch yolo_ros_onnx yolo_ros_onnx.launch.py model:=yolo11n-seg
```

Optionally, yolo can be run externally in another PC (connected via cable), by setting the ROS2 connection and running yolo there. Moreover, other bigger yolo models can be used, by pointing to a local file, and performance can be modified by playing with parallel execution. GPU acceleration is possible, but the installation of the onnx cuda provider is fairly complex (LFE can help with this).

Example of running yolo externally with a bigger model:

```bash
ros2 launch yolo_ros_onnx yolo_ros_onnx.launch.py model_path:=/home/user/exchange/yolo_onnx_models/segmentation/yolo11m-seg.onnx execution_mode:=parallel inter_op_num_threads:=6 intra_op_num_threads:=6
```

### Troubleshooting

There are several modules that do not sometimes work properly, and the solution is to restart them. The main ones are:

- rgbd: check it is publishing with `ros2 topic echo /head_front_camera/aligned_depth_to_color/image_raw --field header`, if not restart the module, or even the robot if it does not work. Anyway, the topic will stop publishing if there are more than two suscribers, so you might need to stop some modules (like hri_body_detect) if you want to use remap objects (see above)
- navigation: if the robot does not move at all, and the joystick has no priority, restart navigation. However, if the robot moves with bad performance it might be related to the controller not being able to follow the path, which could be related to a CPU overload.
- skill_ask_human_for_help: due to issue with hri_listener, after some time it can't retrieve the human's transforms.

Moreover, navigation configuration was modified to change maximum speeds and footprint, which might need to be tailored depending on your environment. The configuration files are in `/home/pal/.pal/config/`.

