# LEO-RobotAgent
Paper: LEO-RobotAgent: A General-purpose Robotic Agent for Language-driven Embodied Operator

[![arXiv](https://img.shields.io/badge/arXiv-2512.10605-b31b1b.svg)](https://arxiv.org/abs/2512.10605)

[中文 README](./README_cn.md)

## Introduction

LEO-RobotAgent is a general-purpose robotic intelligent agent framework based on Large Language Models (LLMs). Under this framework, LLMs can operate **different types of robots** in **various scenarios** to complete **unpredictable complex tasks**, demonstrating high generalizability and robustness.

![agent_detail](./docs/agent_detail.png)

The LLM-based general-purpose robotic agent framework, LEO-RobotAgent, is shown in the figure above. The large model can autonomously think, plan, and act within this clear framework. We provide a modular and easily registrable collection of tools, enabling the LLM to flexibly invoke various tools according to different needs. At the same time, the framework provides a human-computer interaction mechanism, allowing the algorithm to collaborate with humans like a partner.

The LLM relies on preset prompts and user tasks to output information, actions, and action parameters. The tool collection can cover various domains based on actual situations, requiring basic information such as enable status, tool name, corresponding function, and tool description. Observations provide varied feedback content depending on the tool. During the loop, the History is continuously accumulated for subsequent operations by the LLM.

![agent_system](./docs/agent_system.png)

The figure above shows an application system designed around LEO-RobotAgent. We built this complete system based on ROS and Web technologies. Users can directly operate the visual interface to configure existing tools, converse and interact with the Agent, monitor topics, etc. The system is easy to extend and get started with in terms of tool registration and node management.

## Demonstration

[![Watch the video](./docs/cover.png)](https://youtu.be/f0-ZOk4GSFY)

The demonstration video above presents four sets of experiments, namely basic features verification, real UAV experiment, UAV urban searching experiment, and long-horizon task experiment with the wheeled robot equipped with the robotic arm.

![effect](./docs/sim2real.png)

The simulation and corresponding real-world experiments are shown above. An example of the Agent's operation process and output during a task can be found in [this file](./docs/agent_run.txt).

## Project Content

![resource](./docs/resource.png)

Our framework has been verified for feasibility on UAVs, custom-made wheeled mobile robots (with robotic arms), and mechanical dogs. The project contains their relevant ready-made control nodes.

The maps provided in `src/agent/world` are based on [gazebo_models_worlds_collection](https://github.com/leonhartyao/gazebo_models_worlds_collection).

# 🔥 Environment & Dependencies
- Development Environment: Ubuntu 20.04 + ROS Noetic. The core framework works in other environments, but robots may need self-adaptation. **The following installation steps may omit some detailed libraries; they are for reference only. Supplements are welcome.**

## General Configuration
1. First, **download this repository to your workspace**.
2. Install Python dependencies (Python 3.8 confirmed to work):

```bash
pip install -r requirements.txt
```

3. Install web_video_server and rosbridge:
```bash
sudo apt install ros-noetic-rosbridge-suite ros-noetic-web-video-server
```
4. Install [gazebo_models_worlds_collection](https://github.com/leonhartyao/gazebo_models_worlds_collection).

5. Configure LLM API:
```bash
echo 'export OPENAI_API_KEY="your key"' >> ~/.bashrc
echo 'export OPENAI_BASE_URL="your url"' >> ~/.bashrc
source ~/.bashrc
```

Note: This project uses the [Qwen3 series models](https://bailian.console.aliyun.com/), including Qwen-VL, so adaptation is only ensured for these models. If there are conflicts in the LLM output format, you can modify it in `src/agent/src/api_agent.py`.

Next are the dependencies required for the corresponding robots. Below are the robots already adapted to LEO-RobotAgent. Configure as needed. Remember to `catkin_make` and `source` at the end.

## 🛸 UAV:
1. [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)
```bash
# Download
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
# Finish remaining downloads
cd PX4-Autopilot/ 
git submodule update --init --recursive
# Execute script
cd ..
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
# If errors occur, execute:
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --fix-missing
# Environment variables: Use nano or gedit to enter bashrc and add to the end
sudo gedit ~/.bashrc
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
# Finally
source ~/.bashrc
```

2. Mavros
```bash
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
```

3. [QGroundControl](https://docs.qgroundcontrol.com/Stable_V4.3/zh/qgc-user-guide/getting_started/download_and_install.html)

## 🦾 Wheeled Robot with Robotic Arm
1. MoveIt
```bash
sudo apt install ros-noetic-moveit
sudo apt install ros-noetic-moveit-setup-assistant
```

2. Plugins:
```bash
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers

# Gripper plugin, can be installed in your workspace or globally
cd ~/catkin_ws/src
git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

## 🐶 Mechanical Dog (Unitree)
Unitree GO1 (others are also fine), install the following content yourself: [unitree_guide](https://github.com/unitreerobotics/unitree_guide), [unitree_ros](https://github.com/unitreerobotics/unitree_ros), [unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real).

Then execute the following to adapt the topics to the LEO-RobotAgent framework:
```bash
cp src/agent/utils/KeyBoard.cpp src/unitree_guide/unitree_guide/src/interface/KeyBoard.cpp
```

# 🔥 Running the Agent
## Web Interface
![web_ui](./docs/webui.png)

Our application system is Web-based. The interface is shown above. The **System** panel in the top left can start various preset terminal commands (including but not limited to roslaunch, rosrun). You can also add your own. LEO-RobotAgent is our core architecture; all buttons will actually open a terminal (closing the corresponding terminal will shut down the node), facilitating debugging output. The **Camera** panel allows switching and viewing Image format topics.

The **Tools** panel allows you to set the tools available to the Agent. You can check tasks to enable them, or double-click to change the configuration like in Excel. You can also add new tools via the bottom button (not visible in the image). Any changes must be saved by clicking Save. **The LEO-RobotAgent node must be restarted for changes to take effect.**

On the right is the **Chat Interface**. Entering commands can issue tasks. You can also input during task execution to interrupt, temporarily modify tasks, or point out errors. After the task for the current stage is completed, the Agent will output the final answer in a green bubble. You can continue to issue tasks afterwards (memory is retained). Blue bubbles indicate tool calling Actions, and yellow indicates Observation results.

**Preset questions, tool configurations, and preset terminal commands are saved under `src/agent/config`**. They are automatically loaded every time the web interface is opened. You can perform more detailed additions, deletions, and modifications there, or check which program file is running and develop it yourself.

## Running the Program
1. First, start the server: `python3 src/agent/webui/server.py`.
2. Next, open in a browser: `src/agent/webui/web_ui.html`. Then start RosBridge and VideoServer (if you want camera feed) in the System panel.

3. Then, depending on the robot:
   - UAV:
     1. Configure and save needed tools in the Tools panel (`uav_fly` is necessary).
     2. Sequentially start via buttons: QGC, UAV sim, UAV fly (wait for gazebo to load fully), Vision, LEO-RobotAgent.

   - Wheeled Robot with Arm:
     1. Configure and save needed tools in the Tools panel (`car_run`, `arm_grasp` are necessary).
     2. Sequentially start via buttons: Car sim, Car ctrl (wait for gazebo to load fully), Arm ctrl, Vision, LEO-RobotAgent.

   - Mechanical Dog:
     1. Configure and save needed tools in the Tools panel (`dog_run` is necessary).
     2. Sequentially start via buttons: Dog sim, Dog joint (wait for gazebo to load fully), Dog ctrl, Vision, LEO-RobotAgent.

4. Finally, input commands in the chat interface to issue tasks for automatic execution.

## About the Vision Node
- The Vision node provides VLM and object detection as visual tools. The implementation method of VLM can be rewritten in `vision.py` according to different model interface implementations; object detection uses `yolov8l-worldv2`. You can choose and download models from [Ultralytics](https://github.com/ultralytics/ultralytics) and place them in `src/agent/weights`.
- You can fill in `uav`, `car`, or `dog` in `src/agent/config/vision_device.txt` to adapt to cameras and other topics.

## Developing New Tools
If you want to develop new tools based on this project, here is an example of creating the simplest tool.
1. First, define a new function `add` under `AgentTools` in `src/agent/src/tools.py`:

```python
def add(self, nums):
    return nums['a'] + nums['b']
```

2. Then add a tool in the Web Tools panel, fill in the corresponding content, check and save it, for example:
`Name: add, Function: add, Description: Input a dictionary with a, b. Return: the result of a + b.`

![add](./docs/tool_add_example.png)

3. It is now ready for use. From this, you can also implement complex algorithms in your own project and then register them into `tools.py` by setting ROS topics as interfaces.

## Modifying the Prompt
`src/agent/src/api_agent.py` contains the core code of this framework. The prompts within can be modified according to your own tasks. Tools and Vision also use LLM and VLM, which can be modified independently.

# 🔥 Manual Execution (Legacy)
- You can still manually open multiple terminals to run these commands.

## General Nodes
- Vision Node
```bash
source ./devel/setup.bash && rosrun agent vision.py
```

- Agent Node
```bash
source ./devel/setup.bash && rosrun agent api_agent.py
```

## 🛸 UAV
- QGC Ground Station (in home directory)
```bash
./QGroundControl.AppImage
```

- Mavros + PX4 launch file
```bash
roslaunch px4 mavros_posix_sitl.launch
# Choose your own world
roslaunch px4 mavros_posix_sitl.launch world:=/path/to/your.world
# Takeoff/Land commands
commander takeoff
commander land
```

- UAV Control Node
```bash
source ./devel/setup.bash && rosrun agent fly.py
```

## 🦾 Wheeled Robot with Arm
- Car Launch
```bash
source ./devel/setup.bash
# Without Arm
roslaunch agent gazebo_car.launch
# With Arm
roslaunch armcar_moveit_config demo_gazebo.launch
# No GUI
roslaunch armcar_moveit_config demo_gazebo.launch gazebo_gui:=false
```

- Car Nodes
```bash
# Car Control Node
source ./devel/setup.bash && rosrun agent car_ctrl.py

# Arm Control Node
source ./devel/setup.bash && rosrun agent arm_ctrl.py
```

## 🐶 Mechanical Dog
- Dog Launch
```bash
source ./devel/setup.bash && roslaunch unitree_guide gazeboSim.launch 
```

- Dog Nodes
```bash
# Dog Joint Control
./devel/lib/unitree_guide/junior_ctrl

# Dog Control Node
source ./devel/setup.bash && rosrun agent dog_ctrl.py
```

# 🔥Ciation
If you find this project useful in your research, please consider citing:

```bibtex
@article{chen2025leorobotagent,
  title={LEO-RobotAgent: A General-purpose Robotic Agent for Language-driven Embodied Operator},
  author={Chen, Lihuang and Luo, Xiangyu and Meng, Jun},
  journal={arXiv preprint arXiv:2512.10605}, 
  year={2025}
}
```