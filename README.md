<h2 align="center">
    <img src="https://yueyang1996.github.io/images/logo.png" width="200px"/><br/>
    Language Guided Generation of 3D Embodied AI Environments<br>
</h2>

<h5 align="center">
<img src="https://yueyang1996.github.io/images/office_example.png" width="800px"/><br/>
</h5>

<h4 align="center">
  <a href="https://arxiv.org/abs/2312.09067"><i>Paper</i></a> | <a href="https://yueyang1996.github.io/holodeck/"><i>Project Page</i></a>
</h4>

<h4 align="center">
    This version of Holodeck is integrated with ROS2.
</h4>

## Requirements
- Ubuntu 20.04 (Unity 2020.3.25f1 only runs on this Ubuntu version)
- Conda Environment
- C++ Compiler (```sudo apt install build-essential```)
- ROS2 Foxy
- Holodeck is based on [AI2-THOR](https://ai2thor.allenai.org/ithor/documentation/#requirements).

## Installation
Clone the repo and **change directory** into it.
```bash
git clone https://github.com/calebchia1807/Holodeck.git
```

After cloning the repo, run the following commands:
```bash
conda create --name holodeck python=3.10.12
conda activate holodeck
pip3 install -r requirements.txt
pip3 install --extra-index-url https://ai2thor-pypi.allenai.org ai2thor==0+8524eadda94df0ab2dbb2ef5a577e4d37c712897
pip3 install --upgrade torch
pip3 install moviepy==1.0.3
pip3 install --upgrade torchvision
ulimit -n 10000
```

## Data
Download the data by running the following commands:
```bash
python -m objathor.dataset.download_holodeck_base_data --version 2023_09_23
python -m objathor.dataset.download_assets --version 2023_09_23
python -m objathor.dataset.download_annotations --version 2023_09_23
python -m objathor.dataset.download_features --version 2023_09_23
```
by default these will save to `~/.objathor-assets/...`, you can change this director by specifying the `--path` argument.  If you change the `--path`, you'll need to set the `OBJAVERSE_ASSETS_DIR` environment variable to the path where the assets are stored when you use Holodeck.

## Usage
Run the following command in root of repo to generate a new environment:
```bash
python main.py --query "<ROOM DESIRED>" --openai_api_key <OPENAI_API_KEY>
```
- ```<ROOM DESIRED>```: Change to a query statement of room type you desire (example: "an apartment with 2 rooms")
- System uses `gpt-4o-2024-05-13`, **so please ensure you have access to an OPENAI API KEY.**

## Load the scene in Unity
1. Install [Unity](https://unity.com/download) and select the editor version `2020.3.25f1`.
2. Clone [AI2-THOR repository](https://github.com/allenai/ai2thor) and switch to the appropriate AI2-THOR commit.
```bash
git clone https://github.com/allenai/ai2thor.git
git checkout 07445be8e91ddeb5de2915c90935c4aef27a241d
```
3. Reinstall some packages:
```bash
pip uninstall Werkzeug
pip uninstall Flask
pip install Werkzeug==2.0.1
pip install Flask==2.0.1
```
4. Load `ai2thor/unity` as project in Unity and open `ai2thor/unity/Assets/Scenes/Procedural/Procedural.unity`.
5. In the terminal, run [this python script](connect_to_unity.py):
```bash
python connect_to_unity.py --scene <SCENE_JSON_FILE_PATH>
```
6. Press the play button (the triangle) in Unity to load the scene. No actions can be made as it is waiting for ROS2 commands, see below.

## ROS2 Integration
#### Ensure that ROS2 Foxy is installed for this to run.

Run the following to build the package:
```bash
cd ~/Holodeck
source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
```

Once the package is built, run the following to launch the package:
```bash
ros2 launch simulator_ros unity_controller.launch.py
```

There are the following functions in this package:
1. Publish ```/cmd_vel``` to control the motion. Run:
   ```bash
   ros2 topic pub -r 50 cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
   ```
   #### linear is in m/s; angular is in degrees/s (can be changed to randian/s, refer to [this ros node](src/simulator_ros/simulator_ros/unity_nav.py).
   | Parameter | Negative /cmd_vel | Positive /cmd_vel |
   | :----: | :----: | :----: |
   | linear x | move front | move back |
   | linear y | move left | move right |
   | linear z | stand | crouch |
   | angular x | NA | NA |
   | angular y | look up | look down |
   | angular z | rotate left | rotate right |

2. Control motion via ```teleop_twist_keyboard```. Run:
   ``` bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   #### linear is in m/s; angular is in degrees/s (can be changed to randian/s, refer to [this ros node](src/simulator_ros/simulator_ros/unity_nav.py).
   | Key | Movement | Key | Movement |
   | :----: | :----: | :----: | :----: |
   | k / K | stop | u | move forward & rotate left |
   | i / I | move forward | o | move forward & rotate right |
   | , / < | move back | U | move forward & move left |
   | J | move left | O | move forward & move right |
   | L | move right | m | move back & rotate right |
   | j | rotate left | . | move back & rotate left |
   | l | rotate right |  M | move back & move left |
   | t | stand | > | move back & move right |
   | b | crouch |

3. ROS2 Service for Stand & Crouch. Run:
   ``` bash
   ros2 service call /unity_stand_crouch/<service> std_srvs/srv/Trigger
   ```
   ```<services>``` available: ```stand``` or ```crouch```

4. Live Feed in ```rviz2```.
  
   Under image in ```rviz2```, there are 5 feeds available. The ```topic``` available are as follows:
   ```bash
   unity_rgb_camera
   unity_bgr_camera
   unity_depth_camera
   unity_segmentation_camera
   unity_bounding_box
   ```
        
## Adding new THOR assets
Please refer to this repo [ObjaTHOR](https://github.com/calebchia1807/objathor) on how to run.

After converting .glb assets into THOR assets, you need to manually add the asset into the .json generated earlier.

Refer to lines 155 to 266 of this [.json file](/living_room.json) for examples for how to add.

** Future work to automate this process!
