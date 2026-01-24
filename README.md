
# Open Manipulator X Teleoperation (ROS 2 Humble)

This repository contains a single ROS 2 package (`teleop_omx`) providing a vision-based teleoperation pipeline for the ROBOTIS Open Manipulator X arm
## Prerequisites

  * Ubuntu 22.04 LTS
  * ROS 2 Humble Hawksbill
    ```bash
    sudo apt update
    sudo apt install ros-humble-desktop
    ```
  * Open Manipulator X driver
    ```bash
    sudo apt install \
      ros-humble-open-manipulator-x-controller \
      ros-humble-open-manipulator-x-msgs
    ```
  * Development tools
    ```bash
    sudo apt install python3-colcon-common-extensions git
    ```
  * Python dependencies (from `setup.py`)
    ```bash
    pip install numpy scipy opencv-python mediapipe matplotlib
    ```

## Installation
The installation of the repository can be done automatically by running the auto1.sh and auto2.sh in your command prompt as follows:

1. **Download auto1.sh and auto2.sh in your system**
   **Run the following script in the terminal of the folder where the files are saved**
   
   ```bash
   bash auto1.sh
   # Follow the instructions given on your terminal screen and continue with the installation 
    ```

2. **Once the system reboots, once again open the terminal of the folder where the files are saved**

   ```bash
   bash auto2.sh
   # Follow the instructions given on your terminal screen and continue with the installation 
    ```

The installation can be done manually as follows.

1.  **Install the DynamixelWizard2 for Linux from this link:**
    **https://www.robotis.com/service/download.php?no=1671**
    **Open the Downloads folder and check if `DynamixelWizard2Setup-linux-x64.run` is downloaded or not**
    
    ```bash
    cd ~/Downloads
    chmod +x DynamixelWizard2Setup-linux-x64.run
    ./DynamixelWizard2Setup-linux-x64.run
    ```

    **After installation, add your user to the dialout group to access USB devices:**

    ```bash
    sudo usermod -aG dialout <your_username>
    sudo reboot # This needs to be done to apply the changes done until now
    ```

2.  **Update your ~/.bashrc environment with the shortcuts below for easy use (Optional but recommended)**
    
    ```bash
    nano ~/.bashrc
    ```

    **Add these lines at eof in your bashrc**

    ```bash
    # Colcon argcomplete
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    
    # OMX workspace
    source ~/omx/install/setup.bash
    # Aliases
    alias srcs='source install/setup.bash'
    alias om='ros2 run open_manipulator_x_controller open_manipulator_x_controller'
    alias ho='ros2 run omx_position_control basic_robot_control'
    alias hoo='ros2 run omx_position_control basic_robot_control2'
    alias lau='ros2 launch omx_position_control position_control.launch.py'
    alias cam='python3 apture.py position'
    alias tp='ros2 topic echo /target_pose'
    alias ap='ros2 topic echo /actual_pose'
    alias el='ros2 launch omx_position_control evaluation.launch.py'
    alias web='ros2 run omx_position_control websocket_bridge_node'
    alias web2='ros2 run omx_position_control websocket_bridge_node2'
    ```
    
    **Save and Exit.**
    ```bash
    source ~/.bashrc
    ```

3.  **Create a workspace named `omx`**

    ```bash
    mkdir -p ~/omx
    cd ~/omx
    ```

4.  **Download and install `src.zip` inside `omx/`**

    ```bash
    git clone https://github.com/cake-lab/AR-OMX.git src
    ```
    
    **or**

    ```bash
    wget <src_zip_link> -O src.zip
    unzip src.zip
    rm src.zip

    ```

5.  **Build the workspace**

    ```bash
    colcon build --symlink-install
    ```

5.  **Source the setup files (add to `~/.bashrc`)**

    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source ~/omx/install/setup.bash"  >> ~/.bashrc
    source ~/.bashrc
    ```

## Running the Teleop Pipeline

Open three to four terminals (or tabs). In each terminal, first source your environment:

```bash
cd ~/omx/
source install/setup.bash # or simply run srcs
```

The aliases that have been added into your "~/.bashrc" file are:

| Alias  | Command                                                                | Purpose                                                |
| :----- | :--------------------------------------------------------------------- | :------------------------------------------------------|
| `srcs` | `source install/setup.bash`                                            | Source the local OMX workspace setup files             |
| `om`   | `ros2 run open_manipulator_x_controller open_manipulator_x_controller` | Start the OpenManipulator X hardware controller        |
| `ho`   | `ros2 run omx_position_control basic_robot_control`                    | Run basic robot position control                       |
| `hoo`  | `ros2 run omx_position_control basic_robot_control2`                   | Run alternative basic robot position control           |
| `lau`  | `ros2 launch omx_position_control position_control.launch.py`          | Launch the position control ROS2 nodes                 |
| `cam`  | `python3 apture.py position`                                           | Run camera capture for position tracking               |
| `tp`   | `ros2 topic echo /target_pose`                                         | Display the target pose topic in real time             |
| `ap`   | `ros2 topic echo /actual_pose`                                         | Display the actual pose topic in real time             |
| `el`   | `ros2 launch omx_position_control evaluation.launch.py`                | Launch evaluation nodes for testing robot behavior     |
| `web`  | `ros2 run omx_position_control websocket_bridge_node`                  | Start the WebSocket bridge node for CV Control         |
| `web2` | `ros2 run omx_position_control websocket_bridge_node2`                 | Start the WebSocket bridge node for ARCore App Control |

Then, run the following commands in separate terminals:

| Terminal | Command                                                                 | Purpose                                                 |
| :------- | :---------------------------------------------------------------------- | :------------------------------------------------------ |
| 1        | `ros2 launch open_manipulator_x_controller controller.launch.py` or `om`| Start the OpenManipulator X hardware controller         |
| 2        | `ros2 run omx_position_control basic_robot_control` or `ho`             | Run basic robot position control                        |
|          | `ros2 launch omx_position_control position_control.launch.py` or `lau`  | Launch the position control ROS2 nodes                  |
| 3        | `ros2 run omx_position_control websocket_bridge_node` or `web`          | Start the WebSocket bridge node for CV Control          |
|          |`ros2 run omx_position_control websocket_bridge_node2` or `web2`         | Start the WebSocket bridge node for ARCore App Control  |

Feel free to open issues or submit pull requests for improvements\!
