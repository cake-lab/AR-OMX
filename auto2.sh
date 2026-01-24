#!/bin/bash

echo "======================================="
echo " OpenManipulatorX Setup – Part 2 (auto2.sh)"
echo "======================================="

############################################
# Step 3: Update ~/.bashrc
############################################
echo ""
echo "Step 3: Updating ~/.bashrc..."

BASHRC="$HOME/.bashrc"

add_line() {
    grep -qxF "$1" "$BASHRC" || echo "$1" >> "$BASHRC"
}

add_line 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash'
add_line 'source ~/omx/install/setup.bash'
add_line "alias srcs='source install/setup.bash'"
add_line "alias sr='source venv/bin/activate'"
add_line "alias om='ros2 run open_manipulator_x_controller open_manipulator_x_controller'"
add_line "alias ho='ros2 run omx_position_control basic_robot_control'"
add_line "alias hoo='ros2 run omx_position_control basic_robot_control2'"
add_line "alias lau='ros2 launch omx_position_control position_control.launch.py'"
add_line "alias cam='python3 apture.py position'"
add_line "alias tp='ros2 topic echo /target_pose'"
add_line "alias ap='ros2 topic echo /actual_pose'"
add_line "alias el='ros2 launch omx_position_control evaluation.launch.py'"
add_line "alias web='ros2 run omx_position_control websocket_bridge_node'"
add_line "alias web2='ros2 run omx_position_control websocket_bridge_node2'"

echo " ~/.bashrc updated."

############################################
# Step 4: Create OMX workspace and build
############################################
echo ""
echo "Step 4: Creating and building OMX workspace..."

mkdir -p ~/omx
cd ~/omx || exit 1

echo ""
read -p "Enter the direct download link for src.zip: " SRC_LINK

#wget "$SRC_LINK" -O src.zip
#if [ $? -ne 0 ]; then
#    echo "Failed to download src.zip"
#    exit 1
#fi
read -p "Press ENTER once the download is complete..."

unzip -o src.zip
rm src.zip

echo ""
echo "Building workspace..."
source /opt/ros/humble/setup.bash
colcon build

if [ $? -ne 0 ]; then
    echo "colcon build failed"
    exit 1
fi

source install/setup.bash

echo ""
echo "======================================="
echo " OMX workspace setup complete!"
echo "======================================="
echo "Open a new terminal or run:"
echo "source ~/.bashrc"
