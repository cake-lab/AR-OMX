#!/bin/bash

echo "======================================="
echo " OpenManipulatorX Setup – Part 1 (auto.sh)"
echo "======================================="

############################################
# Step 1: Check ROS 2 Humble installation
############################################
echo ""
echo "Step 1: Checking ROS 2 installation..."

# Check if ROS directory exists
if [ -d "/opt/ros/humble" ]; then
    echo "ROS 2 Humble detected at /opt/ros/humble"
else
    echo "ROS 2 Humble not found!"
    echo "Please install ROS 2 Humble Hawksbill from:"
    echo "https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

# Check if ros2 executable exists
if command -v ros2 >/dev/null 2>&1; then
    echo "ros2 command detected"
else
    echo "ros2 command not available"
    echo "Make sure ROS 2 Humble is installed and sourced"
    exit 1
fi

############################################
# Step 2: Dynamixel Wizard 2 installation
############################################
echo ""
echo "Step 2: Dynamixel Wizard 2 Installation"
echo "---------------------------------------"
echo "Download from:"
echo "https://www.robotis.com/service/download.php?no=1671"
echo ""
echo "Expected file:"
echo "DynamixelWizard2Setup-linux-x64.run"
read -p "Press ENTER once download is complete..."

cd ~/Downloads || exit 1

FILE="DynamixelWizard2Setup-linux-x64.run"

if [ ! -f "$FILE" ]; then
    echo "$FILE not found in ~/Downloads"
    ls -lh ~/Downloads
    exit 1
fi

echo "Setting executable permission..."
sudo chmod +x "$FILE"

echo "Launching installer..."
./"$FILE"

USER_ID=$(whoami)
echo ""
echo "Adding user '$USER_ID' to dialout group..."
sudo usermod -aG dialout "$USER_ID"

echo ""
echo "======================================="
echo " Installation complete – REBOOT REQUIRED"
echo "======================================="
read -p "Press ENTER to reboot now (Ctrl+C to cancel)..."
sudo reboot

