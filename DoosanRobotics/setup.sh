#!/bin/bash
echo "###############################################################################"
echo "Workshop environment setup starting.."
echo "###############################################################################"

PROJEC=DoosanRobotics

cd ~/
if [ -e environment/roboMakerSettings.json ]; then
    echo "Ok."
else
    echo "Error!"
    echo "  It seems you are not in RoboMaker developer environment now.."
    echo "  Please execute this on RoboMaker developer environment."
    echo ""
    exit 1
fi

if [ -e environment/${PROJEC}/roboMakerSettings.json ]; then
    echo "Ok."
else
    echo "Error!"
    echo "  It seems you extracted the file in the wrong place.."
    echo "  Please extract this material under the root folder of RoboMaker development environment."
    echo ""
    exit 1
fi

cd environment
sudo apt-get update
source /opt/ros/$ROS_DISTRO/setup.sh
rosdep update

sudo apt-get install -y python3-apt python3-pip
sudo pip3 install -U setuptools
pip3 install -U colcon-common-extensions colcon-ros-bundle
pip install boto3

cd ${PROJEC}
python ./setup.py
