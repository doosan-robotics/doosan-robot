#!/bin/bash

emulator_version="3.0.0"
emulator_image="doosanrobot/dsr_emulator:$emulator_version"
emulator_name="dsr_emulator"


pulling() {
    printf " $1"
    docker pull "$2" > /dev/null 2>&1
    while [ $? -ne 0 ]; do
        printf "\n Downloading..."
        sleep 1
        docker pull "$2" > /dev/null 2>&1
    done
    echo "\n -> Completed $1"
}


setup() {
    printf " $1"
    docker run -dit --net=host --name $emulator_name "$emulator_image" > /dev/null 2>&1 && docker stop $emulator_name > /dev/null 2>&1
    while [ $? -ne 0 ]; do
        sleep 1
        docker run -dit --net=host --name $emulator_name "$emulator_image" > /dev/null 2>&1 && docker stop $emulator_name > /dev/null 2>&1
    done
    echo "\n -> Completed $1"
}


if docker ps -a --format '{{.Names}}' | grep -q $emulator_name; then
    echo "$emulator_name is already installed"
else
    # Pull dsr_emulator image
    pulling "pull dsr_emulator version : $emulator_version" "$emulator_image"
    # Run dsr_emulator container in detached mode with host network
    setup "setup $emulator_name" "$emulator_image"
fi
