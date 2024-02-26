#!/bin/bash

emulator_name="dsr_emulator"
emulator_image="doosanrobot/dsr_emulator:3.0.0"

# Function to stop and remove container
stop_and_remove_container() {
    printf " Starting to stop and remove $1"
    docker stop "$1" > /dev/null 2>&1
    docker rm "$1" > /dev/null 2>&1
}

# Function to remove image
remove_image() {
    docker rmi "$1" > /dev/null 2>&1
    echo "\n Completed $1."
}

# Stop and remove container
stop_and_remove_container "$emulator_name"

# Remove image
remove_image "$emulator_image" "$emulator_name"