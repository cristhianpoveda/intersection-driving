#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
# stop signal finder

dt-exec roslaunch \
    stop_finder stop_finder_node.launch \
    veh:="$VEHICLE_NAME" \
    robot_type:="$ROBOT_TYPE" \
    robot_configuration:="$ROBOT_CONFIGURATION"

# object detection

#dt-exec roslaunch \
#    road_users_detection detection_node.launch \
#    veh:="$VEHICLE_NAME" \
#    robot_type:="$ROBOT_TYPE" \
#    robot_configuration:="$ROBOT_CONFIGURATION"

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
