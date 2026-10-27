#!/bin/bash
# Fait bouger le bras : empile les trois cubes en (0.6, 0).
# La simulation doit tourner : ros2 launch robotic_arm_sim sim.launch.py

source /opt/ros/jazzy/setup.bash
source /workspaces/robotic_arm_ws/install/setup.bash

# Amene le point de saisie (entre les doigts) en x y z, et attend la fin du mouvement
move() {
    echo "move $1 $2 $3"
    ros2 topic echo --once /arm_controller/trajectory_finished > /dev/null &
    sleep 1
    ros2 topic pub --once -w 1 /target_position robot_msgs/msg/TargetPosition "{x_target: $1, y_target: $2, z_target: $3}" > /dev/null
    wait
}

open() {
    ros2 service call /gripper/open std_srvs/srv/Trigger | grep message
}

close() {
    ros2 service call /gripper/close std_srvs/srv/Trigger | grep message
}

# Attrape le cube pose sur la table en x y
pick() {
    open
    move $1 $2 1.2
    move $1 $2 1.045
    close
    move $1 $2 1.3
}

# Pose le cube tenu en x y, a la hauteur z (centre du cube)
place() {
    move $1 $2 1.3
    move $1 $2 $3
    open
    move $1 $2 1.3
}

pick 0.8 -0.25          # cube à droite du bras
place 0.6 0 1.048       # sur la table

pick 0.8 0.25           # cube à gauche du bras
place 0.6 0 1.108       # sur le premier cube

pick 0.8 0              # cube bleu
place 0.6 0 1.168       # sur le deuxième cube

move 0.6 0 1.4          # position de repos
