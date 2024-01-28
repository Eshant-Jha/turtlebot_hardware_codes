#!/bin/bash

session="tmux_apriltag"

tmux new-session -d -s $session

window=0
tmux rename-window -t $session:$window 'camera_init'
tmux send-keys -t $session:$window 'source ~/uicam_ws/devel/setup.bash' C-m
tmux send-keys -t $session:$window 'roslaunch ueye_cam rgb8.launch' C-m
sleep 2  # Introduce a 2-second delay

window=1
tmux new-window -t $session:$window -n 'rect_img'
tmux send-keys -t $session:$window 'ROS_NAMESPACE=camera rosrun image_proc image_proc' C-m
sleep 1  # Introduce a 1-second delay

window=2
tmux new-window -t $session:$window -n 'april_detection'
tmux send-keys -t $session:$window 'roslaunch apriltag_ros continuous_detection.launch' C-m
sleep 2  # Introduce a 1-second delay

window=3
tmux new-window -t $session:$window -n 'coordinate_broadcast'
tmux send-keys -t $session:$window 'rosrun apriltag_ros tf_broadcaster' C-m
sleep 1  # Introduce a 1-second delay


tmux attach-session -t $session

