#!/bin/bash

session="tmux_colreg"
tmux new-session -d -s $session

window=0
tmux rename-window -t $session:$window 'sim_init'
tmux send-keys -t $session:$window 'source  ~/catkin_ws/devel/setup.bash' C-m
tmux send-keys -t $session:$window 'roslaunch my_robot_controller multi_d_dec_14.launch' C-m
sleep 5  # Introduce a 2-second delay


#window=1
#tmux new-window -t $session:$window -n 'visualise'
#tmux send-keys -t $session:$window 'source  ~/catkin_ws/devel/setup.bash' C-m
#tmux send-keys -t $session:$window 'rosrun my_robot_controller visualize.py' C-m
#sleep 0.5



window=2
tmux new-window -t $session:$window -n 'visualise_try'
tmux send-keys -t $session:$window 'source  ~/catkin_ws/devel/setup.bash' C-m
tmux send-keys -t $session:$window 'rosrun my_robot_controller try_visual.py' C-m
sleep 35


window=3
tmux new-window -t $session:$window -n 'bot_0'
tmux send-keys -t $session:$window 'source  ~/catkin_ws/devel/setup.bash' C-m
tmux send-keys -t $session:$window 'rosrun my_robot_controller 3sim_0bot_head.py' C-m
sleep 1
tmux send-keys -t $session:$window '10' C-m
sleep 0.1
tmux send-keys -t $session:$window '5' C-m
sleep 0.1



window=4
tmux new-window -t $session:$window -n 'bot_1'
tmux send-keys -t $session:$window 'source  ~/catkin_ws/devel/setup.bash' C-m
tmux send-keys -t $session:$window 'rosrun my_robot_controller 3sim_1bot_head.py' C-m
sleep 1
tmux send-keys -t $session:$window '1' C-m
sleep 0.1
tmux send-keys -t $session:$window '5' C-m
sleep 0.1



window=5
tmux new-window -t $session:$window -n 'bot_2'
tmux send-keys -t $session:$window 'source  ~/catkin_ws/devel/setup.bash' C-m
tmux send-keys -t $session:$window 'rosrun my_robot_controller 3sim_2bot_head.py' C-m
sleep 1
tmux send-keys -t $session:$window '4' C-m
sleep 0.1
tmux send-keys -t $session:$window '2' C-m
sleep 0.1



tmux attach-session -t $session

