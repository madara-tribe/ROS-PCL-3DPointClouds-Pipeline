# 1 topic
roslaunch topics chat.launch
rostopic echo /chatter

# 2 server
roscore
rosrun topics service_server.py
rosrun topics service_client.py

# 3 srv server
roscore
rosrun topics velocity_server.py
rosrun topics velocity_client.py 1.0 0.5

# 4
roslaunch topics cpp_twist.launch
