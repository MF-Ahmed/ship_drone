import os
from launch.actions import ExecuteProcess
import time
import subprocess
import signal

# Set the paths to the ROS1 and ROS2 installations
ros1_path_setup_distro = "/opt/ros/noetic"
ros1_path_setup_ws = "/home/usr/data/catkin_ws/devel"


ros2_path_setup_distro = "/opt/ros/jazzy"
ros2_path_setup_ws = "/home/user/data/drones_ship_ws/install"



killgz_cmd = "/home/user/data/drones_ship_ws/bin/killgz_services" 
        


# Define ROS1 commands

ros2_aquabot_simulation_launch = "ros2 launch aquabot_gz full_system_launch.py gui:=False"

ros2_aquabot_ekf_launch = "ros2 launch aquabot_ekf ekf_launch.py"

ros2_crazyflie_ekf_launch = "ros2 launch crazyflie_ekf all_drones_ekf_new.launch.py drones:=drone1,drone2,drone3"



# Function to run each terminal
def launch_terminal(cmd, title=None, ros1=True):
    title_option = f'--title="{title}"' if title else ''
    if ros1:
        setup_distro_path = ros1_path_setup_distro
        setup_ws_path = ros1_path_setup_ws
    else:
        setup_distro_path = ros2_path_setup_distro
        setup_ws_path = ros2_path_setup_ws

    os.system(
        f"gnome-terminal {title_option} --window -- zsh -c 'source {setup_distro_path}/setup.zsh; source {setup_ws_path}/setup.zsh; {cmd}; exec zsh'")


def main():    

    launch_terminal(ros2_aquabot_simulation_launch, "world simulation start", ros1=False)
    print("Simulation world starting  ...")
    
    # Wait for user input to kill all processes
    print("")
    print("")
    time.sleep(5)   
    launch_terminal(ros2_aquabot_ekf_launch, "aquabot_ekf_starting", ros1=False)
    print("Aquabot_ekf_starting ...")
    print("")
    print("")
    time.sleep(5)
    launch_terminal(ros2_crazyflie_ekf_launch, "crazyflie_ekf", ros1=False)
    print("Crazyflie_ekf_launch ...")
    print("")
    print("")

        
        
    # Wait for user input to kill all processes
    input("Kill all processes [Enter]: ")

    time.sleep(5)
    launch_terminal(killgz_cmd,  "killgz_cmd", ros1=False)
    print("killgz_cmd  ...")
    print("")
    print("")    
    time.sleep(5)   

    # Close all terminals
    # Kill all processes containing "gnome-terminal" in the name
    os.system("pkill -f gnome-terminal")

if __name__ == "__main__":
    main()
