import os
import time
import subprocess
import signal

# Set the paths to the ROS1 and ROS2 installations
ros1_path_setup_distro = "/opt/ros/noetic"
ros1_path_setup_ws = "/home/usr/data/catkin_ws/devel"


ros2_path_setup_distro = "/opt/ros/jazzy"
ros2_path_setup_ws = "/home/user/data/drones_ship_ws/install"
          


killgz_cmd = "killgz" 

# Define ROS1 commands
ros1_roscore_cmd = "roscore"
ros1_simulation_cmd = "roslaunch aslam_rosbot house.launch  "
ros1_karto_cmd = "roslaunch aslam_rosbot graph_dopt.launch    "
d_opti_plot = "rosrun aslam_rosbot d_opti_plot.py"




# Define ROS1 commands

ros2_aquabot_simulation_cmd = "ros2 launch aquabot_gz system_launch.py"
ros2_aquabot_ekf_cmd = "ros2 launch aquabot_ekf ekf_launch.py"

ros2_crazyflie_ekf_cmd = "ros2 launch crazyflie_ekf crazyflie_ekf.launch.py  "

ros2_crazyflie_control_cmd = "ros2 run crazyflie_control crazyflie_control_node"


ros2_crazyflie_yolo_cmd = "ros2 run crazyflie_yolo yolo_detector_node.py  "

ros2_crazyflie_yolo_stereo_cmd = "ros2 run crazyflie_yolo stereo_depth_node"





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
    

    launch_terminal(ros2_aquabot_simulation_cmd , "RVIZ simulation", ros1=False)
    print("RVIZ Starting ...")
    
    # Wait for user input to kill all processes
    print("")
    print("")
    time.sleep(10)   
    launch_terminal(ros2_aquabot_ekf_cmd , "aquabot_ekf_cmd ", ros1=False)
    print("ros2_aquabot_ekf_cmd Starting...")
    print("")
    print("")
    time.sleep(10)
    launch_terminal(ros2_crazyflie_ekf_cmd , "crazyflie_ekf_cmd", ros1=False)
    print("crazyflie_ekf Started ...")
    print("")
    print("")   

    time.sleep(10)
    launch_terminal(ros2_crazyflie_control_cmd  , "ros2_crazyflie_control_cmd", ros1=False)
    print("ros2_crazyflie_control Started ...")
    print("")
    print("")   
    #time.sleep(10)
    #launch_terminal(ros2_crazyflie_yolo_cmd , "crazyflie_yolo_cmd ", ros1=False)
    #print("crazyflie_yolo Started ...")
    #print("")
    #print("")   


    #time.sleep(10)
    #launch_terminal(ros2_crazyflie_yolo_stereo_cmd, "ros2_crazyflie_yolo_stereo_cmd", ros1=False)
    #print("crazyflie_yolo_stereo Started ...")
    #print("")
    #print("")   
        
        
        
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
