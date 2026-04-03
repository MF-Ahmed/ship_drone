import os
import time
import subprocess
import signal
from datetime import datetime



ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

def with_ts(path: str, ts: str) -> str:
    base, ext = os.path.splitext(path)
    return f"{base}_{ts}{ext}"



# Set the paths to the ROS1 and ROS2 installations
ros1_path_setup_distro = "/opt/ros/noetic"
ros1_path_setup_ws = "/home/usr/data/catkin_ws/devel"


ros2_path_setup_distro = "/opt/ros/jazzy"
ros2_path_setup_ws = "/home/user/data/drones_ship_ws/install"
          
# Define ROS2 commands

ros2_run_servers_cmd = "ros2 launch crazyflie_servers all_drone_explore_track_servers_new.launch.py drones:=drone1,drone2,drone3"
ros2_crazyflie_yolo_launch = "ros2 launch crazyflie_yolo full_system_yolo_stereo_track.launch.py"



fusion_csv = with_ts("/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs_fusion/1st/Path1_fused_obstacles.csv", ts)
assign_csv = with_ts("/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs_assignment/1st/Path1_assignment_log.csv", ts)

event_log_path = with_ts("/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs_fusion/1st/Path1_fusion_events.csv", ts)
prune_log_path = with_ts("/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs_fusion/1st/Path1_prune_events.csv", ts)
stats_log_path = with_ts("/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs_fusion/1st/Path1_fusion_stats.csv", ts)





ros2_crazyflie_yolo_fusion_launch = (
    "ros2 launch crazyflie_yolo full_system_fusion_hungarian.launch.py "
    f"fusion_log_path:={fusion_csv} "
    f"event_log_path:={event_log_path} "
    f"prune_log_path:={prune_log_path} "
    f"stats_log_path:={stats_log_path} "
    f"assignment_csv_path:={assign_csv}"
)

ros2_move_drone1_cmd = "ros2 run crazyflie_servers dual_action_client --ros-args \
  -p ns:=drone1 -p target_z:=10.0 -p hover_sec:=2 -p yaw:=0.0 \
  -p distance_x:=8.0 -p distance_y:=-8.0 -p speed:=0.20"
ros2_move_drone2_cmd = "ros2 run crazyflie_servers dual_action_client --ros-args \
  -p ns:=drone2 -p target_z:=10.0 -p hover_sec:=2 -p yaw:=0.0 \
  -p distance_x:=8.0 -p distance_y:=0.0 -p speed:=0.20"
ros2_move_drone3_cmd = "ros2 run crazyflie_servers dual_action_client --ros-args \
  -p ns:=drone3 -p target_z:=10.0 -p hover_sec:=2 -p yaw:=0.0 \
  -p distance_x:=8.0 -p distance_y:=8.0 -p speed:=0.20"



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

    launch_terminal(ros2_run_servers_cmd , "Servers_Starting", ros1=False)
    print("Servers Starting for all drones ...")
    
    # Wait for user input to kill all processes
    print("")
    print("")
    time.sleep(10)   
    launch_terminal(ros2_move_drone1_cmd , "ros2_move_drone1_cmd ", ros1=False)
    print("ros2_move_drone1_cmd...")
    print("")
    print("")
    time.sleep(10)
    launch_terminal(ros2_move_drone2_cmd , "ros2_move_drone2_cmd ", ros1=False)
    print("ros2_move_drone2_cmd...")
    print("")
    print("")

    time.sleep(10)
    launch_terminal(ros2_move_drone3_cmd , "ros2_move_drone3_cmd ", ros1=False)
    print("ros2_move_drone3_cmd...")
    print("")
    print("")


    time.sleep(50)
    time.sleep(25)
    launch_terminal(ros2_crazyflie_yolo_launch , "crazyflie_yolo", ros1=False)
    print("crazyflie_yolo Started ...")
    print("")
    print("")


    time.sleep(5)
    launch_terminal(ros2_crazyflie_yolo_fusion_launch, "Yolo_fusion_launch", ros1=False)
    print("Yolo_fusion Started ...")
    print("")
    print("")
        
        
    time.sleep(5)  
    # Wait for user input to kill all processes
    #input("Kill all processes [Enter]: ")

    #time.sleep(5)
  
    print("")
    print("")    
  
    # Close all terminals
    # Kill all processes containing "gnome-terminal" in the name
    #os.system("pkill -f gnome-terminal")

if __name__ == "__main__":
    main()
