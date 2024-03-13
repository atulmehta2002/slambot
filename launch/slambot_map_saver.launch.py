import os 
import subprocess
import time

command = "ros2 run nav2_map_server map_saver_cli -f room"  # Adapt arguments as needed

# # Set the target directory to the Desktop
desktop_path = os.path.expanduser("/home/atul/ros2_ws/src/slambot/maps")

# # Change the working directory to the Desktop
os.chdir(desktop_path)


while True:
    subprocess.run(command, shell=True)  # Execute the command
    time.sleep(15)  # Wait for 30 seconds