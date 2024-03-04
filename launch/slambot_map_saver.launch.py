import subprocess
import time

command = "ros2 run nav2_map_server map_saver_cli -f room"  # Adapt arguments as needed
# command = "pwd"

while True:
    subprocess.run(command, shell=True)  # Execute the command
    time.sleep(15)  # Wait for 30 seconds