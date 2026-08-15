import os
import time


def launchsim():
    os.system('gnome-terminal -t "ground_node" -e "bash /home/cyberdog_race2026_ws/src/launch/ground_node.sh"')
    time.sleep(0.5)
    os.system('gnome-terminal -t "test3_pid_live" -e "bash /home/cyberdog_race2026_ws/src/launch/test3.sh"')


if __name__ == "__main__":
    launchsim()
