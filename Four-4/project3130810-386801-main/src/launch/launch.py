import os
import sys
import time

def launchsim():

    os.system('gnome-terminal -t "master" -e "bash /home/cyberdog_race2026_ws/src/launch/master.sh"')
    time.sleep(0.51)
    os.system('gnome-terminal -t "adjust_demo" -e "bash /home/cyberdog_race2026_ws/src/launch/adjust_node.sh"')
    time.sleep(0.1)
    os.system('gnome-terminal -t "fisheyes_adjust_demo" -e "bash /home/cyberdog_race2026_ws/src/launch/fisheyes_adjust_node.sh"')
    time.sleep(0.1)
    os.system('gnome-terminal -t "ground_node" -e "bash /home/cyberdog_race2026_ws/src/launch/ground_node.sh"')
    time.sleep(0.1)

if __name__=="__main__":
    launchsim()
