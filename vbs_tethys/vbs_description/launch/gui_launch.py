import os
os.environ['ROS_DISTRO'] = 'humble'
from simple_launch import SimpleLauncher

sl = SimpleLauncher()

def launch_setup():
    # Start the GUI node
    sl.node('vbs_description', 'ballast_gui.py', name='ballast_gui', output='screen')
    return sl.launch_description()

generate_launch_description = sl.launch_description(launch_setup)
