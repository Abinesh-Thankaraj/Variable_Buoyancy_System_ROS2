from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = False)
    
    sl.declare_arg('namespace', default_value='tethys')
    sl.declare_arg('jsp', False)
    
    namespace = sl.arg('namespace')
    
    with sl.group(ns=namespace):

        # xacro parsing + change moving joints to fixed if no Gazebo here
        xacro_args = {'namespace': namespace, 'simulation': sl.sim_time}
        sl.robot_state_publisher('vbs_description', 'tethys.xacro', xacro_args=xacro_args)

        with sl.group(if_arg='jsp'):
            sl.joint_state_publisher(True)
        
    return sl.launch_description()
