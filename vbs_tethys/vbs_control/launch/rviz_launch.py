from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = False)
    
    sl.declare_arg('namespace', default_value='tethys')
    
    sl.rviz(sl.find('vbs_control', 'tethys.rviz'))

    with sl.group(ns=sl.arg('namespace')):
        sl.node('thruster_manager', 'publish_wrenches',
                parameters={'control_frame': sl.arg('namespace') / 'base_link',
                            'use_gz_topics': sl.sim_time})
    
    return sl.launch_description()
