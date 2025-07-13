import os
os.environ['ROS_DISTRO'] = 'humble'
from simple_launch import SimpleLauncher, GazeboBridge

sl = SimpleLauncher(use_sim_time=True)

sl.declare_arg('namespace', default_value='tethys')
sl.declare_arg('ground_truth', default_value=True)
sl.declare_arg('sliders', default_value=False)
sl.declare_arg('camera', True)
sl.declare_arg('gazebo_world_name', 'none')

# initial pose
sl.declare_gazebo_axes(x=0, y=0., z=0., roll=0., pitch=0., yaw=0.)

def launch_setup():
    ns = sl.arg('namespace')

    if sl.arg('gazebo_world_name') != 'none':
        GazeboBridge.set_world_name(sl.arg('gazebo_world_name'))
    
    # robot state publisher
    sl.include('vbs_description', 'state_publisher_launch.py',
               launch_arguments={'namespace': ns, 'use_sim_time': sl.sim_time})
               
    with sl.group(ns=ns):
        # URDF spawner to Gazebo
        sl.spawn_gz_model(ns, spawn_args=sl.gazebo_axes_args())
            
        # ROS-Gz bridges
        bridges = []
        gz_js_topic = GazeboBridge.model_prefix(ns) + '/joint_state'
        bridges.append(GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros))
        
        # pose ground truth
        bridges.append(GazeboBridge(f'/model/{ns}/pose',
                                 'pose_gt', 'geometry_msgs/Pose', GazeboBridge.gz2ros))
        
        # Barometer/depth sensor bridge
        bridges.append(GazeboBridge(f'/{ns}/baro',
                                 'baro', 'ros_gz_interfaces/msg/Altimeter', GazeboBridge.gz2ros))
        
        # ground truth if requested
        if sl.arg('ground_truth'):
            bridges.append(GazeboBridge(f'/model/{ns}/odometry',
                                     'odom', 'nav_msgs/Odometry', GazeboBridge.gz2ros,
                                     'gz.msgs.Odometry'))
            sl.node('pose_to_tf', parameters={'child_frame': ns + '/base_link'})
        else:
            sl.node('pose_to_tf', parameters={'child_frame': ns+'/base_link_gt'})

        # IMU sensors
        for imu in ('mpu', 'lsm'):
            bridges.append(GazeboBridge(f'{ns}/{imu}',
                          imu, 'sensor_msgs/Imu', GazeboBridge.gz2ros))

        # VBS control
        for thr in ('1', '2'):
            ballast = f'ballast{thr}'
            gz_thr_topic = f'/{ns}/{ballast}/cmd'
            bridges.append(GazeboBridge(gz_thr_topic, f'cmd_{ballast}', 'std_msgs/Float64', GazeboBridge.ros2gz))

        sl.create_gz_bridge(bridges)

        if sl.arg('sliders'):
            sl.node('slider_publisher', arguments=[sl.find('vbs_description', 'manual.yaml')])
    
    return sl.launch_description()

generate_launch_description = sl.launch_description(launch_setup)
