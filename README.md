🚀 Launching the Simulation
1. Launch the Canyon World

Spawns the Stingray AUV in an underwater canyon environment:

ros2 launch ray_description canyon_world_launch.py

2. Launch the Nonlinear Control System

Starts the Sliding Mode Controller (SMC) and visualizes wrenches and forces in RViz:

ros2 launch ray_control custom_sliding custom_sliding.launch.py

3. Launch YOLOv8-Based Person Detection

Activates underwater human detection using YOLOv8:

ros2 launch yolobot_recognition launch.py

📊 Results and Visualization

RMS position error: < 0.1 m

Orientation error: < 0.05 rad

Reduced control effort compared to traditional PID systems

Real-time perception and control demonstrated in Ignition Gazebo and RViz2 environments

📚 Future Work

Integration with real-time underwater camera hardware

Adaptive control under dynamic current profiles

Multi-AUV cooperative exploration and rescue planning
