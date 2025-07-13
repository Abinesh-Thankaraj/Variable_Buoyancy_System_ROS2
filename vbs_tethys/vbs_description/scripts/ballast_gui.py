#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import math
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

class BallastGUI(Node):
    def __init__(self):
        super().__init__('ballast_gui')
        # Ballast parameters
        self.density = 1025.0  # kg/m³
        self.gravity = 9.81    # m/s²
        self.max_mass = 9.43   # kg
        self.max_volume = 0.009203  # m³
        
        # Create GUI
        self.app = QtWidgets.QApplication([])
        self.win = QtWidgets.QWidget()
        self.win.setWindowTitle('Tethys Ballast Monitoring')
        self.layout = QtWidgets.QGridLayout()
        self.win.setLayout(self.layout)
        
        # Create ballast bars
        self.ballast1_bar = self.create_ballast_bar("Ballast Tank 1")
        self.ballast2_bar = self.create_ballast_bar("Ballast Tank 2")
        self.depth_bar = self.create_depth_bar("Depth (m)")
        
        # Add to layout
        self.layout.addWidget(self.ballast1_bar['group'], 0, 0)
        self.layout.addWidget(self.ballast2_bar['group'], 0, 1)
        self.layout.addWidget(self.depth_bar['group'], 0, 2)
        
        # Subscribers
        self.ballast1_sub = self.create_subscription(
            Float64, '/tethys/cmd_ballast1', self.ballast1_cb, 10)
        self.ballast2_sub = self.create_subscription(
            Float64, '/tethys/cmd_ballast2', self.ballast2_cb, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/tethys/mpu', self.imu_cb, 10)
        
        # Initialize values
        self.ballast1_mass = 0.0
        self.ballast2_mass = 0.0
        self.depth = 0.0
        
        # Start GUI update timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)  # Update every 100ms
        
        self.win.show()
        
    def create_ballast_bar(self, title):
        group = QtWidgets.QGroupBox(title)
        layout = QtWidgets.QVBoxLayout()
        
        # Mass label
        mass_label = QtWidgets.QLabel("Mass: 0.0 kg")
        mass_label.setAlignment(QtCore.Qt.AlignCenter)
        
        # Volume label
        volume_label = QtWidgets.QLabel("Volume: 0.0 m³")
        volume_label.setAlignment(QtCore.Qt.AlignCenter)
        
        # Progress bar
        progress_bar = QtWidgets.QProgressBar()
        progress_bar.setOrientation(QtCore.Qt.Vertical)
        progress_bar.setRange(0, 100)
        progress_bar.setValue(0)
        progress_bar.setFixedWidth(100)
        progress_bar.setFixedHeight(300)
        
        layout.addWidget(mass_label)
        layout.addWidget(progress_bar)
        layout.addWidget(volume_label)
        group.setLayout(layout)
        
        return {
            'group': group,
            'mass_label': mass_label,
            'volume_label': volume_label,
            'progress_bar': progress_bar
        }
    
    def create_depth_bar(self, title):
        group = QtWidgets.QGroupBox(title)
        layout = QtWidgets.QVBoxLayout()
        
        # Depth label
        depth_label = QtWidgets.QLabel("Depth: 0.0 m")
        depth_label.setAlignment(QtCore.Qt.AlignCenter)
        
        # Progress bar
        progress_bar = QtWidgets.QProgressBar()
        progress_bar.setOrientation(QtCore.Qt.Vertical)
        progress_bar.setRange(0, 100)
        progress_bar.setValue(0)
        progress_bar.setFixedWidth(100)
        progress_bar.setFixedHeight(300)
        progress_bar.setInvertedAppearance(True)  # 0 at top, max at bottom
        
        layout.addWidget(depth_label)
        layout.addWidget(progress_bar)
        group.setLayout(layout)
        
        return {
            'group': group,
            'depth_label': depth_label,
            'progress_bar': progress_bar
        }
    
    def ballast1_cb(self, msg):
        # Calculate mass from thruster command
        force = msg.data**2 * 0.925
        self.ballast1_mass = force / self.gravity
    
    def ballast2_cb(self, msg):
        # Calculate mass from thruster command
        force = msg.data**2 * 0.925
        self.ballast2_mass = force / self.gravity
    
    def imu_cb(self, msg):
        # Depth from IMU z position (linear acceleration integrated twice)
        # For simplicity, we'll use linear z acceleration as depth proxy
        # In a real system, you'd integrate this properly
        self.depth = abs(msg.linear_acceleration.z)
    
    def update_gui(self):
        # Update ballast 1
        mass1 = min(self.ballast1_mass, self.max_mass)
        volume1 = mass1 / self.density
        percent1 = int((mass1 / self.max_mass) * 100)
        
        self.ballast1_bar['mass_label'].setText(f"Mass: {mass1:.2f} kg")
        self.ballast1_bar['volume_label'].setText(f"Volume: {volume1:.5f} m³")
        self.ballast1_bar['progress_bar'].setValue(percent1)
        
        # Update ballast 2
        mass2 = min(self.ballast2_mass, self.max_mass)
        volume2 = mass2 / self.density
        percent2 = int((mass2 / self.max_mass) * 100)
        
        self.ballast2_bar['mass_label'].setText(f"Mass: {mass2:.2f} kg")
        self.ballast2_bar['volume_label'].setText(f"Volume: {volume2:.5f} m³")
        self.ballast2_bar['progress_bar'].setValue(percent2)
        
        # Update depth (assume max depth 20m for visualization)
        depth = min(self.depth, 20.0)
        depth_percent = int((depth / 20.0) * 100)
        
        self.depth_bar['depth_label'].setText(f"Depth: {depth:.2f} m")
        self.depth_bar['progress_bar'].setValue(depth_percent)
        
        # Process GUI events
        self.app.processEvents()

def main(args=None):
    rclpy.init(args=args)
    node = BallastGUI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
