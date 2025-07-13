#!/usr/bin/env python3
import os
os.environ['DISPLAY'] = ':0'
os.environ['QT_X11_NO_MITSHM'] = '1'

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from PyQt5 import QtWidgets, QtCore

class BallastGUI(Node):
    def __init__(self):
        super().__init__('ballast_gui')
        
        # Initialize Qt application
        self.app = QtWidgets.QApplication.instance()
        if not self.app:
            self.app = QtWidgets.QApplication([' '])
        
        # Create main window
        self.win = QtWidgets.QWidget()
        self.win.setWindowTitle('Tethys Ballast Monitoring')
        self.layout = QtWidgets.QHBoxLayout()
        self.win.setLayout(self.layout)
        
        # Ballast parameters
        self.max_mass = 9.43  # kg
        self.density = 1025.0  # kg/m³
        
        # Create bars
        self.create_ballast_bars()
        
        # Subscribers
        self.create_subscription(Float64, '/tethys/cmd_ballast1', self.ballast1_cb, 10)
        self.create_subscription(Float64, '/tethys/cmd_ballast2', self.ballast2_cb, 10)
        self.create_subscription(Imu, '/tethys/mpu', self.imu_cb, 10)
        
        # Show window
        self.win.show()
        
    def create_ballast_bars(self):
        """Create all GUI elements"""
        # Ballast 1
        self.ballast1 = self.create_ballast_widget("Ballast 1")
        self.layout.addWidget(self.ballast1)
        
        # Ballast 2
        self.ballast2 = self.create_ballast_widget("Ballast 2")
        self.layout.addWidget(self.ballast2)
        
        # Depth
        self.depth_widget = self.create_depth_widget("Depth")
        self.layout.addWidget(self.depth_widget)
    
    def create_ballast_widget(self, title):
        """Create a ballast tank widget"""
        group = QtWidgets.QGroupBox(title)
        layout = QtWidgets.QVBoxLayout()
        
        # Progress bar
        progress = QtWidgets.QProgressBar()
        progress.setOrientation(QtCore.Qt.Vertical)
        progress.setRange(0, 100)
        progress.setValue(0)
        progress.setFixedSize(100, 200)
        
        # Labels
        mass_label = QtWidgets.QLabel("Mass: 0.0 kg")
        volume_label = QtWidgets.QLabel("Volume: 0.0 m³")
        
        layout.addWidget(mass_label)
        layout.addWidget(progress)
        layout.addWidget(volume_label)
        group.setLayout(layout)
        
        # Store references
        group.mass_label = mass_label
        group.volume_label = volume_label
        group.progress = progress
        
        return group
    
    def create_depth_widget(self, title):
        """Create depth display widget"""
        group = QtWidgets.QGroupBox(title)
        layout = QtWidgets.QVBoxLayout()
        
        progress = QtWidgets.QProgressBar()
        progress.setOrientation(QtCore.Qt.Vertical)
        progress.setRange(0, 100)
        progress.setValue(0)
        progress.setFixedSize(100, 200)
        progress.setInvertedAppearance(True)
        
        label = QtWidgets.QLabel("Depth: 0.0 m")
        
        layout.addWidget(label)
        layout.addWidget(progress)
        group.setLayout(layout)
        
        group.label = label
        group.progress = progress
        
        return group
    
    def ballast1_cb(self, msg):
        self.update_ballast(self.ballast1, msg.data)
    
    def ballast2_cb(self, msg):
        self.update_ballast(self.ballast2, msg.data)
    
    def update_ballast(self, widget, cmd_value):
        """Update ballast display"""
        force = cmd_value**2 * 0.925
        mass = min(force / 9.81, self.max_mass)
        volume = mass / self.density
        
        widget.mass_label.setText(f"Mass: {mass:.2f} kg")
        widget.volume_label.setText(f"Volume: {volume:.5f} m³")
        widget.progress.setValue(int((mass / self.max_mass) * 100))
        
        self.app.processEvents()
    
    def imu_cb(self, msg):
        """Update depth display"""
        depth = min(abs(msg.linear_acceleration.z), 20.0)  # Max 20m for display
        self.depth_widget.label.setText(f"Depth: {depth:.2f} m")
        self.depth_widget.progress.setValue(int((depth / 20.0) * 100))
        self.app.processEvents()

def main(args=None):
    rclpy.init(args=args)
    node = BallastGUI()
    
    # Start Qt event loop
    timer = QtCore.QTimer()
    timer.start(100)  # Update every 100ms
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
