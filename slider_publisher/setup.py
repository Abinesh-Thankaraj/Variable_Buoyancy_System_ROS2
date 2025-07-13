from setuptools import setup

package_name = 'slider_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abinesh',
    maintainer_email='abinesh@example.com',
    description='A slider publisher node for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slider_publisher = slider_publisher.slider_publisher:main',
        ],
    },
)

