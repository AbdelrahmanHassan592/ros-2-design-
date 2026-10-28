"""Install the turtle_lab_control package and shared assets."""
from glob import glob
from setuptools import find_packages, setup
setup(name='turtle_lab_control', version='1.0.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/turtle_lab_control']),
                  ('share/turtle_lab_control', ['package.xml']),
                  ('share/turtle_lab_control/launch', glob('launch/*.launch.py')),
                  ('share/turtle_lab_control/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Lab Team', maintainer_email='student@example.com',
      description='ROS 2 turtle lab', license='Apache-2.0',
      entry_points={'console_scripts': ['bag_demo = turtle_lab_control.bag_demo:main', 'go_to_pose_server = turtle_lab_control.go_to_pose_server:main', 'reset_service = turtle_lab_control.reset_service:main', 'square_action_server = turtle_lab_control.square_action_server:main']})
