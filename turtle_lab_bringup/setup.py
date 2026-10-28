"""Install the turtle_lab_bringup package and shared assets."""
from glob import glob
from setuptools import find_packages, setup
setup(name='turtle_lab_bringup', version='1.0.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/turtle_lab_bringup']),
                  ('share/turtle_lab_bringup', ['package.xml']),
                  ('share/turtle_lab_bringup/launch', glob('launch/*.launch.py')),
                  ('share/turtle_lab_bringup/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Lab Team', maintainer_email='student@example.com',
      description='ROS 2 turtle lab', license='Apache-2.0',
      entry_points={'console_scripts': []})
