from glob import glob
from setuptools import find_packages, setup

setup(name='vision_car', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/vision_car']),
                  ('share/vision_car', ['package.xml']),
                  ('share/vision_car/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': [
          'camera_node = vision_car.camera_node:main',
          'yolo_detect_node = vision_car.yolo_detect_node:main',
          'control_node = vision_car.control_node:main']})
