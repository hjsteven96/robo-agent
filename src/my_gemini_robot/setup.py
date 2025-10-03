import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_gemini_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 파일 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Config 파일 설치
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robo',
    maintainer_email='robo@todo.todo',
    description='Gemini agent-driven robot with OTA loop (observe-think-act)',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'gemini_planner = my_gemini_robot.gemini_bridge:main_planner',
            'gemini_vision  = my_gemini_robot.gemini_bridge:main_vision',
            'gemini_agent   = my_gemini_robot.gemini_bridge:main_agent',
        ],
    },
)
