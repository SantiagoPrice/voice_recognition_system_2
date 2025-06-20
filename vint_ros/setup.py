import os
from glob import glob
from setuptools import setup

package_name = 'vint_ros'

model_files = [
    (os.path.join('share', package_name, 'vosk-model'),
    glob('vint_ros/vosk-model/**/*', recursive=True))
]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + model_files,
    install_requires=['setuptools','vosk','sounddevice'],
    zip_safe=True,
    maintainer='orin',
    maintainer_email='makino@ai.iit.tsukuba.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy = vint_ros.dummy:main',
            'spin_node = vint_ros.spin_node:main',
            'voice_assist_ROS_woGUI = vint_ros.voice_assist_ROS_woGUI:main'

        ],
    },
)
