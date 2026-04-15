import os
from glob import glob
from setuptools import setup

package_name = 'vint_ros'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools','vosk','sounddevice','google-genai'],
    zip_safe=True,
    maintainer='orin',
    maintainer_email='makino@ai.iit.tsukuba.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
)
