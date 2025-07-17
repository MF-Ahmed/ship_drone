from setuptools import find_packages
from setuptools import setup

setup(
    name='crazyflie_yolo',
    version='0.0.1',
    packages=find_packages(
        include=('crazyflie_yolo', 'crazyflie_yolo.*')),
)
