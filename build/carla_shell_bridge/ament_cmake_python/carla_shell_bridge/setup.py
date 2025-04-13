from setuptools import find_packages
from setuptools import setup

setup(
    name='carla_shell_bridge',
    version='0.0.0',
    packages=find_packages(
        include=('carla_shell_bridge', 'carla_shell_bridge.*')),
)
