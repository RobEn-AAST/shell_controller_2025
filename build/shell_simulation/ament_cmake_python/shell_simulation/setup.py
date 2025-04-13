from setuptools import find_packages
from setuptools import setup

setup(
    name='shell_simulation',
    version='1.0.0',
    packages=find_packages(
        include=('shell_simulation', 'shell_simulation.*')),
)
