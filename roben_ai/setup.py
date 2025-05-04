from setuptools import find_packages, setup
import os
from glob import glob

package_name = "roben_ai"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]) + ['ai_src', 'ai_src.gym_carla', 'ai_src.gym_carla.envs', 
                                                'ai_src.models', 'ai_src.utils', 'ai_src.training', 'config'],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include AI models and configuration files
        (os.path.join('share', package_name, 'ai_src'), glob('ai_src/*.py')),
        (os.path.join('share', package_name, 'ai_src/gym_carla'), glob('ai_src/gym_carla/*.py')),
        (os.path.join('share', package_name, 'ai_src/gym_carla/envs'), glob('ai_src/gym_carla/envs/*.py')),
        (os.path.join('share', package_name, 'ai_src/models'), glob('ai_src/models/*.py')),
        (os.path.join('share', package_name, 'ai_src/utils'), glob('ai_src/utils/*.py')),
        (os.path.join('share', package_name, 'ai_src/training'), glob('ai_src/training/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.py')),
    ],
    install_requires=[
        "setuptools",
        "numpy==1.24.3",
        "opencv-python",
        "gymnasium",
        "matplotlib",
        "scikit-image",
        "typing_extensions==4.5.0",
    ],
    zip_safe=True,
    maintainer="zeyadcode_jammy",
    maintainer_email="zeyadcode_jammy@todo.todo",
    description="Autonomous vehicle control using reinforcement learning",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["brain = roben_ai.brain:main"],
    },
)
