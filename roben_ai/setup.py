from setuptools import find_packages, setup
import os
from glob import glob

package_name = "roben_ai"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=[
        "setuptools",
        "numpy==1.24.4",
        "torch",
        "torchvision",
        "torchaudio",
        "opencv-python",
        "gymnasium",
        "stable-baselines3",
        "matplotlib",
        "optuna",
        "scikit-image",
        "carla==0.9.13"
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
