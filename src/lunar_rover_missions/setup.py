from glob import glob
from setuptools import find_packages, setup

package_name = "lunar_rover_missions"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lunar Rover Maintainers",
    maintainer_email="maintainer@example.com",
    description="Waypoint missions and metrics.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mission_runner = lunar_rover_missions.mission_runner:main",
            "metrics_recorder = lunar_rover_missions.metrics_recorder:main",
            "step_climb_mission = lunar_rover_missions.step_climb_mission:main",
            "maze_mapping_mission = lunar_rover_missions.maze_mapping_mission:main",
            "shuttle_mapping_mission = lunar_rover_missions.shuttle_mapping_mission:main",
        ],
    },
)
