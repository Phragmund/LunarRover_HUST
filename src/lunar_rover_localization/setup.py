from glob import glob
from setuptools import find_packages, setup

package_name = "lunar_rover_localization"

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
    description="Localization and online mapping for the lunar rover.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lio_odom_adapter = lunar_rover_localization.lio_odom_adapter:main",
            "pose_tf_splitter = lunar_rover_localization.pose_tf_splitter:main",
        ],
    },
)

