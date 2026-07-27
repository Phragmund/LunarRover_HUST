from glob import glob
from setuptools import find_packages, setup

package_name = "lunar_rover_bringup"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lunar Rover Maintainers",
    maintainer_email="maintainer@example.com",
    description="Minimal lunar rover bringup without SLAM or navigation.",
    license="Apache-2.0",
)
