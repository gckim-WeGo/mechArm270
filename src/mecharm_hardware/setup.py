from setuptools import setup, find_packages
import os
from glob import glob

package_name = "mecharm_hardware"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your@email.com",
    description="Hardware driver for mechArm 270 M5 using pymycobot",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mecharm_driver = mecharm_hardware.mecharm_driver:main",
            "mecharm_example = mecharm_hardware.mecharm_moveit_example:main",
            "pick_and_place = mecharm_hardware.pick_and_place_node:main",
            "apriltag_tf = mecharm_hardware.apriltag_tf_node:main",
        ],
    },
)
