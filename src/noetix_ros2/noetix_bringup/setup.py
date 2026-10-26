from glob import glob
import os

from setuptools import find_packages, setup


package_name = "noetix_bringup"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join(
                "share", package_name, "launch"
            ),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=[
        "setuptools",
        "PyYAML",
    ],

    zip_safe=True,
    maintainer="Noetix Robotics",
    maintainer_email="robotics@noetix.ai",
    description=(
        "Real-hardware bringup for Noetix robots."
    ),
    license="Apache-2.0",
)
