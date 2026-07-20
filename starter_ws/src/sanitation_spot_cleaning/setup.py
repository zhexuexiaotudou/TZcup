from glob import glob
import os

from setuptools import find_packages, setup


package_name = "sanitation_spot_cleaning"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Sanitation Vehicle Team",
    maintainer_email="team@example.com",
    description="Deferred spot-cleaning coordinator and evaluator.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "spot_cleaning_node = sanitation_spot_cleaning.node:main",
            "stage5a_spot_clean_evaluator = sanitation_spot_cleaning.evaluator:main",
            "stage5br5_observation_pose_node = sanitation_spot_cleaning.observation_pose_node:main",
        ],
    },
)
