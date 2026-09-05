from glob import glob
import os

from setuptools import find_packages, setup


package_name = "sanitation_active_cleaning"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/config",
            glob("config/*.json") + glob("config/*.yaml"),
        ),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Sanitation Vehicle Team",
    maintainer_email="team@example.com",
    description="URDF-independent active-cleaning environment and evaluation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "active_cleaning_demo = sanitation_active_cleaning.cli:main",
            "active_cleaning_train = sanitation_active_cleaning.train_cli:main",
            "formal_active_cleaning_train = sanitation_active_cleaning.formal_training:main",
            "formal_stage_a_active_cleaning_train = sanitation_active_cleaning.formal_stage_a_training:main",
            "formal_planning_preflight = sanitation_active_cleaning.formal_contract:main",
            "formal_observation_bridge = sanitation_active_cleaning.formal_observation_bridge:main",
            "formal_policy_planner = sanitation_active_cleaning.formal_policy_planner:main",
            "formal_cleaning_coordinator = sanitation_active_cleaning.formal_cleaning_coordinator:main",
            "formal_trajectory_executor = sanitation_active_cleaning.formal_trajectory_executor:main",
        ],
    },
)
