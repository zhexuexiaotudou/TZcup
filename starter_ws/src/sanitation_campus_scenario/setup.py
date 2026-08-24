from glob import glob
import os

from setuptools import find_packages, setup

package_name = "sanitation_campus_scenario"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Sanitation Vehicle Team",
    maintainer_email="team@example.com",
    description="Deterministic URDF-independent campus world and episode generation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sanitation-campus-scenario = sanitation_campus_scenario.cli:main",
            "sanitation-campus-pedestrian-driver = sanitation_campus_scenario.pedestrian_driver:main",
        ]
    },
)
