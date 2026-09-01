from glob import glob
import os

from setuptools import find_packages, setup


package_name = "sanitation_manipulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Sanitation Vehicle Team",
    maintainer_email="team@example.com",
    description="Truth-free grasp planning, formal physical execution, bin verification, and recovery.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "active_cleaning_grasp_adapter = sanitation_manipulation.active_cleaning_adapter:main",
            "formal_physical_grasp_executor = sanitation_manipulation.formal_grasp_executor:main",
            "placeholder_cube_demo = sanitation_manipulation.placeholder_demo:main",
        ],
    },
)
