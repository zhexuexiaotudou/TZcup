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
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Sanitation Vehicle Team",
    maintainer_email="team@example.com",
    description="Safe grasp planning, bin state, and recovery for AUTO-09.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
