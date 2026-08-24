from glob import glob

from setuptools import find_packages, setup


package_name = "sanitation_active_cleaning"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.json")),
    ],
    install_requires=["setuptools"],
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
        ],
    },
)
