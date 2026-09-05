from setuptools import find_packages, setup


package_name = "sanitation_localization"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", ["config/formal_fusion.yaml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/formal_localization_fusion.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sanitation Vehicle Team",
    maintainer_email="team@example.com",
    description=(
        "Wheel, IMU, GNSS and lidar-map localization fusion for the formal "
        "sanitation vehicle."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "validate_formal_localization_runtime = sanitation_localization.formal_runtime_validator:main",
        ],
    },
)
