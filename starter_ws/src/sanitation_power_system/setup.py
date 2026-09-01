from setuptools import find_packages, setup


package_name = "sanitation_power_system"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", ["config/a300_40ah_bms.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sanitation Vehicle Team",
    maintainer_email="team@example.com",
    description="A300 40 Ah battery and BMS simulation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "a300_bms_simulator = sanitation_power_system.a300_bms_node:main",
            "charge_interface_manager = sanitation_power_system.charge_interface_manager:main",
        ],
    },
)
