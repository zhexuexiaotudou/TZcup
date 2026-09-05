from setuptools import find_packages, setup

package_name = "sanitation_safety"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/operational_envelopes.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sanitation Vehicle Team",
    maintainer_email="team@example.com",
    description="Fail-closed velocity and whole-vehicle actuator safety gateways.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "velocity_gate = sanitation_safety.velocity_gate:main",
            "whole_vehicle_safety_manager = "
            "sanitation_safety.whole_vehicle_safety_manager:main",
            "simulation_safety_inputs = "
            "sanitation_safety.simulation_safety_inputs:main",
            "service_drain_safety_manager = "
            "sanitation_safety.service_drain_manager:main",
        ],
    },
)
