from glob import glob
import os

from setuptools import find_packages, setup


package_name = "journey6_hil_gateway"

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
    description="Fail-closed PC/Journey 6 split HIL command gateway.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "journey6_hil_gateway = journey6_hil_gateway.gateway_node:main",
            "journey6_loopback_harness = journey6_hil_gateway.loopback_harness_node:main",
            "pc_onnx_algorithm_host = journey6_hil_gateway.pc_onnx_algorithm_node:main",
        ],
    },
)
