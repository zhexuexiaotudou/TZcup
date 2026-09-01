from glob import glob
import os

from setuptools import find_packages, setup


package_name = "sanitation_formal_campus_integration"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Sanitation Vehicle Team",
    maintainer_email="team@example.com",
    description=(
        "Fail-closed integration between the formal skid-steer vehicle and "
        "legacy campus autonomy topics."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "formal-legacy-topic-adapter = "
            "sanitation_formal_campus_integration.topic_adapter:main",
            "formal-spawn-initializer = "
            "sanitation_formal_campus_integration.spawn_initializer:main",
            "formal-map-lifecycle-manager = "
            "sanitation_formal_campus_integration.map_lifecycle_manager:main",
            "formal-frontier-explorer = "
            "sanitation_formal_campus_integration.frontier_explorer:main",
            "formal-scan-self-filter = "
            "sanitation_formal_campus_integration.formal_scan_self_filter:main",
            "formal-saved-map-coverage-executor = "
            "sanitation_formal_campus_integration.saved_map_coverage_executor:main",
            "formal-dynamic-footprint-manager = "
            "sanitation_formal_campus_integration.dynamic_footprint_manager:main",
        ],
    },
)
