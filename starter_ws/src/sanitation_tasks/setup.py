from setuptools import find_packages, setup

package_name = "sanitation_tasks"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/demo_area.yaml",
            "config/mission_schema.json",
            "config/mapping_completion_route.json",
            "config/localization_route.json",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sanitation Vehicle Team",
    maintainer_email="team@example.com",
    description="Mission configuration and automated checks.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sanitation_smoke_check = sanitation_tasks.smoke_check:main",
            "sanitation_runtime_probe = sanitation_tasks.runtime_probe:main",
            "sanitation_navigation_probe = sanitation_tasks.navigation_probe:main",
            "sanitation_safety_probe = sanitation_tasks.safety_probe:main",
            "sanitation_ground_truth_adapter = sanitation_tasks.ground_truth_adapter:main",
            "sanitation_localization_evaluator = sanitation_tasks.localization_evaluator:main",
            "sanitation_mapping_probe = sanitation_tasks.mapping_probe:main",
            "sanitation_map_quality = sanitation_tasks.map_quality:main",
            "sanitation_image_capture = sanitation_tasks.image_capture:main",
            "sanitation_filter_probe = sanitation_tasks.filter_probe:main",
        ],
    },
)
