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
            "config/measurement_covariance.yaml",
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
            "sanitation_ground_truth_identity_probe = sanitation_tasks.ground_truth_identity_probe:main",
            "sanitation_motion_calibration_runner = sanitation_tasks.motion_calibration_runner:main",
            "sanitation_motion_fit_probe = sanitation_tasks.motion_fit_probe:main",
            "sanitation_localization_evaluator = sanitation_tasks.localization_evaluator:main",
            "sanitation_mapping_probe = sanitation_tasks.mapping_probe:main",
            "sanitation_map_quality = sanitation_tasks.map_quality:main",
            "sanitation_image_capture = sanitation_tasks.image_capture:main",
            "sanitation_filter_probe = sanitation_tasks.filter_probe:main",
            "sanitation_measurement_adapter = sanitation_tasks.measurement_adapter:main",
            "sanitation_covariance_audit = sanitation_tasks.covariance_audit:main",
            "sanitation_transient_response_runner = sanitation_tasks.transient_response_runner:main",
            "sanitation_operational_envelope_audit = sanitation_tasks.operational_envelope_audit:main",
            "sanitation_oracle_odom_adapter = sanitation_tasks.oracle_odom_adapter:main",
            "sanitation_dynamic_obstacle_probe = sanitation_tasks.dynamic_obstacle_probe:main",
        ],
    },
)
