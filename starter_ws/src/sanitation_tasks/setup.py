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
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sanitation Vehicle Team",
    maintainer_email="team@example.com",
    description="Mission configuration and automated checks.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sanitation_smoke_check = sanitation_tasks.smoke_check:main",
        ],
    },
)
