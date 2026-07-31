from setuptools import find_packages, setup

package_name = "sanitation_hmi"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/web",
            ["web/index.html", "web/demo.html"],
        ),
        (
            "share/" + package_name + "/rviz",
            ["rviz/visual_demo.rviz"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zhexu",
    maintainer_email="zhexu@example.com",
    description="Authenticated HMI and constrained task DSL.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sanitation_hmi_server = sanitation_hmi.server:main",
            "sanitation_live_dashboard = sanitation_hmi.live_server:main",
        ]
    },
)
