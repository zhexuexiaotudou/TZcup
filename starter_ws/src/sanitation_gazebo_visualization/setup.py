from setuptools import find_packages, setup


package_name = "sanitation_gazebo_visualization"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sanitation Vehicle Team",
    maintainer_email="team@example.com",
    description="Gazebo-native cleaning trail, route, and mission status visualization.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cleaning_visualizer = sanitation_gazebo_visualization.cleaning_visualizer:main",
        ],
    },
)
