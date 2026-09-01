from glob import glob
import os

from setuptools import find_packages, setup


package_name = "sanitation_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=[
        "setuptools",
        "PyYAML",
        "numpy>=1.24,<2",
    ],
    zip_safe=True,
    maintainer="Sanitation Vehicle Team",
    maintainer_email="team@example.com",
    description="Registry-driven garbage perception and tracking.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "garbage_perception_node = sanitation_perception.perception_node:main",
            "stage5a_backend_probe = sanitation_perception.backend_probe:main",
            "formal_perception_preflight = sanitation_perception.formal_contract:main",
            "pc_open_vocab_product_adapter = sanitation_perception.pc_open_vocab_adapter:main",
            "open_vocab_product_adapter = sanitation_perception.s100p_product_adapter:main",
            "rgb_to_nv12_adapter = sanitation_perception.rgb_to_nv12_adapter:main",
            "formal_random_scene_perception_evaluator = sanitation_perception.formal_random_scene_evaluator:main",
        ],
    },
)
