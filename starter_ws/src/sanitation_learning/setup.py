from glob import glob
import os

from setuptools import find_packages, setup


package_name = "sanitation_learning"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="Sanitation Vehicle Team",
    maintainer_email="team@example.com",
    description="Rendered dataset, learned model, and Stage5B evidence tooling.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stage5b_generate_assets = sanitation_learning.cli:generate_assets_main",
            "stage5b_generate_dataset = sanitation_learning.cli:generate_dataset_main",
            "stage5b_train_models = sanitation_learning.cli:train_models_main",
            "stage5b_evaluate_models = sanitation_learning.cli:evaluate_models_main",
            "stage5b_j6_preflight = sanitation_learning.cli:j6_preflight_main",
        ],
    },
)
