from setuptools import find_packages, setup


package_name = "sanitation_dataset"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="Sanitation Vehicle Team",
    maintainer_email="team@example.com",
    description="Synthetic RGB-D dataset and ONNX evaluation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stage5a_generate_dataset = sanitation_dataset.cli:generate_main",
            "stage5a_build_onnx = sanitation_dataset.cli:build_model_main",
            "stage5a_evaluate_onnx = sanitation_dataset.cli:evaluate_main",
        ],
    },
)
