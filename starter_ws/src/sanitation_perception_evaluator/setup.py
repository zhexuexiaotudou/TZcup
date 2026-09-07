from setuptools import setup

setup(name="sanitation_perception_evaluator", version="0.1.0",
      packages=["sanitation_perception_evaluator"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/sanitation_perception_evaluator"]),
                  ("share/sanitation_perception_evaluator", ["package.xml"])],
      install_requires=["setuptools", "numpy>=1.24,<2", "PyYAML"], zip_safe=True,
      tests_require=["pytest"],
      maintainer="Sanitation Vehicle Team", maintainer_email="team@example.com", license="Apache-2.0",
      entry_points={"console_scripts": ["formal_random_scene_perception_evaluator = sanitation_perception_evaluator.formal_random_scene_evaluator:main"]})
