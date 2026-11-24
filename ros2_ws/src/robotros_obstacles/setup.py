from setuptools import find_packages, setup

package_name = "robotros_obstacles"

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
    maintainer="RobotROS team",
    maintainer_email="todo@example.invalid",
    description="Obstacle handling of robotROS: scan_masker (softwarespezifikation.md section 8).",
    license="TODO",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["scan_masker = robotros_obstacles.scan_masker:main"]},
)
